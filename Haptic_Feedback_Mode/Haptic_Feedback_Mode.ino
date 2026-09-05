#include <SimpleFOC.h>

// Hardware instances
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(1, 2, 4, 5); // ESP32-S3 Pins (IN1, IN2, IN3, EN)

// Haptic Modes
enum HapticMode {
  FREE_SPIN = 0,
  SPRING = 1,
  DETENTS = 2,
  FRICTION = 3,
  VIRTUAL_WALLS = 4
};

HapticMode currentMode = DETENTS;

void printMenu() {
  Serial.println("\n=== Haptic Feedback Control ===");
  Serial.println("Enter number to select mode:");
  Serial.println("0: Free Spin (No force)");
  Serial.println("1: Spring (Return-to-Center)");
  Serial.println("2: Detents / Ratchet (Clicky knob)");
  Serial.println("3: Friction / Damping (Viscous drag)");
  Serial.println("4: Virtual Walls (Hard stops at -90 / +90 deg)");
  Serial.println("================================\n");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9); // SDA = 8, SCL = 9 for ESP32-S3
  Wire.setClock(400000);

  // Initialize AS5600 Sensor
  sensor.init();
  motor.linkSensor(&sensor);

  // Initialize Driver
  driver.voltage_power_supply = 12; // Power supply voltage
  driver.init();
  motor.linkDriver(&driver);

  // Configure Motor for Torque (Voltage) Mode
  // motor.torque_controller_type = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 2.5; // Max voltage limit to protect 2804 motor

  // Velocity filter for smooth friction/damping calculations
  motor.LPF_velocity.Tf = 0.02;

  motor.init();
  motor.initFOC();

  printMenu();
}

void loop() {
  // SimpleFOC core execution loop
  motor.loopFOC();

  // Read non-blocking commands from Serial Monitor
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch >= '0' && ch <= '4') {
      currentMode = (HapticMode)(ch - '0');
      Serial.print("Active Mode: ");
      Serial.println(currentMode);
    }
  }

  float target_voltage = 0.0;
  float angle = motor.shaft_angle;
  float velocity = motor.shaft_velocity;

  // Haptic logic algorithms
  switch (currentMode) {
    case FREE_SPIN:
      target_voltage = 0.0;
      break;

    case SPRING: {
      // Pulls back to zero position
      float K_spring = 2.5; 
      target_voltage = -K_spring * angle;
      break;
    }

    case DETENTS: {
      // Detents spaced every 30 degrees (0.5235 radians)
      float step_rad = 0.5235; 
      float K_detent = 3.5;
      float nearest_step = round(angle / step_rad) * step_rad;
      target_voltage = -K_detent * (angle - nearest_step);
      break;
    }

    case FRICTION: {
      // Opposes rotation velocity (viscous damping)
      float K_damping = 0.15;
      target_voltage = -K_damping * velocity;
      break;
    }

    case VIRTUAL_WALLS: {
      // Hard bounds at -90 degrees (-1.57 rad) and +90 degrees (+1.57 rad)
      float wall_limit = 1.57; 
      float K_wall = 6.0;
      float K_damp = 0.05; // Damping to prevent wall bouncing

      if (angle > wall_limit) {
        target_voltage = -K_wall * (angle - wall_limit) - K_damp * velocity;
      } else if (angle < -wall_limit) {
        target_voltage = -K_wall * (angle - (-wall_limit)) - K_damp * velocity;
      } else {
        target_voltage = 0.0; // Free movement within limits
      }
      break;
    }
  }

  // Safety clamp target voltage
  target_voltage = _constrain(target_voltage, -motor.voltage_limit, motor.voltage_limit);

  // Apply torque voltage to motor
  motor.move(target_voltage);
}