#include <SimpleFOC.h>
#include "DisplayUI.h"

DisplayUI display;

// Hardware setup
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(7); // 2804 motor (7 pole pairs)
BLDCDriver3PWM driver = BLDCDriver3PWM(1, 2, 4, 5); // ESP32-S3 Pins (IN1, IN2, IN3, EN)

enum SmartKnobMode {
  UNBOUNDED_NO_DETENTS = 1,
  BOUNDED_0_10_NO_DETENTS = 2,
  MULTI_REV_NO_DETENTS = 3,
  ON_OFF_STRONG_DETENT = 4,
  RETURN_TO_CENTER = 5,
  FINE_NO_DETENTS = 6,
  FINE_WITH_DETENTS = 7,
  COARSE_STRONG_DETENTS = 8,
  COARSE_WEAK_DETENTS = 9
};

SmartKnobMode currentMode = UNBOUNDED_NO_DETENTS;

// Reusable virtual wall function (restoring spring + damping outside limits)
float computeWallTorque(float k_angle, float k_vel, float min_k, float max_k) {
  float K_wall = 8.0;  
  float K_damp = 0.08; 
  if (k_angle > max_k) return K_wall * (k_angle - max_k) + K_damp * k_vel;
  if (k_angle < min_k) return K_wall * (k_angle - min_k) + K_damp * k_vel;
  return 0.0;
}

// Reusable detent generator
float computeDetentTorque(float k_angle, float k_vel, float step_rad, float snap_k, float damp_k) {
  float nearest_step = round(k_angle / step_rad) * step_rad;
  float error = k_angle - nearest_step;
  return snap_k * error + damp_k * k_vel;
}

void printMenu() {
  Serial.println("\n=== SmartKnob Haptic Modes ===");
  Serial.println("1: Unbounded, No detents");
  Serial.println("2: Bounded 0-10, No detents");
  Serial.println("3: Multi-rev (-2 to +2 revs), No detents");
  Serial.println("4: On/Off, Strong detent");
  Serial.println("5: Return-to-center");
  Serial.println("6: Fine values, No detents (High viscous drag)");
  Serial.println("7: Fine values, With detents (5 deg spacing)");
  Serial.println("8: Coarse values, Strong detents (30 deg spacing)");
  Serial.println("9: Coarse values, Weak detents (30 deg spacing)");
  Serial.println("===============================\n");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9); // ESP32-S3: SDA=8, SCL=9

  // Initialize display
  display.begin();

  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // motor.torque_controller_type = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = 2.5; // Max output voltage for 2804 motor
  motor.LPF_velocity.Tf = 0.02; // Velocity filter constant

  motor.init();
  motor.initFOC();

  // --- Smooth Return to Center (0.0 rad) ---
  Serial.println("Centering motor to 0.0...");
  unsigned long start_time = millis();
  
  while (millis() - start_time < 2500) { // 2.5 second timeout safety
    motor.loopFOC();

    float knobAngle = -motor.shaft_angle;
    float knobVel = -motor.shaft_velocity;

    // Apply proportional spring force to pull back to 0.0
    float target_voltage = 3.0 * knobAngle + 0.05 * knobVel;
    target_voltage = _constrain(target_voltage, -motor.voltage_limit, motor.voltage_limit);
    motor.move(target_voltage);

    // Exit early if settled within 1 degree of center
    if (abs(knobAngle) < 0.02 && abs(knobVel) < 0.1) {
      break;
    }
  }

  printMenu();
}

void loop() {
  motor.loopFOC();

  // Read non-blocking menu selections from Serial Monitor
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch >= '1' && ch <= '9') {
      currentMode = (SmartKnobMode)(ch - '0');
      Serial.print("Switched to Mode ");
      Serial.println((int)currentMode);
    }
  }

  // Unified CW-Positive coordinate system (CW = positive, CCW = negative)
  float knobAngle = -motor.shaft_angle;
  float knobVel = -motor.shaft_velocity;
  float target_voltage = 0.0;

  float displayValue = knobAngle;
  const char* modeLabel = "UNBOUNDED";

  switch (currentMode) {

    // 1. Unbounded, No detents
    case UNBOUNDED_NO_DETENTS:
      target_voltage = 0.01 * knobVel; // Minimal drag for smooth continuous spinning

      displayValue = knobAngle;
      modeLabel = "UNBOUNDED";
      break;

    // 2. Bounded 0-10, No detents (Spans 1 full revolutions1)
    case BOUNDED_0_10_NO_DETENTS: {
      float min_value = 0.0, max_value = 10.0;
      float min_k = 0.0, max_k = 2 * _PI;
      target_voltage = computeWallTorque(knobAngle, knobVel, min_k, max_k);
      if (knobAngle >= min_k && knobAngle <= max_k) target_voltage = 0.02 * knobVel;
      
      displayValue = _constrain(knobAngle / (2*_PI) * (max_value-min_value), min_value, max_value);
      modeLabel = "0 - 10";
      break;
    }

    // 3. Multi-rev, No detents (-2 revolutions to +2 revolutions)
    case MULTI_REV_NO_DETENTS: {
      float limit = 4.0 * _PI; // 2 full revolutions = 4 * PI
      target_voltage = computeWallTorque(knobAngle, knobVel, -limit, limit);
      if (knobAngle >= -limit && knobAngle <= limit) target_voltage = 0.01 * knobVel;

      displayValue = knobAngle / (2*_PI); // Display in total revolutions
      modeLabel = "2REV SMOOTH";
      break;
    }

    // 4. On/Off switch, Strong detent (Positions at 0 and ~45 degrees)
    case ON_OFF_STRONG_DETENT: {
      float off_pos = 0.0, on_pos = 0.8;
      target_voltage = computeWallTorque(knobAngle, knobVel, off_pos, on_pos);
      if (knobAngle >= off_pos && knobAngle <= on_pos) {
        float mid = (off_pos + on_pos) / 2.0;
        float target = (knobAngle < mid) ? off_pos : on_pos;
        target_voltage = 7.0 * (knobAngle - target) + 0.05 * knobVel;
      }

      displayValue = (knobAngle > 0.4) ? 1.0 : 0.0;
      modeLabel = "ON / OFF";
      break;
    }

    // 5. Return-to-center spring
    case RETURN_TO_CENTER:
      target_voltage = 2.5 * knobAngle + 0.05 * knobVel;

      displayValue = knobAngle;
      modeLabel = "Center";
      break;

    // 6. Fine values, No detents (Heavy viscous friction for micro-adjustment)
    case FINE_NO_DETENTS:
      target_voltage = 0.20 * knobVel; // Heavy drag prevents overshooting

      displayValue = knobAngle;
      modeLabel = "FINE SMOOTH";
      break;

    // 7. Fine values, With detents (5-degree / 0.087 rad steps, light tick)
    case FINE_WITH_DETENTS: {
      float step_5deg = 0.087;
      target_voltage = computeDetentTorque(knobAngle, knobVel, step_5deg, 2.0, 0.03);

      displayValue = round(knobAngle / 0.087);
      modeLabel = "FINE STEP";
      break;
    }

    // 8. Coarse values, Strong detents (30-degree / 0.523 rad steps, tactile snap)
    case COARSE_STRONG_DETENTS: {
      float step_30deg = 0.523;
      target_voltage = computeDetentTorque(knobAngle, knobVel, step_30deg, 6.0, 0.05);

      displayValue = round(knobAngle / 0.523);
      modeLabel = "COARSE STRONG";
      break;
    }

    // 9. Coarse values, Weak detents (30-degree / 0.523 rad steps, soft bump)
    case COARSE_WEAK_DETENTS: {
      float step_30deg = 0.523;
      target_voltage = computeDetentTorque(knobAngle, knobVel, step_30deg, 1.8, 0.02);

      displayValue = round(knobAngle / 0.523);
      modeLabel = "COARSE WEAK";
      break;
    }
  }

  // Constrain voltage to protect motor and driver
  target_voltage = _constrain(target_voltage, -motor.voltage_limit, motor.voltage_limit);
  motor.move(target_voltage);

  // Update Display
  display.update((int)currentMode, modeLabel, displayValue, knobAngle);
}