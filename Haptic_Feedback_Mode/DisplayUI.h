#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include <Arduino_GFX_Library.h>

// ESP32-S3 Hardware Pin Mapping for GC9A01 Display
#define TFT_CS   10
#define TFT_DC   11
#define TFT_RST  12
#define TFT_SCL  13 // SCK
#define TFT_SDA  14 // MOSI

class DisplayUI {
private:
  Arduino_DataBus *bus;
  Arduino_GFX     *gfx;

  int prev_dot_x = 120;
  int prev_dot_y = 120;
  int prev_mode = -1;
  float prev_val = -999.0;

  const int CENTER_X = 120;
  const int CENTER_Y = 120;
  const int RADIUS_DOT = 102; // Edge distance for absolute position indicator
  const int DOT_RADIUS = 6;   // Dot radius in pixels

public:
  DisplayUI() {
    bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCL, TFT_SDA, GFX_NOT_DEFINED);
    gfx = new Arduino_GC9A01(bus, TFT_RST, 0 /* rotation */, true /* IPS panel */);
  }

  ~DisplayUI() {
    delete gfx;
    delete bus;
  }

  void begin() {
    gfx->begin();
    gfx->fillScreen(RGB565_BLACK);

    // Static circular boundary ring
    gfx->drawCircle(CENTER_X, CENTER_Y, RADIUS_DOT + DOT_RADIUS + 2, RGB565_BLUE);
  }

  void update(int modeNum, const char* modeName, float currentValue, float rawAngleRad) {
    // 1. Compute dot position along outer edge (0 rad aligns to 12 o'clock)
    float dotAngle = rawAngleRad - (M_PI / 2.0);
    int dot_x = CENTER_X + (int)(RADIUS_DOT * cos(dotAngle));
    int dot_y = CENTER_Y + (int)(RADIUS_DOT * sin(dotAngle));

    // 2. Move position dot only when physical shaft angle changes
    if (dot_x != prev_dot_x || dot_y != prev_dot_y) {
      gfx->fillCircle(prev_dot_x, prev_dot_y, DOT_RADIUS, RGB565_BLACK);
      gfx->fillCircle(dot_x, dot_y, DOT_RADIUS, RGB565_CYAN);
      prev_dot_x = dot_x;
      prev_dot_y = dot_y;
    }

    // 3. Refresh text box selectively to avoid screen flicker
    if (modeNum != prev_mode || abs(currentValue - prev_val) >= 0.1) {
      gfx->fillRect(35, 75, 170, 90, RGB565_BLACK); // Clear text box region

      // Mode Title
      gfx->setTextColor(RGB565_YELLOW);
      gfx->setTextSize(2);
      gfx->setCursor(45, 80);
      gfx->print(modeName);

      // Value Reading
      gfx->setTextColor(RGB565_WHITE);
      gfx->setTextSize(3);
      gfx->setCursor(70, 120);
      gfx->print(currentValue, 1);

      prev_mode = modeNum;
      prev_val = currentValue;
    }
  }
};

#endif