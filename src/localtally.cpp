#include "main.h"

#ifdef HAS_PIXEL
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(256, PixelPin);
#endif

unsigned long tallyLast = 0;
static bool tallyCleared = false;
static unsigned long statusLast = 0;

static uint8_t tallyState[TALLY_MAX_NUM];
static uint8_t tallyBrightness[TALLY_MAX_NUM];
static bool tally_changed = false;
static char tallyText[100];

void localtally_setup() {
    memset(tallyState, 0, sizeof(tallyState));
#ifdef HAS_PIXEL
    pinMode(PixelPin, OUTPUT);
    strip.Begin();
#endif
}

void sendStatus(void) {
    DynamicJsonDocument json(128);
    String output;
    json["cmd"] = "STATUS";
    json["version"] = VERSION_STR "-" PLATFORM_STR "-" BUILD_STR;
    json["uptime"] = millis()/1000;
    json["battPercent"] = battVoltToPercent(get_batt_volt());
    json["battVolt"] = get_batt_volt();
    json["LoRaMsgCnt"] = LoRaGetMsgCnt();
    json["LoRaRSSI"] = LoRaGetRSSI();

    serializeJson(json, output);
    ws2All(output.c_str());
}

void sendTally(void) {
    DynamicJsonDocument json(128);
    String output;
    uint8_t stateRH, brightnessRH;
    uint8_t stateLH, brightnessLH;
    if (getTallyState(cfg.tally_id|TALLY_RH, stateRH, brightnessRH) && getTallyState(cfg.tally_id|TALLY_LH, stateLH, brightnessLH)) {
        json["cmd"] = "TALLY";
        json["stateLH"] = stateLH;
        json["brightnessLH"] = brightnessLH;
        json["stateRH"] = stateRH;
        json["brightnessRH"] = brightnessRH;
        json["text"] = tallyText;
        serializeJson(json, output);
        ws2All(output.c_str());
    }
}

void rgbFromTSL(int &r, int &g, int &b, int state, int brightness, bool atem) {
    brightness++;
    r = g = b = 0;
    if (atem && state == 3) {
        state = 1;
    }
    switch (state) {
        case 1:
            r = 63 * brightness;
            break;
        case 2:
            g = 63 * brightness;
            break;
        case 3:
            r = 63 * brightness;
            g = 22 * brightness;
            break;
    }
}

bool setTallyState(int index, uint8_t state, uint8_t brightness, char *text) {
    if (index < TALLY_MAX_NUM) {
        if (index == cfg.tally_id) {
            if (text) {
                strncpy(tallyText, text, sizeof(tallyText) - 1);
            } else {
                tallyText[0] = 0;
            }
        }
        if (tallyState[index - 1] != state ||
            tallyBrightness[index - 1] != brightness) {
            tallyState[index - 1] = state;
            tallyBrightness[index - 1] = brightness;
            tally_changed = true;           
        }
    }
    return tally_changed;
}

bool getTallyState(int index, uint8_t &state, uint8_t &brightness) {
    if (index >0 && index < TALLY_MAX_NUM) {
        state = tallyState[index - 1];
        brightness = tallyBrightness[index - 1];
        return true;
    }
    return false;
}

bool getTallyChanged(bool clear) {
    bool val = tally_changed;
    if (clear) {
        tally_changed = false;
    }
    return val;
}

// Function to get noise floor
int16_t LoRaGetNoiseFloor() {
  // Get RSSI and SNR from the last received packet
  int16_t rssi = LoRaGetRSSI(); // dBm
  int16_t snr = LoRaGetSNR();     // dB

  // Check if SNR is valid (e.g., packet was recently received)
  // SNR typically ranges from -20 to +10 dB for LoRa
  if (snr >= -20.0 && snr <= 20.0) {
    int16_t noiseFloor = rssi - snr; // Noise Floor = RSSI - SNR
    // Constrain to reasonable range for LoRa (e.g., -130 to -100 dBm)
    return constrain(noiseFloor, -130, -100);
  } else {
    // Fallback to typical LoRa noise floor if SNR is invalid
    return -125; // Typical for SF12, 125 kHz bandwidth
  }
}

// Function to display LoRa signal strength as a bar graph with noise floor
void displayLoRaSignalStrength() {
  // Get RSSI and noise floor values (assumed to be provided by LoRa library)
  int16_t rssi = LoRaGetRSSI(); // Returns dBm, e.g., -120 to 0
  int16_t snr = LoRaGetSNR(); // Returns dB, e.g., -20 to +10
  int16_t noiseFloor = LoRaGetNoiseFloor(); // Returns dBm, e.g., -125 to -110

  // Set text alignment and font for RSSI value display
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  // Logarithmic mapping for RSSI to number of bars
  const int16_t minRssi = -120; // Minimum RSSI (weakest signal)
  const int16_t maxRssi = -30;  // Maximum RSSI (strongest signal)
  const int numBars = 9;        // Number of bar segments
  const float curveFactor = 20.0; // Controls steepness of logarithmic curve (1.0 to 10.0, higher = steeper)

  // Normalize RSSI to 0-1 range
  float rssiNormalized = (float)(rssi - minRssi) / (maxRssi - minRssi);
  rssiNormalized = constrain(rssiNormalized, 0.0, 1.0); // Ensure within bounds

  // Logarithmic scaling with adjustable curve
  // Use log10(1 + curveFactor * x) to control steepness
  float logScaled = log10(1 + curveFactor * rssiNormalized);
  int bars = round(logScaled / log10(1 + curveFactor) * numBars); // Map to 0-9 bars
  bars = constrain(bars, 0, numBars); // Ensure bars stay within range

  // Bar graph parameters
  const int barHeight = 5;   // Height of each bar
  const int barSpacing = 1;  // Spacing between bars
  const int xOffset = 2;     // X position of RSSI bar graph (shifted right to accommodate noise floor)
  const int yOffset = 1;     // Y position of the bar graph (top of display)
  const int noiseBarWidth = 1; // Width of noise floor bar
  const int noiseBarX = 0;   // X position of noise floor bar (leftmost edge)

  // Map noise floor to bar height (same vertical scale as RSSI bars)
  const int totalHeight = numBars * (barHeight + barSpacing) - barSpacing; // 54 pixels (9 * (5+1) - 1)
  int noiseHeight = map(noiseFloor, minRssi, maxRssi, 0, totalHeight); // Linear mapping for noise floor
  noiseHeight = constrain(noiseHeight, 0, totalHeight);

  // Draw noise floor bar (outlined to distinguish from RSSI bars)
  display.drawRect(noiseBarX, yOffset + (totalHeight - noiseHeight), noiseBarWidth, noiseHeight);

  // Draw the RSSI bar graph
  for (int i = 0; i < numBars; i++) {
    int y = yOffset + (numBars - 1 - i) * (barHeight + barSpacing); // Draw from bottom up
    int barWidth = 4 + i; // Bar width increases from 4 to 12 pixels
    if (i < bars) {
      // Filled bar for signal strength
      display.fillRect(xOffset, y, barWidth, barHeight);
    } else {
      // Empty bar outline
      display.drawRect(xOffset, y, barWidth, barHeight);
    }
  }

  String rssiText = "X";

  // Display the RSSI and SNR values at the bottom
  if (millis() - tallyLast > 5000) {
    rssiText = "X";
  } else {
    rssiText = String(rssi) + "dBm"; //, SNR " + String(snr) + "dB";
  }
  
  display.drawString(0, 64 - 10, rssiText); // 64 is display height, 10 accounts for font height
}

// Function to display battery indicator
void displayBatteryIndicator() {
  // Get battery values (assumed to be provided by get_batt_volt() and percent)
  int16_t battVoltage = get_batt_volt(); // Returns mV, e.g., 4200 for 4.2V
  uint8_t percent = battVoltToPercent(get_batt_volt());
  uint8_t battPercent = percent; // Battery percentage, 0-100

  // Set text alignment and font for text display
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  // Battery indicator parameters (right side)
  const int battX = 128 - 9; // X position, 12 pixels from right edge
  const int battY = 1;        // Y position, top of display
  const int battWidth = 8;    // Width of battery bar
  const int battMaxHeight = 50; // Maximum height of battery bar
  const int battTipHeight = 3;  // Height of battery "tip" (nub at top)

  // Map battery percentage to bar height
  int battHeight = map(battPercent, 0, 100, 0, battMaxHeight);
  battHeight = constrain(battHeight, 0, battMaxHeight);

  // Draw battery outline (full capacity)
  display.drawRect(battX, battY + battTipHeight, battWidth, battMaxHeight);
  // Draw battery tip (nub at top)
  display.drawRect(battX + 2, battY, battWidth - 4, battTipHeight);
  
  // Draw filled battery bar (proportional to percentage)
  display.fillRect(battX, battY + battTipHeight + (battMaxHeight - battHeight), battWidth, battHeight);

  // Display battery voltage and percentage at the bottom
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  String battText = String(battVoltage / 1000.0, 1) + "V " + String(battPercent) + "%";
  display.drawString(128, 64 - 10, battText); // Centered at bottom
}

void setTallyLight(int r, int g, int b, dispMode_t disp, int pixel,
                   char *text) {
    tallyLast = millis();

    if (r || g || b) {
        tallyCleared = false;
    }

    r = r * cfg.led_max_brightness / 255;
    g = g * cfg.led_max_brightness / 255;
    b = b * cfg.led_max_brightness / 255;
#ifdef HAS_PIXEL
    if (pixel == 0) {
        for (int i = 0; i < cfg.num_pixels; i++) {
            strip.SetPixelColor(i, RgbColor(r, g, b));
        }
    } else {
        if (pixel <= cfg.num_pixels)
            strip.SetPixelColor(pixel - 1, RgbColor(r, g, b));
    }
    strip.Show();
#endif    
#ifdef HAS_DISPLAY
    if (disp > DISP_OFF) {
        uint8_t percent = battVoltToPercent(get_batt_volt());
        int16_t snr = LoRaGetSNR();
        if (text != NULL && strlen(text)) {
            display.clear();
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.setFont(ArialMT_Plain_24);
            display.drawString(64, 20, text);
            display.setFont(ArialMT_Plain_10);
            //display.drawString(64, 54, "Batt: " + String(get_batt_volt()/1000.0, 1) + "V / " + String(percent)+"%");
            displayLoRaSignalStrength();
            displayBatteryIndicator();
            d();
        } else if (r | g | b) {
            display.clear();
            if (r) {
                display.invertDisplay();
            } else {
                display.normalDisplay();
            }
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.setFont(ArialMT_Plain_24);
            display.drawString(64, 20, "CAM " + String(cfg.tally_id));
            display.setFont(ArialMT_Plain_10);
            if (r) {
                display.drawString(64, 0, "PROGRAM");
            } else if (g) {
                display.drawString(64, 0, "PREVIEW");
            } else {
                display.drawString(64, 0, "OFF");
            }
            display.drawString(64, 10, "SNR:" + String(snr) + "dB");
            //display.drawString(64, 44, "Batt: " + String(get_batt_volt()/1000.0, 1) + "V / " + String(percent)+"%");
            //display.drawString(64, 55, "WiFi:" + String(WiFi.RSSI()) + "dBm");
            display.drawString(64, 44, "WiFi: " + String(WiFi.RSSI()) + "dBm");
            

            displayLoRaSignalStrength();
            displayBatteryIndicator();
            d();
        } else {
            display.normalDisplay();
            display.clear();
            displayLoRaSignalStrength();
            displayBatteryIndicator();
            d();
        }
        /*
                    display.drawString(64, 14, "Red  : " + String(r));
                    display.drawString(64, 24, "Green: " + String(g));
                    display.drawString(64, 34, "Blue : " + String(b));
        */
        if (disp == DISP_RSSI) {
            //display.setFont(ArialMT_Plain_10);
            //display.drawString(64, 44, "RSSI : " + String(LoRaGetRSSI()));
            displayLoRaSignalStrength();
            displayBatteryIndicator();
            d();
        }
    }
#endif
}

void setTallyLight(int tally_id, dispMode_t disp, int pixel, char *text) {
    uint8_t state, brightness;
    if (pixel & 1) {
        tally_id |= TALLY_RH;
    } else {
        tally_id |= TALLY_LH;
    }
    if (getTallyState(tally_id, state, brightness)) {
        int r, g, b;
        rgbFromTSL(r, g, b, state, brightness);
        setTallyLight(r, g, b, disp, pixel, text);
    }
}

void tally_loop() {
    if (getTallyChanged()) {
        if(cfg.tally_id > 0 && cfg.tally_id < (TALLY_MAX_NUM/2)) {
            setTallyLight(cfg.tally_id, DISP_ON, 1, tallyText);
            setTallyLight(cfg.tally_id, DISP_OFF, 2);
            setTallyLight(cfg.tally_id, DISP_OFF, 3);
            setTallyLight(cfg.tally_id, DISP_OFF, 4);
            sendTally();
        }
    }

    if (cfg.tally_timeout > 1000 && (millis() - tallyLast > cfg.tally_timeout) && !tallyCleared) {
        setTallyLight(0, 0, 0, DISP_OFF);
        tallyCleared = true;
    }
    if ((millis() - statusLast) > 1000) {
        statusLast = millis();
        sendStatus();
    }
}
