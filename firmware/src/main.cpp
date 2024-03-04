/*
 * Copyright 2023 spezifisch <spezifisch23@proton.me> https://github.com/spezifisch
 *
 * This file is part of T12PenSolder.
 *
 * T12PenSolder is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, version 3 of the
 * License.
 *
 * T12PenSolder is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with Foobar. If not, see
 * <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <U8g2lib.h>

#include "config.h"
#include "cube_init.h"
#include "debounce.h"
#include "solderingtip.h"

// display
U8G2_SSD1306_128X32_UNIVISION_2_SW_I2C u8g2(U8G2_R0, /* clock=*/SCL_PIN, /* data=*/SDA_PIN, /* reset=*/U8X8_PIN_NONE);

// yh for button set long pressed
unsigned long buttonPressTime = 0; // Time when the button was pressed
bool longPressDetected = false; // Flag to indicate a long press
bool inStandby;
int32_t tt_missing_prev = 1; // init as tip missing
int32_t tt_missing = 1;
// Variable to store motion detection state
volatile bool motionDetected = false;
volatile unsigned long lastMotionTime = 0;

#define Untitled_width 128
#define Untitled_height 32
static unsigned char Untitled_bits[] = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c,
   0x73, 0x88, 0x39, 0x46, 0x00, 0xc0, 0x07, 0x00, 0xe0, 0x03, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x94, 0x54, 0x4c, 0x22, 0x69, 0x00, 0x20, 0x0f, 0x00,
   0x70, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x44, 0x46, 0x3a, 0x49,
   0x00, 0x30, 0x0e, 0x00, 0x38, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88,
   0x24, 0x4a, 0x22, 0x49, 0x00, 0x30, 0x1c, 0x00, 0x1c, 0x0e, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x84, 0x16, 0x4f, 0x23, 0x4d, 0x00, 0xb0, 0x39, 0x00,
   0xfe, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9e, 0x7b, 0xc8, 0x3d, 0xe7,
   0x00, 0xb8, 0xfb, 0xff, 0xff, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xd8, 0x07, 0x00, 0xf8, 0x08, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x07, 0x00,
   0xf8, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xc8, 0x07, 0x00, 0xf0, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xc8, 0x01, 0x00, 0xc0, 0x08, 0x00, 0x00,
   0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00,
   0x00, 0x18, 0x00, 0x80, 0x83, 0x03, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x08, 0x00, 0x00, 0x00, 0x38, 0x00, 0x40, 0x00, 0x04, 0x40, 0x00,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x04, 0x3f, 0x80, 0x3f, 0x30, 0x00, 0x20,
   0x00, 0x04, 0x20, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86, 0x00, 0x00,
   0x40, 0x30, 0x00, 0x20, 0x00, 0x04, 0x10, 0x40, 0x01, 0x00, 0x00, 0x00,
   0x00, 0x06, 0x00, 0x00, 0x00, 0x30, 0x00, 0x20, 0x00, 0x0c, 0x18, 0xc0,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x3c, 0x00, 0x0e, 0x30, 0x00, 0x20,
   0x00, 0x04, 0x18, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x7c, 0x00,
   0x1f, 0x70, 0x00, 0x20, 0x00, 0x06, 0x4c, 0x20, 0x00, 0x00, 0x00, 0x00,
   0xe0, 0x07, 0xfa, 0x80, 0x27, 0xf0, 0x03, 0x40, 0x00, 0x02, 0x66, 0x10,
   0x24, 0x00, 0x00, 0x00, 0x00, 0x1e, 0xfa, 0x80, 0x27, 0x3c, 0x00, 0x80,
   0x03, 0x03, 0x23, 0x58, 0x23, 0x66, 0x00, 0x00, 0x00, 0x1f, 0xfe, 0x80,
   0x3f, 0x7c, 0x00, 0x00, 0x00, 0x81, 0x91, 0x4d, 0x19, 0x11, 0x01, 0x00,
   0xe0, 0x07, 0xfe, 0x80, 0x3f, 0xf0, 0x07, 0x00, 0x80, 0x81, 0xf8, 0x8e,
   0xf9, 0x8e, 0x00, 0x00, 0x00, 0x1e, 0x7c, 0x1c, 0x1f, 0x7c, 0x00, 0x00,
   0x80, 0x40, 0x30, 0x45, 0x77, 0x57, 0x00, 0x00, 0xe0, 0x1f, 0x00, 0x1c,
   0x00, 0xfc, 0x03, 0x00, 0x80, 0xfc, 0x00, 0x38, 0x00, 0x38, 0x00, 0x00,
   0xe0, 0x09, 0x00, 0x08, 0x00, 0xcc, 0x07, 0x00, 0x80, 0x65, 0x00, 0x10,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f, 0x00, 0xfe, 0x0f, 0x00, 0x00,
   0x80, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f, 0x00,
   0xff, 0x0f, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x08, 0xc0, 0x80, 0x01, 0x08, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xc0, 0xff, 0x01, 0x08, 0x00, 0x00,
   0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xc0, 0xc1,
   0x01, 0x08, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x88, 0xcd, 0x80, 0xb1, 0x05, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x3f, 0x00, 0xfe, 0x03, 0x00, 0x00,
   0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

void motionInterrupt() {
    motionDetected = digitalRead(motionDetectorPin) == HIGH;
    lastMotionTime = millis();
    motionDetected = true;
}

void setup()
{
    // lowlevel stm32cube setup
    MX_GPIO_Init();

    // turn heat off
    digitalWrite(TIPHEAT_DRV, 0);
    solderingTip.safeMode();

    // stock firmware does what looks like a display reset, but it isn't connected on my board
    // digitalWrite(PA9, 0);
    // delay(200);
    // digitalWrite(PA9, 1);
    delay(500); // keep some delay to be sure that display is up

    // adc setup
    pinMode(VIN_MEAS, INPUT_ANALOG);
    pinMode(TIPTEMP_MEAS, INPUT_ANALOG);
    analogReadResolution(12);
    // set sampling time to same as stock firmware
    MX_ADC_Config();
    // motion detector setup
    pinMode(motionDetectorPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(motionDetectorPin), motionInterrupt, CHANGE);

    // PWM setup
    solderingTip.setup();

    // yh 
    // put tip in ready position
    inStandby = true;
    solderingTip.setTargetTemperature(DEFAULT_TEMPERATURE_STANDBY_degC);

    pinMode(BUT_SET, INPUT_PULLUP);

    // display setup
    u8g2.begin();

    // set offset. Only for v7 board
    u8g2.sendF("ca", 0x0a8, 0x02f); 

    // splash screen
    u8g2.setFont(u8g2_font_5x8_mr);
    u8g2.firstPage();
    do
    {
        u8g2.drawXBM(0,0,128,32,Untitled_bits);
    } while (u8g2.nextPage());
    delay(3000);
}

void loop(void)
{
    static char tmp[64];

    // timekeeping
    static constexpr uint32_t UPDATE_PERIOD_ms = 100;                               // display update period
    static uint32_t last_millis = UPDATE_PERIOD_ms * (millis() / UPDATE_PERIOD_ms); // last display update
    static uint32_t last_showtime = 0;                                              // last display buffer transfer duration

    // buttons
    static constexpr uint32_t BUTTON_TEMPERATURE_STEP_K = 10; // temperature increment on button press
    static constexpr uint32_t BUTTON_DEBOUNCE_COUNT = 2;      // button state must be stable for this many display updates
    static Debounce<BUTTON_DEBOUNCE_COUNT> buttonSetDebounce, buttonMinusDebounce, buttonPlusDebounce;
    
    // runtime settings
    static bool heatOn = false;                                          // soldering tip enabled
    static uint32_t selectedTemperature_degC = DEFAULT_TEMPERATURE_degC; // target temp., change with -/+
    static uint32_t idle_time_ms = 0;

    // 10 Hz UI update
    const uint32_t now = millis();
    if (now - last_millis < UPDATE_PERIOD_ms)
    {
        return;
    }
    // update idle time counter
    if ((heatOn && idle_time_ms < DEFAULT_STANDBY_TIME_ms) 
        || (inStandby && idle_time_ms < DEFAULT_OFF_TIME_ms))
    {
        idle_time_ms += now - last_millis;
    }
    last_millis = now;

    // loop benchmark
    const uint32_t start = micros();

    // read and handle buttons
    const bool buttonSet = buttonSetDebounce.measure(digitalRead(BUT_SET));
    const bool buttonMinus = buttonMinusDebounce.measure(digitalRead(BUT_MINUS));
    const bool buttonPlus = buttonPlusDebounce.measure(digitalRead(BUT_PLUS));
    
    // detect long press button set
    int buttonState = digitalRead(BUT_SET);
    if (buttonState == HIGH) { // Button is pressed
        if (buttonPressTime == 0) {
        buttonPressTime = millis(); // Record the press time
        }
        
        if (millis() - buttonPressTime > DEBOUNCE_INTERVAL_ms && !longPressDetected) {
        longPressDetected = true; // Set the long press flag
        }
    } else { // Button is released
        if (buttonPressTime != 0) {
        buttonPressTime = 0; // Reset the press time
        longPressDetected = false; // Reset the long press flag
        }
    }

    // long press = Off
    if (longPressDetected) {
        u8g2.firstPage();
        do
        {
            u8g2.setFont(u8g2_font_7x14_tr);
            u8g2.drawStr(0,15,"Going off..!");        
        } while (u8g2.nextPage());
        inStandby = false;
        heatOn = false;
        solderingTip.safeMode(); 
        delay(2000);
    }

/* Removed - using auto to switch Active - Standby
    if (buttonSet)
    {
        if (!heatOn) {
            inStandby = false;
            heatOn = true;
            solderingTip.setTargetTemperature(selectedTemperature_degC); }
        else if (heatOn) {
            inStandby = true;
            heatOn = false;
            solderingTip.setTargetTemperature(DEFAULT_TEMPERATURE_STANDBY_degC);
        }
    }
*/

    if (buttonMinus)
    {
        selectedTemperature_degC -= BUTTON_TEMPERATURE_STEP_K;
        if (selectedTemperature_degC < SolderingTip::MIN_TARGET_TEMPERATURE_degC)
        {
            selectedTemperature_degC = SolderingTip::MIN_TARGET_TEMPERATURE_degC;
        }
        if (heatOn)
        {
            solderingTip.setTargetTemperature(selectedTemperature_degC);
        }

        buttonMinusDebounce.reset(); // repeat when holding button
    }

    if (buttonPlus)
    {
        selectedTemperature_degC += BUTTON_TEMPERATURE_STEP_K;
        if (selectedTemperature_degC > SolderingTip::MAX_TARGET_TEMPERATURE_degC)
        {
            selectedTemperature_degC = SolderingTip::MAX_TARGET_TEMPERATURE_degC;
        }
        heatOn = true;
        solderingTip.setTargetTemperature(selectedTemperature_degC);

        buttonPlusDebounce.reset(); // repeat
    }

    // reset idle time & active on any button press / motion detected
    if (buttonSet || buttonMinus || buttonPlus || motionDetected)
    {
        idle_time_ms = 0;
        heatOn = true;
        solderingTip.setTargetTemperature(selectedTemperature_degC);
    }

    // motion detector : only indicate no motion after 3s idle
    if (motionDetected && millis() - lastMotionTime > 3000) {
        motionDetected = false;
    }

    // get tip state
    const uint32_t vin_raw = solderingTip.getVinRaw();
    const uint32_t vin_mv_dec = solderingTip.getVinmV() % 1000;
    const uint32_t vin_v = solderingTip.getVinmV() / 1000;

    const uint32_t tt_raw = solderingTip.getTipTempRaw();
    const uint32_t tt_uv = solderingTip.getTipTempuV();
    const int32_t tt_degC = solderingTip.getTipTempDegC();
    
 
    tt_missing = solderingTip.getTipMissing();
    const int32_t t_pwm = solderingTip.getPWM();
    const int32_t t_output_w = solderingTip.getOutputW();

    // UI info
    const uint32_t timestamp = (now % 10000000) / 100;
    const uint32_t showtime_ms = last_showtime / 1000;
    uint32_t standby_counter_s = 0;

    // Standby mode if exceed standby_time
    if (heatOn && DEFAULT_STANDBY_TIME_ms > idle_time_ms)
    {
        standby_counter_s = (DEFAULT_STANDBY_TIME_ms - idle_time_ms) / 1000;
        if (standby_counter_s <= 0) {
        inStandby = true;
        heatOn = false;
        solderingTip.setTargetTemperature(DEFAULT_TEMPERATURE_STANDBY_degC);
        idle_time_ms = 0;
        }
    } 
    // Off mode if exceed Off time
    else if (inStandby && DEFAULT_OFF_TIME_ms > idle_time_ms)
    {
        standby_counter_s = (DEFAULT_OFF_TIME_ms - idle_time_ms) / 1000;
        if (standby_counter_s <= 0) {
        inStandby = false;
        heatOn = false;
        solderingTip.safeMode(); 
        idle_time_ms = 0;
        }
    }

    // yh - display output simple
    u8g2.firstPage();
    do
    {
        // smaller font
        u8g2.setFont(u8g2_font_7x14_tr);   

        // if tip missing, display No Tip.
        u8g2.setCursor(4, 32);

        if (!tt_missing)
        {
            if (heatOn)
            {
                u8g2.print("ON");
            }
            else if (inStandby)  
            {
                u8g2.print("Standby");    
            }
            else
            {
                u8g2.print("OFF");
            }
        } 
        else 
        {            
            u8g2.print("No Tip");
        }

        // idle timer
        u8g2.setCursor(101, 13); 
        snprintf(tmp, sizeof(tmp), "%2ds", standby_counter_s);              
        u8g2.print(tmp);

        // voltage in
        u8g2.setCursor(102, 32);
        snprintf(tmp, sizeof(tmp), "%2dv", vin_v);
        u8g2.print(tmp);

        // wattage tip
        u8g2.setCursor(74, 32);
        snprintf(tmp, sizeof(tmp), "%2dw", t_output_w);
        u8g2.print(tmp);

        // Tip temp. Display big font
        u8g2.setFont(u8g2_font_crox2cb_tr);
        u8g2.setCursor(0, 15);
        snprintf(tmp, sizeof(tmp), "%3d/%3dC",tt_degC, selectedTemperature_degC);
        u8g2.print(tmp);

    } while (u8g2.nextPage());

    last_showtime = micros() - start;
}
