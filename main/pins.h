#ifndef M5STICKC_PLUS2_PINS_H
#define M5STICKC_PLUS2_PINS_H

#include "hal/gpio_types.h"

// ============================================================================
// M5StickC Plus2 (ESP32-PICO-V3-02) Pin Definitions
// ============================================================================

// --- Power Management ---
#define M5_POWER_HOLD_PIN       GPIO_NUM_4   // MUST be set HIGH on boot to stay on
#define M5_VBAT_MEASURE_PIN     GPIO_NUM_38  // ADC pin to measure battery voltage

// --- Buttons ---
// Note: Buttons are active-low (read 0 when pressed, 1 when released)
#define M5_BUTTON_A_PIN         GPIO_NUM_37  // Big front button
#define M5_BUTTON_B_PIN         GPIO_NUM_39  // Right side button
#define M5_BUTTON_C_PIN         GPIO_NUM_35  // Left side (Power) button / Wake

// --- Display (ST7789V2 135x240) ---
#define M5_TFT_MOSI_PIN         GPIO_NUM_15
#define M5_TFT_SCLK_PIN         GPIO_NUM_13
#define M5_TFT_CS_PIN           GPIO_NUM_5
#define M5_TFT_DC_PIN           GPIO_NUM_14
#define M5_TFT_RST_PIN          GPIO_NUM_12
#define M5_TFT_BACKLIGHT_PIN    GPIO_NUM_27  // PWM to control brightness

// --- Internal I2C Bus ---
// Connects to MPU6886 (IMU) and BM8563 (RTC)
#define M5_I2C_SDA_PIN          GPIO_NUM_21
#define M5_I2C_SCL_PIN          GPIO_NUM_22

// --- Onboard Peripherals ---
#define M5_LED_PIN              GPIO_NUM_19  // Internal LED (Shared with IR)
#define M5_IR_TX_PIN            GPIO_NUM_19  // Infrared Transmitter (Shared with LED)
#define M5_BUZZER_PIN           GPIO_NUM_2   // Passive Buzzer (requires PWM via LEDC)

// --- Microphone (SPM1423 PDM) ---
#define M5_MIC_CLK_PIN          GPIO_NUM_0
#define M5_MIC_DATA_PIN         GPIO_NUM_34

// --- External Ports ---
// Grove Port (HY2.0-4P) - Default I2C or UART
#define M5_GROVE_SDA_RX_PIN     GPIO_NUM_32
#define M5_GROVE_SCL_TX_PIN     GPIO_NUM_33

// Top Header / HAT Expansion Pins
#define M5_HAT_G0_PIN           GPIO_NUM_0   // Shared with Mic CLK
#define M5_HAT_G25_PIN          GPIO_NUM_25
#define M5_HAT_G26_PIN          GPIO_NUM_26
#define M5_HAT_G36_PIN          GPIO_NUM_36

#endif // M5STICKC_PLUS2_PINS_H