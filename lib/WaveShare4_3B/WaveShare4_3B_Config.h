/**
 * @file WaveShare4_3B_Config.h
 * @brief Pin definitions and hardware constants for Waveshare ESP32-S3-Touch-LCD-4.3B
 * @link https://docs.waveshare.com/ESP32-S3-Touch-LCD-4.3B
 */

#ifndef WAVESHARE_4_3B_CONFIG_H
#define WAVESHARE_4_3B_CONFIG_H

#include <Arduino.h>

// =============================================================================
// 1. RS485 Interface (SP3485EN Transceiver with Hardware Auto Direction)
// =============================================================================
// The onboard SP3485EN transceiver features an automatic direction control circuit.
// No software DE/RE pin toggle is needed; transmit & receive are automatic!
#define WS43B_RS485_TX_PIN            44   // ESP32-S3 GPIO44 -> SP3485 DI (TX)
#define WS43B_RS485_RX_PIN            43   // ESP32-S3 GPIO43 -> SP3485 RO (RX)
#define WS43B_RS485_DEFAULT_BAUD      9600 // Default industrial baud rate (supports up to 115200+)
#define WS43B_RS485_UART_NUM          1    // Use HardwareSerial 1 (Serial1)

// =============================================================================
// 2. CAN Bus / TWAI Interface (TJA1051T/3 Transceiver)
// =============================================================================
// Connects to external screw terminal pins CANH (Pin 8) & CANL (Pin 7).
// An onboard 120-ohm termination switch (SW2) can be toggled as needed.
#define WS43B_CAN_TX_PIN              15   // ESP32-S3 GPIO15 -> TJA1051 TXD
#define WS43B_CAN_RX_PIN              16   // ESP32-S3 GPIO16 -> TJA1051 RXD
#define WS43B_CAN_DEFAULT_BAUD        500000 // 500 kbit/s default

// =============================================================================
// 3. I2C Bus Interface (Shared between Onboard ICs and External Screw Terminal)
// =============================================================================
// Connects internally to:
//   - GT911 Capacitive Touch Screen Controller (0x5D or 0x14)
//   - CH422G IO Expander (0x24 / 0x38 / 0x26)
//   - RTC PCF85063 (0x51, if populated)
// Connects externally to:
//   - Screw Terminal Pin 5: D_SDA
//   - Screw Terminal Pin 6: D_SCL
//   - Screw Terminal Pin 3: I2C_VCC (3.3V or 5V selected via onboard solder jumper)
//   - Screw Terminal Pin 4: GND
#define WS43B_I2C_SDA_PIN             8    // ESP32-S3 GPIO8
#define WS43B_I2C_SCL_PIN             9    // ESP32-S3 GPIO9
#define WS43B_I2C_DEFAULT_FREQ        100000 // 100 kHz standard mode (supports 400kHz)

// Known Onboard I2C Slave Addresses
#define WS43B_I2C_ADDR_GT911_PRIMARY   0x5D // GT911 Default
#define WS43B_I2C_ADDR_GT911_ALT       0x14 // GT911 Alternate
#define WS43B_I2C_ADDR_CH422G_SYS      0x24 // CH422G System Parameter Address
#define WS43B_I2C_ADDR_CH422G_OUT      0x38 // CH422G Output Control Address
#define WS43B_I2C_ADDR_CH422G_IN       0x26 // CH422G Input Read Address
#define WS43B_I2C_ADDR_PCF85063        0x51 // RTC address (if present)

// =============================================================================
// 4. CH422G IO Expander Pin Mappings
// =============================================================================
// The CH422G provides 8 bidirectional/open-collector expanded pins:
#define WS43B_EXIO_DI0                0   // EXIO0: Isolated Digital Input 0 (DIN0)
#define WS43B_EXIO_TP_RST             1   // EXIO1: Capacitive Touch GT911 Reset
#define WS43B_EXIO_DISP               2   // EXIO2: LCD Display & Backlight Enable
#define WS43B_EXIO_LCD_RST            3   // EXIO3: LCD Controller Reset
#define WS43B_EXIO_SD_CS              4   // EXIO4: MicroSD Card Chip Select
#define WS43B_EXIO_DI1                5   // EXIO5: Isolated Digital Input 1 (DIN1)

// =============================================================================
// 5. External Terminal Block (P1) Pin Reference
// =============================================================================
/*
 * P1 Pin  1: VIN      (Wide voltage DC input: 7V - 36V)
 * P1 Pin  2: GND      (Power Ground)
 * P1 Pin  3: I2C_VCC  (I2C Power Output: 3.3V / 5V jumper-selectable)
 * P1 Pin  4: GND      (I2C Ground)
 * P1 Pin  5: D_SDA    (I2C Data Line - GPIO8)
 * P1 Pin  6: D_SCL    (I2C Clock Line - GPIO9)
 * P1 Pin  7: CANL     (CAN Bus Differential Low)
 * P1 Pin  8: CANH     (CAN Bus Differential High)
 * P1 Pin  9: RS485_A  (RS485 Differential A / +)
 * P1 Pin 10: RS485_B  (RS485 Differential B / -)
 * P1 Pin 11: DOUT0    (Isolated Digital Output 0)
 * P1 Pin 12: DOUT1    (Isolated Digital Output 1)
 * P1 Pin 13: DI_COM   (Isolated Digital Input Common)
 * P1 Pin 14: GND      (Signal Ground)
 * P1 Pin 15: DIN0     (Isolated Digital Input 0)
 * P1 Pin 16: DIN1     (Isolated Digital Input 1)
 */

// =============================================================================
// 6. Display & Touch Pin Mapping (Reference for ST7701 800x480 RGB Screen)
// =============================================================================
#define WS43B_LCD_DE_PIN              5
#define WS43B_LCD_VSYNC_PIN           3
#define WS43B_LCD_HSYNC_PIN           46
#define WS43B_LCD_PCLK_PIN            7

// RGB Data Pins (RGB565 format)
#define WS43B_LCD_R1_PIN              1
#define WS43B_LCD_R2_PIN              2
#define WS43B_LCD_R3_PIN              42
#define WS43B_LCD_R4_PIN              41
#define WS43B_LCD_R5_PIN              40

#define WS43B_LCD_G0_PIN              39
#define WS43B_LCD_G1_PIN              0
#define WS43B_LCD_G2_PIN              45
#define WS43B_LCD_G3_PIN              48
#define WS43B_LCD_G4_PIN              47
#define WS43B_LCD_G5_PIN              21

#define WS43B_LCD_B1_PIN              14
#define WS43B_LCD_B2_PIN              38
#define WS43B_LCD_B3_PIN              18
#define WS43B_LCD_B4_PIN              17
#define WS43B_LCD_B5_PIN              10

// Capacitive Touch Controller (GT911)
#define WS43B_TOUCH_IRQ_PIN           4
// Touch Reset is controlled via CH422G EXIO1 (WS43B_EXIO_TP_RST)

#endif // WAVESHARE_4_3B_CONFIG_H
