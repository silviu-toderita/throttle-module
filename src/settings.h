#pragma once

#define DEBUG_SERIAL_EN         0
#define SERIAL_MONITOR_SPEED    115200

#define I2C_ADDR_DAC            0x60

#define PIN_DAC_VCC             A3
#define PIN_DAC_GND             A2

#if DEBUG_SERIAL_EN
    #define DEBUG_SERIAL_LN(x) Serial.println(x)
    #define DEBUG_SERIAL(x) Serial.print(x)
    #define DEBUG_SERIAL_F(x, ...) Serial.printf(x, __VA_ARGS__)
#else
    #define DEBUG_SERIAL_LN(x) x
    #define DEBUG_SERIAL(x) x
    #define DEBUG_SERIAL_F(X) x
#endif