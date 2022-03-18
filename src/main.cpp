#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#include "settings.h"

Adafruit_MCP4725 dac;

uint16_t value = 0;

void setup() {
    if(DEBUG_SERIAL_EN) {
        Serial.begin(SERIAL_MONITOR_SPEED);
    }

    pinMode(PIN_DAC_VCC, OUTPUT);
    pinMode(PIN_DAC_GND, OUTPUT);

    digitalWrite(PIN_DAC_VCC, HIGH);
    digitalWrite(PIN_DAC_GND, LOW);
    
    if(dac.begin(I2C_ADDR_DAC)) {
        DEBUG_SERIAL_LN("DAC INITIALIZED SUCCESSFULLY!");
    } else {
        DEBUG_SERIAL_LN("DAC INITIALIZATION ERROR!");
    }
}

void loop() {
    if(dac.setVoltage(value, false)) {
        DEBUG_SERIAL_LN("DAC SET TO " + String(value));
    } else {
        DEBUG_SERIAL_LN("DAC COMM ERROR! " + String(value));
    }

    if(++value == 4096) value = 0;

}