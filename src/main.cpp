#include <Arduino.h>
#include <TinyWireS.h>

// PCA9632 default I2C address
#define I2C_SLAVE_ADDR 0x62

// Register definitions
#define MODE1_REG     0x00
#define MODE2_REG     0x01
#define PWM0_REG      0x02
#define PWM1_REG      0x03
#define PWM2_REG      0x04
#define PWM3_REG      0x05
#define GRPPWM_REG    0x06
#define GRPFREQ_REG   0x07
#define LEDOUT_REG    0x08

uint8_t regs[0x10] = {0};
uint8_t reg_pointer = 0;

// RGB pins (hardware PWM-capable)
const uint8_t pinR = PB3; // OC1A
const uint8_t pinG = PB4; // OC1B
const uint8_t pinB = PB0; // fallback to Timer0 OC0A

void receiveEvent(uint8_t numBytes);
void requestEvent();

void setup() {
    pinMode(pinR, OUTPUT);
    pinMode(pinG, OUTPUT);
    pinMode(pinB, OUTPUT);

    TinyWireS.begin(I2C_SLAVE_ADDR);
    TinyWireS.onReceive(receiveEvent);
    TinyWireS.onRequest(requestEvent);
}

void loop() {
    TinyWireS_stop_check();

    // MODE1 bit 4 = sleep
    if (regs[MODE1_REG] & 0x10) {
        analogWrite(pinR, 0);
        analogWrite(pinG, 0);
        analogWrite(pinB, 0);
        return;
    }

    uint8_t ledout = regs[LEDOUT_REG];

    auto resolveOutput = [&](uint8_t channel, uint8_t pwmVal) -> uint8_t {
        switch ((ledout >> (channel * 2)) & 0x03) {
            case 0x00: return 0;                     // OFF
            case 0x01: return 255;                   // ON
            case 0x02: return pwmVal;                // Individual PWM
            case 0x03: return regs[GRPPWM_REG];      // Group PWM
            default:   return 0;
        }
    };

    analogWrite(pinR, resolveOutput(0, regs[PWM0_REG]));
    analogWrite(pinG, resolveOutput(1, regs[PWM1_REG]));
    analogWrite(pinB, resolveOutput(2, regs[PWM2_REG]));
}

void receiveEvent(uint8_t numBytes) {
    if (numBytes < 1) return;
    reg_pointer = TinyWireS.receive();

    while (TinyWireS.available()) {
        uint8_t val = TinyWireS.receive();
        if (reg_pointer < sizeof(regs)) {
            regs[reg_pointer++] = val;
        }
    }
}

void requestEvent() {
    if (reg_pointer < sizeof(regs)) {
        TinyWireS.send(regs[reg_pointer++]);
    } else {
        TinyWireS.send(0xFF);
    }
}
