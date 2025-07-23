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

const uint8_t pinR = PB3;  // Software PWM (Red)
const uint8_t pinG = PB4;  
const uint8_t pinB = PB1;  

volatile uint8_t* Port[3]   = {&OCR1A, &OCR1B, &OCR0B};
volatile uint8_t current[3] = {0xFF, 0xFF, 0xFF};

// Sets colour Red=0 Green=1 Blue=2 White=3
// to specified intensity 0 (off) to 255 (max)
void SetColour(int colour, int intensity) {
    if (current[colour] != intensity) {
        current[colour] = intensity;
        noInterrupts();
        *Port[colour] = 255 - intensity;
        interrupts();
    }
}  

void testLED() {
    for (int i = 0; i < 255; i++) {
        SetColour(0, i); // Red
        delay(1);  
    }
    for (int i = 255; i >= 0; i--) {
        SetColour(0, i); // Red
        delay(1);
    }
    for (int i = 0; i < 255; i++) {
        SetColour(1, i); // Green
        delay(1);
    }
    for (int i = 255; i >= 0; i--) {
        SetColour(1, i); // Green
        delay(1);
    }
    for (int i = 0; i < 255; i++) {
        SetColour(2, i); // Blue
        delay(1);
    }
    for (int i = 255; i >= 0; i--) {
        SetColour(2, i); // Blue
        delay(1);
    }

    for (int i = 0; i < 255; i++) {
        SetColour(0, i); // Red
        delay(1);  
    }
    for (int i = 0; i < 255; i++) {
        SetColour(1, i); // Green
        delay(1);
    }
    for (int i = 0; i < 255; i++) {
        SetColour(2, i); // Blue
        delay(1);
    }
    for (int i = 0; i < 255; i++) {
        SetColour(0, 255 - i); // Red
        SetColour(1, 255 - i); // Green
        SetColour(2, 255 - i); // Blue
        delay(1);
    }
}

void receiveEvent(uint8_t numBytes);
void requestEvent();

void setup() {
    // Configure counter/timer0 for fast PWM on PB0 and PB1
    TCCR0A = 3<<COM0A0 | 3<<COM0B0 | 3<<WGM00;
    TCCR0B = 0<<WGM02 | 3<<CS00; // Optional; already set
    // Configure counter/timer1 for fast PWM on PB4
    TCCR1 = 1<<CTC1 | 1<<PWM1A | 3<<COM1A0 | 7<<CS10;
    GTCCR = 1<<PWM1B | 3<<COM1B0;
    // Interrupts on OC1A match and overflow
    TIMSK = TIMSK | 1<<OCIE1A | 1<<TOIE1;

    SetColour(0, 0);
    SetColour(1, 0);
    SetColour(2, 0);

    TinyWireS.begin(I2C_SLAVE_ADDR);
    TinyWireS.onReceive(receiveEvent);
    TinyWireS.onRequest(requestEvent);
    sei();  // Enable global interrupts
    delay(3); 
    pinMode(pinR, OUTPUT);
    pinMode(pinG, OUTPUT);
    pinMode(pinB, OUTPUT);
    //testLED();
}

void loop() {
    TinyWireS_stop_check();

    // MODE1 bit 4 = sleep
    if (regs[MODE1_REG] & 0x10) {
        SetColour(0, 0);
        SetColour(1, 0);
        SetColour(2, 0);
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
    SetColour(0, resolveOutput(0, regs[PWM0_REG]));
    SetColour(1, resolveOutput(1, regs[PWM1_REG]));
    SetColour(2, resolveOutput(2, regs[PWM2_REG]));
}

ISR(TIMER1_OVF_vect) {
    bitClear(PORTB, PB3);  // Start of cycle: PB3 LOW
}

ISR(TIMER1_COMPA_vect) {
    if (!bitRead(TIFR, TOV1)) {
        bitSet(PORTB, PB3);  // Set PB3 HIGH if we’re still in cycle
    }
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
