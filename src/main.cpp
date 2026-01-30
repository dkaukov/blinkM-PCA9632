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

uint8_t regs[0x10] = {
    0x00, // MODE1
    0x00, // MODE2
    0x00, // PWM0
    0x00, // PWM1
    0x00, // PWM2
    0x00, // PWM3
    0xFF, // GRPPWM
    0x1F, // GRPFREQ
    0x00  // LEDOUT
    // The rest default to 0
};
uint8_t reg_pointer = 0;
bool auto_increment = false;
volatile bool pending_process = false;

const uint8_t pinR = PB3;  // Software PWM (Red)
const uint8_t pinG = PB4;  
const uint8_t pinB = PB1;  

volatile uint8_t* Port[3]   = {&OCR1A, &OCR1B, &OCR0B};
volatile uint8_t current[3] = {0xFF, 0xFF, 0xFF};

// Sets colour Red=0 Green=1 Blue=2 White=3
// to specified intensity 0 (off) to 255 (max)
void SetColour(uint8_t colour, uint8_t intensity) {
    if (intensity > 254) intensity = 254;
    if (current[colour] != intensity) {
        current[colour] = intensity;
        uint8_t sreg = SREG;
        cli();
        *Port[colour] = 255 - intensity;
        SREG = sreg;
    }
}  

void testLED() {
    for (int i = 0; i <= 255; i++) {
        SetColour(0, i); // Red
        delay(1);  
    }
    for (int i = 255; i >= 0; i--) {
        SetColour(0, i); // Red
        delay(1);
    }
    for (int i = 0; i <= 255; i++) {
        SetColour(1, i); // Green
        delay(1);
    }
    for (int i = 255; i >= 0; i--) {
        SetColour(1, i); // Green
        delay(1);
    }
    for (int i = 0; i <= 255; i++) {
        SetColour(2, i); // Blue
        delay(1);
    }
    for (int i = 255; i >= 0; i--) {
        SetColour(2, i); // Blue
        delay(1);
    }

    for (int i = 0; i <= 255; i++) {
        SetColour(0, i); // Red
        delay(1);  
    }
    for (int i = 0; i <= 255; i++) {
        SetColour(1, i); // Green
        delay(1);
    }
    for (int i = 0; i <= 255; i++) {
        SetColour(2, i); // Blue
        delay(1);
    }
    for (int i = 0; i <= 255; i++) {
        SetColour(0, 255 - i); // Red
        SetColour(1, 255 - i); // Green
        SetColour(2, 255 - i); // Blue
        delay(1);
    }
}

void receiveEvent(uint8_t numBytes);
void requestEvent();
void processEvent();
uint8_t nextRegPointer(uint8_t pointer);

void setup() {
    // Configure counter/timer0 for fast PWM on PB1 (PB0 reserved for SDA)
    TCCR0A = 3<<COM0B0 | 3<<WGM00;
    TCCR0B = 0<<WGM02 | 3<<CS00; // Optional; already set
    // Configure counter/timer1 for fast PWM on PB4 (OC1B); OC1A is unused
    TCCR1 = 1<<CTC1 | 1<<PWM1A | 7<<CS10;
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
    //SetColour(0, 255);
}

void loop() {
    TinyWireS_stop_check();
    if (pending_process) {
        uint8_t sreg = SREG;
        cli();
        pending_process = false;
        SREG = sreg;
        processEvent();
    }
}

void processEvent() {
    // MODE1 bit 4 = sleep
    if (regs[MODE1_REG] & 0x10) {
        SetColour(0, 0);
        SetColour(1, 0);
        SetColour(2, 0);
        return;
    }
    uint8_t ledout =  regs[LEDOUT_REG];
    for (uint8_t channel = 0; channel < 3; channel++) {
        uint8_t mode = (ledout >> (channel * 2)) & 0x03;
        uint8_t value;
        switch (mode) {
            case 0x00: value = 0; break;                          // OFF
            case 0x01: value = 255; break;                        // ON
            case 0x02: value = regs[PWM0_REG + channel]; break;   // Individual PWM
            case 0x03: value = (uint16_t(regs[PWM0_REG + channel]) * uint16_t(regs[GRPPWM_REG])) >> 8; break; // Group PWM
            default:   value = 0; break;
        }
        SetColour(channel, value);
    }
}

uint8_t nextRegPointer(uint8_t pointer) {
    if (auto_increment) {
        pointer++;
        if (pointer >= sizeof(regs)) {
            pointer = 0;
        }
    }
    return pointer;
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
    if (numBytes > sizeof(regs)) return;
    reg_pointer = TinyWireS.receive();
    if (reg_pointer >= sizeof(regs)) {
        reg_pointer = 0;
    }
    numBytes--;
    auto_increment = (regs[MODE1_REG] & 0x20) != 0;
    while (numBytes > 0) {
        regs[reg_pointer] = TinyWireS.receive();
        if (reg_pointer == MODE1_REG) {
            auto_increment = (regs[MODE1_REG] & 0x20) != 0;
        }
        reg_pointer = nextRegPointer(reg_pointer);
        numBytes--;

    }
    pending_process = true;
}

void requestEvent() {
    auto_increment = (regs[MODE1_REG] & 0x20) != 0;
    TinyWireS.send(regs[reg_pointer]);
    reg_pointer = nextRegPointer(reg_pointer);
}
