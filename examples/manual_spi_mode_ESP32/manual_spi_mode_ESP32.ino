#include <ADS8688.h>

#define PIN_CS   7
#define PIN_SCK  6
#define PIN_MOSI 5
#define PIN_MISO 4

#define ENABLE_SPEED_TEST 0
#define SAMPLING_RATE 5
ADS8688 adc(PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO);

// Store raw & converted ADC readings
uint16_t raw[8];
float volts[8];

// Default Vref = 4.096V
// R0: Min = -(2.5 * vref)      = -10.240 V,    Max = +(2.5 * vref)     = +10.240 V  (default)
// R1: Min = -(1.25 * vref)     = -5.120 V,     Max = +(1.25 * vref)    = +5.120 V
// R2: Min = -(0.625 * vref)    = -2.560 V,     Max = +(0.625 * vref)   = +2.560 V
// R3: Min = -(0.3125 * vref)   = -1.280 V,     Max = +(0.3125 * vref)  = +1.280 V
// R4: Min = -(0.15625 * vref)  = -0.640 V,     Max = +(0.15625 * vref) = +0.640 V
// R5: Min = 0.0 V,                             Max = +(2.5 * vref)     = +10.240 V
// R6: Min = 0.0 V,                             Max = +(1.25 * vref)    = +5.120 V
// R7: Min = 0.0 V,                             Max = +(0.625 * vref)   = +2.560 V
// R8: Min = 0.0 V,                             Max = +(0.3125 * vref)  = +1.280 V

// Easy to configure
uint8_t r1_pins[6] = {0,1,2,3,4,5};
uint8_t r5_pins[2] = {6,7};
unsigned long currTime;

void setup() {
    Serial.begin(921600);
    
    for (int i = 0; i < sizeof(r1_pins) / sizeof(r1_pins[0]); i++) {
        adc.setChannelRange(r1_pins[i], R1);
        uint8_t readback = adc.getChannelRange(r1_pins[i]);
    }

    for (int i = 0; i < sizeof(r5_pins) / sizeof(r5_pins[0]); i++) {
        adc.setChannelRange(r5_pins[i], R5);
        uint8_t readback = adc.getChannelRange(r5_pins[i]);
    }

    // Enable all 8 channels in auto scan
    adc.setChannelSequence(0xFF);
    adc.setSampleRate(SAMPLING_RATE);

    // Start auto scan mode
    adc.autoRst();

    delay(500);
    
    currTime = millis();
    SPI.beginTransaction(SPISettings(ADS8688_SPI_CLOCK, MSBFIRST, SPI_MODE1));
}

unsigned long count = 0;
uint16_t raw_readings[8];

void loop() {
    adc.waitForSample();
    adc.readAllChannels(raw_readings);

    for (int i = 0; i < sizeof(r1_pins) / sizeof(r1_pins[0]); i++) {
        uint8_t idx = r1_pins[i];
        raw[idx]   = raw_readings[idx];
        if (!ENABLE_SPEED_TEST) volts[idx] = adc.I2V(raw[idx], R1);
    }

    for (int i = 0; i < sizeof(r5_pins)  / sizeof(r5_pins[0]); i++) {
        uint8_t idx = r5_pins[i];
        raw[idx] = max(raw_readings[idx], (uint16_t)0);
        if (!ENABLE_SPEED_TEST) volts[idx] = adc.I2V(raw[idx], R5);
    }


    if (ENABLE_SPEED_TEST) {
        count += 1;
        if (count % 20000 == 0) {
            unsigned long now = millis();
            Serial.printf("Count = %d, currTime passed since last (s) = %d\n", count, (now - currTime));
            currTime = now;
        }
        return;
    }
    
    Serial.println("--------------------------------");
    for (int ch = 0; ch < 8; ch++) {
        Serial.printf(
            "CH%d : %5u (0x%04X) -> %.5f V\n",
            ch,
            raw[ch],
            raw[ch],
            volts[ch]
        );
    }
    
}