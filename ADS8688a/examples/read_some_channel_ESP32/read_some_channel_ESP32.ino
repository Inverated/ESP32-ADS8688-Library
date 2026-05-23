#include <ADS8688.h>

#define PIN_CS   7
#define PIN_SCK  6
#define PIN_MOSI 5
#define PIN_MISO 4

ADS8688 adc(PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO);

void setup() {
    Serial.begin(115200);
    
    adc.setChannelRange(1, R5);
    adc.setChannelRange(2, R1);
    adc.setChannelRange(3, R5);

    for(int i=0;i<10;i++) {
        Serial.println(adc.getChannelRange(i), HEX);
        delay(100);
    }

    //adc.setChannelSequence(0xFF);
    //adc.setSampleRate(4);

    adc.setChannelSequence(0b00001110);
    adc.setSampleRate(3);

    // Start auto scan mode
    adc.autoRst();
    adc.autoRst();
    delay(1000);
}

void loop() {
    adc.waitForSample();
    Serial.println();
    uint16_t raw1 = adc.noOp();
    Serial.printf(
        "CH%d : %5u (0x%04X) -> %.5f V\n",
        1,
        raw1,
        raw1,
        adc.I2V(raw1, R5)
    );
    uint16_t raw2 = adc.noOp();
    Serial.printf(
        "CH%d : %5u (0x%04X) -> %.5f V\n",
        2,
        raw2,
        raw2,
        adc.I2V(raw2, R1)
    );
    uint16_t raw3 = adc.noOp();
    Serial.printf(
        "CH%d : %5u (0x%04X) -> %.5f V\n",
        3,
        raw3,
        raw3,
        adc.I2V(raw3, R5)
    );

}