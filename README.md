# ADS8688a
ADS8688a Arduino Library adapted from [siteswapjuggler](https://github.com/siteswapjuggler/ADS8688a) 

# Description
- Very convinent and easy to wire chip. Allows bipolar input up to 10.24V with a single 5V analog power source. Comes with internal voltage reference, adjustable digital supply voltage for direct connection with ESP32. Only require capacitor and resistor for decoupling.
- Original code meant for arduino board. SPI pins not customisable
- Reworked the original library for my use on ESP32-C3 project, to guarenteed to work across all ESP platform

# New Features
- Manual SPI pin setting -> Initialise with ADS8688(uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t miso);
- Replace signed variables with unsigned -> In bipolar mode, midpoint is used for 0V, adc reading will not be negative
- Added software speed setting by tracking time between fetching -> adc.setSampleRate(0-500000); adc.waitForSample();
- Fix known bit shift issue when converting SPI transfer for ESP32
    - [Shifted Reading](https://www.reddit.com/r/arduino/comments/1f0yewj/ads8688_16bit_saradc_data_acquisition_module_and/)
    - [SPI falling edge off](https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/452422/ads8688-flow-chart)
    - SPI transfer reading gets shifted by 1 bit throughout (noOps, getChannelRange, etc all shifted - doubles of what is provided)
    - **Note that when setting clock speed above 30M, reading is how shifted back, where the fix is halving the correct value instead. Will not be implementing a work around as recommended speed is 17M**
- ESP32 implementation writes to hardware register instead of digital.read/write for faster reading
    - ifdef added to switch between ESP32 and not ESP32, unsure if will work
- Efficient fetching of ADC values with custom readAllChannels(raw_readings) to directly read from all channels at once
    - Adapted from the adapted cmdRegister manual SPI transaction mode (Unable to achieve max rated sampling rate)
    - **Max achieved speed  -> ~297kSPS**
    - **Max rated speed     ->  500kSPS**
- Added function description (by Claude)
    - Alarm & daisy function not tested on ESP32.
- Additional examples for ESP32 with the newly added functions