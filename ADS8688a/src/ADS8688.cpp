    // ADS8688 library for Arduino / ESP32
    // Original: Sylvain GARNAVAULT - 2016/03/13
    // Revised:  Bug fixes and ESP32 support
    //
    // Datasheet: ADS8684/ADS8688, SBAS582C, Texas Instruments, April 2015

    #include "ADS8688.h"

    // ===========================================================================
    // CONSTRUCTORS
    // ===========================================================================

    // Shared initialisation called by every constructor.
    void ADS8688::_init() {
        _mode           = MODE_IDLE;
        _vref           = 4.096f;
        _feature        = 0;
        _samplePeriodUs = 0;
        _lastSampleUs   = 0;
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);   // CS idles HIGH (active LOW, p.4 pin 38)
    }

    // Default: CS = pin 10, platform-default SPI pins.
    // Not recommended on ESP32 — use the 4-pin constructor.
    ADS8688::ADS8688() {
        _cs   = 10;
        _sck  = -1;
        _mosi = -1;
        _miso = -1;
        _init();
        SPI.begin();
    }

    // Custom CS, platform-default SPI pins.
    // Not recommended on ESP32 — use the 4-pin constructor.
    ADS8688::ADS8688(uint8_t cs) {
        _cs   = cs;
        _sck  = -1;
        _mosi = -1;
        _miso = -1;
        _init();
        SPI.begin();
    }

    // Fully custom SPI pins — use this constructor on ESP32.
    // Argument order matches the ESP32 Arduino SPI.begin() signature:
    //   SPI.begin(SCK, MISO, MOSI, SS)  (datasheet p.32, Figure 70)
    ADS8688::ADS8688(uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t miso) {
        _cs   = cs;
        _sck  = (int16_t)sck;
        _mosi = (int16_t)mosi;
        _miso = (int16_t)miso;
        _init();
        SPI.begin(_sck, _miso, _mosi, _cs);  // ESP32: (SCK, MISO, MOSI, SS)
    }

    // ===========================================================================
    // PUBLIC METHODS — REFERENCE VOLTAGE
    // ===========================================================================

    // Override the Vref used by I2V() and V2I().
    // The hardware default is the internal 4.096 V reference.
    void ADS8688::setVREF(float vref) {
        _vref = vref;
    }

    // ===========================================================================
    // PUBLIC METHODS — DEVICE COMMANDS
    // All return the ADC result clocked out during the SPI frame.
    // The result is 0 when the device is not actively converting.
    // One-frame latency applies: result belongs to the previous channel (p.34).
    // ===========================================================================

    uint16_t ADS8688::noOp() {
        return cmdRegister(NO_OP);
    }

    uint16_t ADS8688::standBy() {
        return cmdRegister(STDBY);
    }

    uint16_t ADS8688::powerDown() {
        return cmdRegister(PWR_DN);
    }

    uint16_t ADS8688::reset() {
        return cmdRegister(RST);
    }

    uint16_t ADS8688::autoRst() {
        return cmdRegister(AUTO_RST);
    }

    // Select a specific channel for manual conversion.
    // ch: 0–7 for analog inputs, 8 for the AUX channel.
    uint16_t ADS8688::manualChannel(uint8_t ch) {
        const uint8_t cmds[] = {
            MAN_Ch_0, MAN_Ch_1, MAN_Ch_2, MAN_Ch_3,
            MAN_Ch_4, MAN_Ch_5, MAN_Ch_6, MAN_Ch_7, MAN_AUX
        };
        uint8_t reg = (ch > 8) ? MAN_Ch_0 : cmds[ch];
        return cmdRegister(reg);
    }

    // ===========================================================================
    // PUBLIC METHODS — AUTO-SCAN CHANNEL CONTROL
    // (AUTO_SEQ_EN 0x01, CH_PWR_DN 0x02 — Table 9, p.47)
    // ===========================================================================

    // Enable channels in flag for scanning and power-down all others.
    void ADS8688::setChannelSPD(uint8_t flag) {
        setChannelSequence(flag);
        setChannelPowerDown((uint8_t)(~flag));
    }

    // Write AUTO_SEQ_EN: bit n = 1 includes channel n in the auto scan.
    // Default power-on value = 0xFF (all channels enabled, p.47 Table 9).
    void ADS8688::setChannelSequence(uint8_t flag) {
        writeRegister(AUTO_SEQ_EN, flag);
    }

    // Write CH_PWR_DN: bit n = 1 powers down channel n front-end.
    // Default power-on value = 0x00 (all channels powered up, p.47 Table 9).
    void ADS8688::setChannelPowerDown(uint8_t flag) {
        writeRegister(CH_PWR_DN, flag);
    }

    uint8_t ADS8688::getChannelSequence() {
        return readRegister(AUTO_SEQ_EN);
    }

    uint8_t ADS8688::getChannelPowerDown() {
        return readRegister(CH_PWR_DN);
    }

    // ===========================================================================
    // PUBLIC METHODS — PER-CHANNEL INPUT RANGE
    // (RG_Ch_n registers, Table 9 p.47; range bits, Table 15 p.51)
    // ===========================================================================

    uint8_t ADS8688::getChannelRange(uint8_t ch) {
        const uint8_t regs[] = {
            RG_Ch_0, RG_Ch_1, RG_Ch_2, RG_Ch_3,
            RG_Ch_4, RG_Ch_5, RG_Ch_6, RG_Ch_7
        };
        return readRegister((ch > 7) ? RG_Ch_0 : regs[ch]);
    }

    void ADS8688::setChannelRange(uint8_t ch, uint8_t range) {
        const uint8_t regs[] = {
            RG_Ch_0, RG_Ch_1, RG_Ch_2, RG_Ch_3,
            RG_Ch_4, RG_Ch_5, RG_Ch_6, RG_Ch_7
        };
        writeRegister((ch > 7) ? RG_Ch_0 : regs[ch], range);
    }

    // Apply the same input range to all eight channels.
    void ADS8688::setGlobalRange(uint8_t range) {
        for (uint8_t i = 0; i < 8; i++) {
            setChannelRange(i, range);
        }
    }

    // ===========================================================================
    // PUBLIC METHODS — FEATURE SELECT  (FT_SEL 0x03, Table 12, p.50)
    //
    // Register layout:
    //   bits 7-6 : DEV[1:0]   daisy-chain device ID
    //   bits 5-3 : 0           read-only, reserved (must stay 0)
    //   bits 2-0 : SDO[2:0]   output data format
    //
    // Note: bit 4 is a reserved read-only zero. The original library incorrectly
    // used bit 4 as an alarm enable — that code has been removed.
    // ===========================================================================

    // Returns the daisy-chain device ID (bits 7-6).
    uint8_t ADS8688::getId() {
        return (getFeatureSelect() >> 6) & 0x03;
    }

    // Set the daisy-chain device ID (0–3). Leave at 0 for single-chip use.
    void ADS8688::setId(uint8_t id) {
        _feature = (_feature & 0x07) | ((id & 0x03) << 6);
        writeRegister(FT_SEL, _feature);
    }

    // Returns the SDO output format selection (bits 2-0).
    uint8_t ADS8688::getSdo() {
        return getFeatureSelect() & 0x07;
    }

    // Set the SDO output format (0–3).
    // WARNING: formats 1–3 output more than 16 bits; this library only captures
    // the first 16 bits and will silently discard the rest. Use 0 (default).
    void ADS8688::setSdo(uint8_t sdo) {
        _feature = (_feature & 0xC0) | (sdo & 0x07);
        writeRegister(FT_SEL, _feature);
    }

    uint8_t ADS8688::getFeatureSelect() {
        return readRegister(FT_SEL);
    }

    // Set both the device ID and SDO format in one register write.
    // The alarm parameter from the original library has been removed (see header).
    void ADS8688::setFeatureSelect(uint8_t id, uint8_t sdo) {
        _feature = ((id & 0x03) << 6) | (sdo & 0x07);
        writeRegister(FT_SEL, _feature);
    }

    // ===========================================================================
    // PUBLIC METHODS — ALARM FLAG REGISTERS  (read-only, p.47)
    // ===========================================================================

    uint8_t ADS8688::getAlarmOverview() {
        return readRegister(ALARM_OVERVIEW);
    }

    uint8_t ADS8688::getFirstTrippedFlag() {
        return readRegister(ALARM_CH0_TRIPPED_FLAG);
    }

    uint8_t ADS8688::getSecondTrippedFlag() {
        return readRegister(ALARM_CH4_TRIPPED_FLAG);
    }

    // Returns a 16-bit combined tripped-flag word: MSB = channels 0–3, LSB = 4–7.
    uint16_t ADS8688::getTrippedFlags() {
        uint8_t msb = readRegister(ALARM_CH0_TRIPPED_FLAG);
        uint8_t lsb = readRegister(ALARM_CH4_TRIPPED_FLAG);
        return ((uint16_t)msb << 8) | lsb;
    }

    uint8_t ADS8688::getFirstActiveFlag() {
        return readRegister(ALARM_CH0_ACTIVE_FLAG);
    }

    uint8_t ADS8688::getSecondActiveFlag() {
        return readRegister(ALARM_CH4_ACTIVE_FLAG);
    }

    // Returns a 16-bit combined active-flag word: MSB = channels 0–3, LSB = 4–7.
    uint16_t ADS8688::getActiveFlags() {
        uint8_t msb = readRegister(ALARM_CH0_ACTIVE_FLAG);
        uint8_t lsb = readRegister(ALARM_CH4_ACTIVE_FLAG);
        return ((uint16_t)msb << 8) | lsb;
    }

    // ===========================================================================
    // PUBLIC METHODS — ALARM THRESHOLDS
    // Register base address for channel n: CH0_base + 5*n  (p.42 comment)
    // ===========================================================================

    uint8_t ADS8688::getChannelHysteresis(uint8_t ch) {
        uint8_t reg = (uint8_t)(5 * (ch > 7 ? 7 : ch)) + CH0_HYST;
        return readRegister(reg);
    }

    uint16_t ADS8688::getChannelHighThreshold(uint8_t ch) {
        uint8_t reg = (uint8_t)(5 * (ch > 7 ? 7 : ch)) + CH0_HT_MSB;
        return ((uint16_t)readRegister(reg) << 8) | readRegister(reg + 1);
    }

    uint16_t ADS8688::getChannelLowThreshold(uint8_t ch) {
        uint8_t reg = (uint8_t)(5 * (ch > 7 ? 7 : ch)) + CH0_LT_MSB;
        return ((uint16_t)readRegister(reg) << 8) | readRegister(reg + 1);
    }

    void ADS8688::setChannelHysteresis(uint8_t ch, uint8_t val) {
        uint8_t reg = (uint8_t)(5 * (ch > 7 ? 7 : ch)) + CH0_HYST;
        writeRegister(reg, val);
    }

    void ADS8688::setChannelHighThreshold(uint8_t ch, uint16_t val) {
        uint8_t reg = (uint8_t)(5 * (ch > 7 ? 7 : ch)) + CH0_HT_MSB;
        writeRegister(reg,     (uint8_t)(val >> 8));
        writeRegister(reg + 1, (uint8_t)(val & 0xFF));
    }

    void ADS8688::setChannelLowThreshold(uint8_t ch, uint16_t val) {
        uint8_t reg = (uint8_t)(5 * (ch > 7 ? 7 : ch)) + CH0_LT_MSB;
        writeRegister(reg,     (uint8_t)(val >> 8));
        writeRegister(reg + 1, (uint8_t)(val & 0xFF));
    }

    // ===========================================================================
    // PUBLIC METHODS — COMMAND READ-BACK  (0x3F, p.51)
    // ===========================================================================

    uint8_t ADS8688::getCommandReadBack() {
        return readRegister(CMD_READBACK);
    }

    // ===========================================================================
    // PUBLIC METHODS — SAMPLE RATE CONTROL
    // ===========================================================================

    // Set the desired aggregate sample rate in SPS (frames per second).
    // The hardware can do up to 500 kSPS (p.8, Sampling Dynamics).
    // Pass 0 to disable rate limiting.
    void ADS8688::setSampleRate(uint32_t sps) {
        if (sps == 0) {
            _samplePeriodUs = 0;
        } else {
            if (sps > 500000UL) sps = 500000UL;   // clamp to hardware maximum
            _samplePeriodUs = 1000000UL / sps;
        }
    }

    // Returns the configured aggregate sample rate. Returns 0 if unlimited.
    uint32_t ADS8688::getSampleRate() {
        return (_samplePeriodUs == 0) ? 0 : (1000000UL / _samplePeriodUs);
    }

    // Returns true when the minimum inter-sample period has elapsed.
    bool ADS8688::isSampleReady() {
        if (_samplePeriodUs == 0) return true;
        return ((uint32_t)(micros() - _lastSampleUs) >= _samplePeriodUs);
    }

    // Block until the minimum inter-sample period has elapsed.
    void ADS8688::waitForSample() {
        while (!isSampleReady()) {
            yield();
        }
    }

    // ===========================================================================
    // CONVERSION HELPERS
    // (straight-binary transfer function, Figure 69, p.31; Table 4, p.31)
    // ===========================================================================

    // Convert a raw 16-bit ADC code to volts.
    float ADS8688::I2V(uint16_t x, uint8_t range) {
        float out_min, out_max;
        switch (range) {
            case R1: out_min = -1.25f  * _vref; out_max =  1.25f  * _vref; break;
            case R2: out_min = -0.625f * _vref; out_max =  0.625f * _vref; break;
            case R3: out_min = -0.3125f* _vref; out_max =  0.3125f* _vref; break;
            case R4: out_min = -0.15625f*_vref; out_max =  0.15625f*_vref; break;
            case R5: out_min =  0.0f;           out_max =  2.5f   * _vref; break;
            case R6: out_min =  0.0f;           out_max =  1.25f  * _vref; break;
            case R7: out_min =  0.0f;           out_max =  0.625f * _vref; break;
            case R8: out_min =  0.0f;           out_max =  0.3125f* _vref; break;
            default: // R0: ±2.5 × Vref (power-on default, Table 9 p.47)
                    out_min = -2.5f * _vref; out_max = 2.5f * _vref;     break;
        }
        // Straight-binary mapping: code 0x0000 = out_min, 0xFFFF = out_max.
        return (float)x * (out_max - out_min) / 65535.0f + out_min;
    }

    // Convert a voltage to the equivalent 16-bit code.
    uint16_t ADS8688::V2I(float x, uint8_t range) {
        float in_min, in_max;
        switch (range) {
            case R1: in_min = -1.25f  * _vref; in_max =  1.25f  * _vref; break;
            case R2: in_min = -0.625f * _vref; in_max =  0.625f * _vref; break;
            case R3: in_min = -0.3125f* _vref; in_max =  0.3125f* _vref; break;
            case R4: in_min = -0.15625f*_vref; in_max =  0.15625f*_vref; break;
            case R5: in_min =  0.0f;           in_max =  2.5f   * _vref; break;
            case R6: in_min =  0.0f;           in_max =  1.25f  * _vref; break;
            case R7: in_min =  0.0f;           in_max =  0.625f * _vref; break;
            case R8: in_min =  0.0f;           in_max =  0.3125f* _vref; break;
            default: // R0
                    in_min = -2.5f * _vref; in_max = 2.5f * _vref;     break;
        }
        float clamped = x < in_min ? in_min : (x > in_max ? in_max : x);
        return (uint16_t)((clamped - in_min) * 65535.0f / (in_max - in_min));
    }

    // ===========================================================================
    // PRIVATE METHODS — SPI TRANSACTIONS
    // ===========================================================================

    void ADS8688::writeRegister(uint8_t reg, uint8_t val) {

        SPI.beginTransaction(
            SPISettings(ADS8688_SPI_CLOCK, MSBFIRST, SPI_MODE1)
        );

        uint8_t tx[3];
        uint8_t rx[3];

        tx[0] = (uint8_t)((reg << 1) | 0x01);
        tx[1] = val;
        tx[2] = 0x00;

        digitalWrite(_cs, LOW);

        SPI.transferBytes(tx, rx, 3);

        digitalWrite(_cs, HIGH);

        SPI.endTransaction();

        _mode = MODE_PROG;
    }

    uint8_t ADS8688::readRegister(uint8_t reg) {

        SPI.beginTransaction(
            SPISettings(ADS8688_SPI_CLOCK, MSBFIRST, SPI_MODE1)
        );

        uint8_t tx[3];
        uint8_t rx[3];

        tx[0] = (uint8_t)((reg << 1) | 0x00);
        tx[1] = 0x00;
        tx[2] = 0x00;

        digitalWrite(_cs, LOW);

        SPI.transferBytes(tx, rx, 3);

        digitalWrite(_cs, HIGH);

        SPI.endTransaction();

        _mode = MODE_PROG;

        uint8_t byte2 = rx[1];
        uint8_t byte3 = rx[2];

        return ((byte2 & 0x01) << 7) | (byte3 >> 1);
    }

    uint16_t ADS8688::cmdRegister(uint8_t reg) {

        SPI.beginTransaction(
            SPISettings(ADS8688_SPI_CLOCK, MSBFIRST, SPI_MODE1)
        );

        uint8_t tx[4];
        uint8_t rx[4];

        tx[0] = reg;
        tx[1] = 0x00;
        tx[2] = 0x00;
        tx[3] = 0x00;

        digitalWrite(_cs, LOW);

        SPI.transferBytes(tx, rx, 4);

        digitalWrite(_cs, HIGH);

        SPI.endTransaction();

        uint8_t msb = rx[2];
        uint8_t lsb = rx[3];

        if (_mode > MODE_PROG) {
            _lastSampleUs = micros();
        }

        if (_mode == MODE_POWER_DN) {
            delay(15);
        }

        uint16_t result = 0;

        if (_mode > MODE_PROG) {
            result = ((uint16_t)(rx[1] & 0x01) << 15)
                | ((uint16_t)rx[2]          <<  7)
                | ((uint16_t)rx[3]          >>  1);
        }

        switch (reg) {

            case NO_OP:
                switch (_mode) {
                    case MODE_RESET:
                        _mode = MODE_IDLE;
                        break;

                    case MODE_PROG:
                        _mode = MODE_IDLE;
                        break;

                    case MODE_AUTO_RST:
                        _mode = MODE_AUTO;
                        break;

                    default:
                        break;
                }
                break;

            case STDBY:
                _mode = MODE_STANDBY;
                break;

            case PWR_DN:
                _mode = MODE_POWER_DN;
                break;

            case RST:
                _mode = MODE_RESET;
                break;

            case AUTO_RST:
                _mode = MODE_AUTO_RST;
                break;

            default:
                _mode = MODE_MANUAL;
                break;
        }

        return result;
    }