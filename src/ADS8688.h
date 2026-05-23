// ADS8688 library for Arduino / ESP32
// Original: Sylvain GARNAVAULT - 2016/03/13
// Revised:  Bug fixes and ESP32 support + Speed up reading (05/2026)
//
// Key fixes vs original:
//  - cmdRegister() result changed int16_t -> uint16_t (datasheet p.31, Fig 69:
//    output is straight binary unsigned; original code misread any code above
//    0x7FFF - i.e. any positive signal above midscale - as negative)
//  - MSB shift explicitly cast to uint16_t to prevent overflow
//  - Always transmits a full 32-bit frame; original sent only 16 bits on the
//    first autoRst()/manualChannel() call from IDLE, risking an invalid first
//    conversion (p.41, Fig 81)
//  - _lastSampleUs now stamped only during active conversions (mode > 4),
//    not during standby/reset commands
//  - setAlarm() / getAlarm() removed: bit 4 of FT_SEL is a read-only reserved
//    bit and must remain 0 (p.50, Table 12); writing it is undefined behaviour
//  - setFeatureSelect() alarm parameter removed for the same reason
//  - All constructors initialise _samplePeriodUs and _lastSampleUs
//  - SPI pins fully configurable via 4-pin constructor (required on ESP32)
//
// Datasheet: ADS8684/ADS8688, SBAS582C, Texas Instruments, April 2015

#ifndef ADS8688_H
#define ADS8688_H

#include "Arduino.h"
#include "SPI.h"

// ---------------------------------------------------------------------------
// COMMAND REGISTER MAP  (Table 6, p.45)
// MSB of the 16-bit command word sent via SDI.
// ---------------------------------------------------------------------------
#define NO_OP     0x00   // Continue operation in previous mode
#define STDBY     0x82   // Enter standby (internal reference stays on)
#define PWR_DN    0x83   // Full power-down (15 ms recovery with internal ref)
#define RST       0x85   // Reset all program registers to defaults
#define AUTO_RST  0xA0   // Enter auto channel-scan mode with sequence reset
#define MAN_Ch_0  0xC0   // Manual select: channel 0
#define MAN_Ch_1  0xC4   // Manual select: channel 1
#define MAN_Ch_2  0xC8   // Manual select: channel 2
#define MAN_Ch_3  0xCC   // Manual select: channel 3
#define MAN_Ch_4  0xD0   // Manual select: channel 4
#define MAN_Ch_5  0xD4   // Manual select: channel 5
#define MAN_Ch_6  0xD8   // Manual select: channel 6
#define MAN_Ch_7  0xDC   // Manual select: channel 7
#define MAN_AUX   0xE0   // Manual select: AUX channel

// ---------------------------------------------------------------------------
// PROGRAM REGISTER MAP  (Table 9, p.47)
// ---------------------------------------------------------------------------

// Auto-scan sequencing
#define AUTO_SEQ_EN  0x01  // bit n = enable channel n in scan (default 0xFF)
#define CH_PWR_DN    0x02  // bit n = power down channel n front-end (default 0x00)

// Feature select  (Table 12, p.50)
//   bits 7-6 : DEV[1:0] daisy-chain device ID
//   bits 5-3 : reserved, must be 0
//   bits 2-0 : SDO[2:0] output data format
// NOTE: bit 4 is NOT an alarm enable - it is read-only reserved.
#define FT_SEL       0x03

// Per-channel input range registers  (p.47, Table 9)
#define RG_Ch_0  0x05
#define RG_Ch_1  0x06
#define RG_Ch_2  0x07
#define RG_Ch_3  0x08
#define RG_Ch_4  0x09
#define RG_Ch_5  0x0A
#define RG_Ch_6  0x0B
#define RG_Ch_7  0x0C

// Alarm flag registers (read-only hardware latches)  (p.47, Table 9)
#define ALARM_OVERVIEW          0x10
#define ALARM_CH0_TRIPPED_FLAG  0x11
#define ALARM_CH0_ACTIVE_FLAG   0x12
#define ALARM_CH4_TRIPPED_FLAG  0x13
#define ALARM_CH4_ACTIVE_FLAG   0x14

// Per-channel alarm threshold registers  (CH n = CH0 base + 5*n, p.42)
#define CH0_HYST    0x15
#define CH0_HT_MSB  0x16
#define CH0_HT_LSB  0x17
#define CH0_LT_MSB  0x18
#define CH0_LT_LSB  0x19

// Command read-back (read-only)
#define CMD_READBACK  0x3F

// ---------------------------------------------------------------------------
// INPUT RANGE SELECTION  (Range_CHn[3:0], Table 15, p.51)
//
// With internal Vref = 4.096 V (nominal):
//   R0  ±10.24 V bipolar     R5   0-10.24 V unipolar
//   R1  ±5.12 V  bipolar     R6   0-5.12 V  unipolar
//   R2  ±2.56 V  bipolar
//
// R3, R4, R7, R8 are ADS8688A extended ranges - not in the standard
// ADS8688 datasheet (SBAS582C) but present on your ADS8688AIDBTR.
// ---------------------------------------------------------------------------
#define R0  0x00   // ±2.5  x Vref  -> ±10.24 V
#define R1  0x01   // ±1.25 x Vref  -> ±5.12 V
#define R2  0x02   // ±0.625x Vref  -> ±2.56 V
#define R3  0x03   // ±0.3125xVref  -> ±1.28 V  (ADS8688A only)
#define R4  0x0B   // ±0.15625xVref -> ±0.64 V  (ADS8688A only)
#define R5  0x05   //  0-2.5  x Vref -> 0-10.24 V
#define R6  0x06   //  0-1.25 x Vref -> 0-5.12 V
#define R7  0x07   //  0-0.625x Vref -> 0-2.56 V  (ADS8688A only)
#define R8  0x0F   //  0-0.3125xVref -> 0-1.28 V  (ADS8688A only)

// ---------------------------------------------------------------------------
// INTERNAL OPERATION MODES  (state diagram, Figure 76, p.37)
// ---------------------------------------------------------------------------
#define MODE_IDLE      0
#define MODE_RESET     1
#define MODE_STANDBY   2
#define MODE_POWER_DN  3
#define MODE_PROG      4
#define MODE_MANUAL    5
#define MODE_AUTO      6
#define MODE_AUTO_RST  7

#define ADS8688_SPI_CLOCK  30000000UL

// ---------------------------------------------------------------------------
// CLASS DECLARATION
// ---------------------------------------------------------------------------
class ADS8688 {
public:
    // Default: CS = pin 10, platform-default SPI pins. Arduino only.
    ADS8688();

    // Custom CS pin, platform-default SPI pins. Arduino only.
    explicit ADS8688(uint8_t cs);

    // Fully custom SPI pins - required on ESP32.
    ADS8688(uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t miso);

    // -----------------------------------------------------------------------
    // Reference voltage
    // -----------------------------------------------------------------------

    // Set the Vref used by I2V() and V2I().
    // Default = 4.096 V (internal reference, p.10 Table 7.5).
    // Only change this if you are using an external reference of a
    // different value; the hardware REFSEL pin controls which source is used.
    void     setVREF(float vref);

    // -----------------------------------------------------------------------
    // Conversion helpers
    // -----------------------------------------------------------------------

    // Convert a raw 16-bit ADC code to volts for the given range constant.
    // Formula based on the straight-binary transfer function (p.31, Fig 69).
    float    I2V(uint16_t x, uint8_t range);

    // Convert a voltage to the equivalent 16-bit code for the given range.
    uint16_t V2I(float x, uint8_t range);

    // -----------------------------------------------------------------------
    // Device commands
    //
    // Every function sends a complete 32-bit SPI frame:
    //   - first 16 bits  : command word sent to the device via SDI
    //   - next  16 bits  : dummy clocks that clock out the ADC result via SDO
    // -----------------------------------------------------------------------
    uint16_t noOp();                     // Continue current mode; returns ADC result
    uint16_t noOpRaw();                  // Continue current mode; returns ADC result; Manually set SPI.beginTransaction(SPISettings(ADS8688_SPI_CLOCK, MSBFIRST, SPI_MODE1)) and SPI.endTransaction()
    uint16_t standBy();                  // Standby (ref on; fast 20 s wake-up)
    uint16_t powerDown();                // Full power-down (15 ms wake-up with int ref)
    uint16_t reset();                    // Reset all registers; must reconfigure before sampling
    uint16_t autoRst();                  // Start auto channel-scan (channels per AUTO_SEQ_EN)
    uint16_t manualChannel(uint8_t ch);  // Select one channel manually; ch 0-7, or 8 for AUX

    // -----------------------------------------------------------------------
    // Auto-scan channel control  (AUTO_SEQ_EN 0x01 / CH_PWR_DN 0x02)
    // flag is an 8-bit mask: bit n corresponds to channel n.
    // -----------------------------------------------------------------------

    // Enable channels in flag for scanning AND power-down channels not in flag.
    void     setChannelSPD(uint8_t flag);

    // Write the AUTO_SEQ_EN register. Default = 0xFF (all channels in scan).
    void     setChannelSequence(uint8_t flag);

    // Write the CH_PWR_DN register. Default = 0x00 (all channels powered up).
    void     setChannelPowerDown(uint8_t flag);

    uint8_t  getChannelSequence();
    uint8_t  getChannelPowerDown();

    // -----------------------------------------------------------------------
    // Per-channel input range  (RG_Ch_n registers)
    // Pass one of the R0-R8 constants.
    // -----------------------------------------------------------------------
    uint8_t  getChannelRange(uint8_t ch);
    void     setChannelRange(uint8_t ch, uint8_t range);
    void     setGlobalRange(uint8_t range);  // Apply same range to all 8 channels

    // -----------------------------------------------------------------------
    // Feature Select register  (FT_SEL 0x03, p.50, Table 12)
    //
    // id  : daisy-chain device ID 0-3 (leave 0 for single-chip use)
    // sdo : output data format 0-3   (leave 0 - library reads 16-bit frames)
    //         0 = 16-bit result only (default)
    //         1 = result + 4-bit channel address   (> 16 bits, truncated)
    //         2 = result + channel + device address (> 16 bits, truncated)
    //         3 = result + channel + device + range (> 16 bits, truncated)
    //
    // setAlarm() / getAlarm() from the original library are intentionally
    // omitted: bit 4 of FT_SEL is a read-only reserved bit (Table 12, p.50).
    // -----------------------------------------------------------------------
    uint8_t  getId();
    void     setId(uint8_t id);
    uint8_t  getSdo();
    void     setSdo(uint8_t sdo);
    uint8_t  getFeatureSelect();
    void     setFeatureSelect(uint8_t id, uint8_t sdo);

    // -----------------------------------------------------------------------
    // Alarm flag registers (read-only hardware latches, p.47)
    // -----------------------------------------------------------------------
    uint8_t  getAlarmOverview();
    uint8_t  getFirstTrippedFlag();    // channels 0-3 tripped
    uint8_t  getSecondTrippedFlag();   // channels 4-7 tripped
    uint16_t getTrippedFlags();        // channels 0-7 combined (MSB = ch0)
    uint8_t  getFirstActiveFlag();     // channels 0-3 currently active
    uint8_t  getSecondActiveFlag();    // channels 4-7 currently active
    uint16_t getActiveFlags();         // channels 0-7 combined (MSB = ch0)

    // -----------------------------------------------------------------------
    // Per-channel alarm thresholds  (p.42 header, CHn base = CH0 + 5*n)
    // -----------------------------------------------------------------------
    uint8_t  getChannelHysteresis(uint8_t ch);
    uint16_t getChannelLowThreshold(uint8_t ch);
    uint16_t getChannelHighThreshold(uint8_t ch);
    void     setChannelHysteresis(uint8_t ch, uint8_t val);
    void     setChannelLowThreshold(uint8_t ch, uint16_t val);
    void     setChannelHighThreshold(uint8_t ch, uint16_t val);

    // -----------------------------------------------------------------------
    // Command read-back register (0x3F, p.51)
    // Returns the upper byte of the last executed command word.
    // -----------------------------------------------------------------------
    uint8_t  getCommandReadBack();

    // -----------------------------------------------------------------------
    // Software-enforced sample rate
    //
    // Hardware maximum: 500 kSPS aggregate across all channels.
    // These helpers use micros() to pace calls to cmdRegister().
    //
    // For N channels at F samples/channel/sec in AUTO_RST mode set:
    //   setSampleRate(N * F)
    // Example: 8 channels at 1 kHz each -> setSampleRate(8000)
    //
    // Pass sps = 0 to disable rate limiting (run at full hardware speed but limited by SPI transaction end).
    // -----------------------------------------------------------------------
    void     setSampleRate(uint32_t sps);
    uint32_t getSampleRate();    // Returns 0 when rate limiting is disabled
    bool     isSampleReady();    // True when the minimum sample period has elapsed
    void     waitForSample();    // Block until the sample period elapses

    // -----------------------------------------------------------------------
    // Optimised Reading
    //
    // Removes multiple micros() by directly assigning all 8 channel reading into an array
    // Returns the timing when the channels are finished reading in micros()
    // -----------------------------------------------------------------------
    uint32_t   readAllChannels(uint16_t* out8);  // Must be in AUTO mode already


private:
    float    _vref;
    int16_t  _sck, _mosi, _miso;   // -1 means use platform default
    uint8_t  _cs, _mode, _feature;
    uint32_t _samplePeriodUs;       // 0 = unlimited
    uint32_t _lastSampleUs;

    void     _init();              // Shared constructor body
    void     writeRegister(uint8_t reg, uint8_t val);
    uint8_t  readRegister(uint8_t reg);
    uint16_t cmdRegister(uint8_t reg, bool manual = false);
};

#endif // ADS8688_H