/***************************************************************************
  This is a library for the BMP280 pressure sensor

  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#ifndef __BMP280_H__
#define __BMP280_H__


#include "application.h"
#include "Adafruit_Sensor.h"

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define BMP280_ADDRESS                (0x77)
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    enum
    {
      BMP280_REGISTER_DIG_T1              = 0x88,
      BMP280_REGISTER_DIG_T2              = 0x8A,
      BMP280_REGISTER_DIG_T3              = 0x8C,

      BMP280_REGISTER_DIG_P1              = 0x8E,
      BMP280_REGISTER_DIG_P2              = 0x90,
      BMP280_REGISTER_DIG_P3              = 0x92,
      BMP280_REGISTER_DIG_P4              = 0x94,
      BMP280_REGISTER_DIG_P5              = 0x96,
      BMP280_REGISTER_DIG_P6              = 0x98,
      BMP280_REGISTER_DIG_P7              = 0x9A,
      BMP280_REGISTER_DIG_P8              = 0x9C,
      BMP280_REGISTER_DIG_P9              = 0x9E,

      BMP280_REGISTER_CHIPID             = 0xD0,
      BMP280_REGISTER_VERSION            = 0xD1,
      BMP280_REGISTER_SOFTRESET          = 0xE0,

      BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

      BMP280_REGISTER_CONTROL            = 0xF4,
      BMP280_REGISTER_CONFIG             = 0xF5,
      BMP280_REGISTER_PRESSUREDATA       = 0xF7,
      BMP280_REGISTER_TEMPDATA           = 0xFA,
    };

/*=========================================================================*/

/*=========================================================================
    CALIBRATION DATA
    -----------------------------------------------------------------------*/
    typedef struct
    {
      uint16_t dig_T1;
      int16_t  dig_T2;
      int16_t  dig_T3;

      uint16_t dig_P1;
      int16_t  dig_P2;
      int16_t  dig_P3;
      int16_t  dig_P4;
      int16_t  dig_P5;
      int16_t  dig_P6;
      int16_t  dig_P7;
      int16_t  dig_P8;
      int16_t  dig_P9;

      uint8_t  dig_H1;
      int16_t  dig_H2;
      uint8_t  dig_H3;
      int16_t  dig_H4;
      int16_t  dig_H5;
      int8_t   dig_H6;
    } bmp280_calib_data;
/*=========================================================================*/

/*
class Adafruit_BMP280_mod_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_BMP280_mod_Unified(int32_t sensorID = -1);

    bool  begin(uint8_t addr = BMP280_ADDRESS);
    void  getTemperature(float *temp);
    void  getPressure(float *pressure);
    float pressureToAltitude(float seaLevel, float atmospheric, float temp);
    float seaLevelForAltitude(float altitude, float atmospheric, float temp);
    void  getEvent(sensors_event_t*);
    void  getSensor(sensor_t*);

  private:
    uint8_t   _i2c_addr;
    int32_t   _sensorID;
};

*/

class Adafruit_BMP280_mod
{
  public:
    /** Oversampling rate for the sensor. */
    enum sensor_sampling {
        /** No over-sampling. */
        SAMPLING_NONE = 0x00,
        /** 1x over-sampling. */
        SAMPLING_X1   = 0x01,
        /** 2x over-sampling. */
        SAMPLING_X2   = 0x02,
        /** 4x over-sampling. */
        SAMPLING_X4   = 0x03,
        /** 8x over-sampling. */
        SAMPLING_X8   = 0x04,
        /** 16x over-sampling. */
        SAMPLING_X16  = 0x05
    };

    /** Operating mode for the sensor. */
    enum sensor_mode {
        /** Sleep mode. */
        MODE_SLEEP  = 0x00,
        /** Forced mode. */
        MODE_FORCED = 0x01,
        /** Normal mode. */
        MODE_NORMAL = 0x03,
        /** Software reset. */
        MODE_SOFT_RESET_CODE = 0xB6
    };

    /** Filtering level for sensor data. */
    enum sensor_filter {
        /** No filtering. */
        FILTER_OFF = 0x00,
        /** 2x filtering. */
        FILTER_X2  = 0x01,
        /** 4x filtering. */
        FILTER_X4  = 0x02,
        /** 8x filtering. */
        FILTER_X8  = 0x03,
        /** 16x filtering. */
        FILTER_X16 = 0x04
    };

    /** Standby duration in ms */
    enum standby_duration {
        /** 1 ms standby. */
        STANDBY_MS_1      = 0x00,
        /** 63 ms standby. */
        STANDBY_MS_63     = 0x01,
        /** 125 ms standby. */
        STANDBY_MS_125    = 0x02,
        /** 250 ms standby. */
        STANDBY_MS_250    = 0x03,
        /** 500 ms standby. */
        STANDBY_MS_500    = 0x04,
        /** 1000 ms standby. */
        STANDBY_MS_1000   = 0x05,
        /** 2000 ms standby. */
        STANDBY_MS_2000   = 0x06,
        /** 4000 ms standby. */
        STANDBY_MS_4000   = 0x07
    };

    /** Default I2C constructor. */
    Adafruit_BMP280_mod(void);

    /**
     * Default SPI constructor.
     * @param cspin     The pin to use for CS/SSEL.
     */
    Adafruit_BMP280_mod(int8_t cspin);

    /**
     * Bit-bang SPI constructor.
     * @param cspin     The pin to use for CS/SSEL.
     * @param mosipin   The pin to use for MOSI.
     * @param misopin   The pin to use for MISO.
     * @param sckpin    The pin to use for SCK.
     */
    Adafruit_BMP280_mod(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin);

    /**
     * Initialises the sensor.
     * @param addr      The I2C address to use (default = 0x77)
     * @param chipid    The expected chip ID (used to validate connection).
     * @return True if the init was successful, otherwise false.
     */
    bool  begin(uint8_t addr = BMP280_ADDRESS);

    /**
     * Reads the temperature from the device.
     * @return The temperature in degress celcius.
     */
    float readTemperature(void);
    float readTemperatureFahrenheit(void);

    /**
     * Reads the barometric pressure from the device.
     * @return Barometric pressure in hPa.
     */
    float readPressure(void);
    float readPressureHg(void);

    /**
     * Calculates the approximate altitude using barometric pressure and the
     * supplied sea level hPa as a reference.
     * @param seaLevelhPa   The current hPa at sea level.
     * @return The approximate altitude above sea level in meters.
     */
    float readAltitude(float seaLevelhPa = 1013.25);

    /**
     * Sets the sampling config for the device.
     * @param mode          The operating mode of the sensor.
     * @param tempSampling  The sampling scheme for temp readings.
     * @param pressSampling The sampling scheme for pressure readings.
     * @param filter        The filtering mode to apply (if any).
     * @param duration      The sampling duration.
     */
    void setSampling(sensor_mode mode      = MODE_NORMAL,
                     sensor_sampling tempSampling  = SAMPLING_X16,
                     sensor_sampling pressSampling = SAMPLING_X16,
                     sensor_filter filter          = FILTER_OFF,
                     standby_duration duration     = STANDBY_MS_1);
  private:
    /** Encapsulates the config register */
    struct config
    {
      /** Inactive duration (standby time) in normal mode */
      unsigned int t_sb : 3;
      /** Filter settings */
      unsigned int filter : 3;
      /** Unused - don't set */
      unsigned int none : 1;
      /** Enables 3-wire SPI */
      unsigned int spi3w_en : 1;
      /** Used to retrieve the assembled config register's byte value. */
      unsigned int get()
      {
        return (t_sb << 5) | (filter << 3) | spi3w_en;
      }
    };

    /** Encapsulates trhe ctrl_meas register */
    struct ctrl_meas
    {
      /** Temperature oversampling. */
      unsigned int osrs_t : 3;
      /** Pressure oversampling. */
      unsigned int osrs_p : 3;
      /** Device mode */
      unsigned int mode : 2;
      /** Used to retrieve the assembled ctrl_meas register's byte value. */
      unsigned int get()
      {
        return (osrs_t << 5) | (osrs_p << 3) | mode;
      }
    };
    void readCoefficients(void);
    uint8_t spixfer(uint8_t x);

    void      write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian

    uint8_t   _i2caddr;
    int32_t   _sensorID;
    int32_t   t_fine;

    int8_t _cs, _mosi, _miso, _sck;

    bmp280_calib_data _bmp280_calib;
    config    _configReg;
    ctrl_meas _measReg;
};

#endif
