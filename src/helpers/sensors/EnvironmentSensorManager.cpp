#include "EnvironmentSensorManager.h"

#include <Wire.h>

#if ENV_PIN_SDA && ENV_PIN_SCL
#define TELEM_WIRE &Wire1  // Use Wire1 as the I2C bus for Environment Sensors
#else
#define TELEM_WIRE &Wire  // Use default I2C bus for Environment Sensors
#endif

// ============================================================
// Sensor library includes and static driver instances
// ============================================================

#if ENV_INCLUDE_BME680_BSEC
#ifndef TELEM_BME680_ADDRESS
#define TELEM_BME680_ADDRESS 0x76
#endif
#define TELEM_BME680_SEALEVELPRESSURE_HPA (1013.25)
#include <bsec.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
static const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_28d/bsec_iaq.txt" // 3.3v, LP, 28 day background calibration window
};
static Bsec     bsec_iaq;
static float    bsec_temperature     = 0;
static float    bsec_humidity        = 0;
static float    bsec_pressure_hpa    = 0;
static float    bsec_iaq_val         = 0;
static uint8_t  bsec_accuracy        = 0;
static bool     bsec_active          = false;
static bool     bsec_data_ready      = false;
static bool     bsec_first_save_done = false;
static uint32_t bsec_last_save_ms    = 0;
#define BSEC_STATE_FILE "/bsec_state.bin"
#define BSEC_SAVE_INTERVAL_MS (8UL * 60 * 60 * 1000) // 8 hour state-save interval
#endif

#ifdef ENV_INCLUDE_BME680
#ifndef TELEM_BME680_ADDRESS
#define TELEM_BME680_ADDRESS 0x76
#endif
#define TELEM_BME680_SEALEVELPRESSURE_HPA (1013.25)
#include <Adafruit_BME680.h>
static Adafruit_BME680 BME680(TELEM_WIRE);
#endif

#ifdef ENV_INCLUDE_BMP085
#define TELEM_BMP085_SEALEVELPRESSURE_HPA (1013.25)
#include <Adafruit_BMP085.h>
static Adafruit_BMP085 BMP085;
#endif

#if ENV_INCLUDE_AHTX0
#ifndef TELEM_AHTX_ADDRESS
#define TELEM_AHTX_ADDRESS      0x38      // AHT10, AHT20 temperature and humidity sensor I2C address
#endif
#include <Adafruit_AHTX0.h>
static Adafruit_AHTX0 AHTX0;
#endif

#if ENV_INCLUDE_BME280
#ifndef TELEM_BME280_ADDRESS
#define TELEM_BME280_ADDRESS    0x76      // BME280 environmental sensor I2C address
#endif
#define TELEM_BME280_SEALEVELPRESSURE_HPA (1013.25)    // Atmospheric pressure at sea level
#include <Adafruit_BME280.h>
static Adafruit_BME280 BME280;
#endif

#if ENV_INCLUDE_BMP280
#ifndef TELEM_BMP280_ADDRESS
#define TELEM_BMP280_ADDRESS    0x76      // BMP280 environmental sensor I2C address
#endif
#define TELEM_BMP280_SEALEVELPRESSURE_HPA (1013.25)    // Atmospheric pressure at sea level
#include <Adafruit_BMP280.h>
static Adafruit_BMP280 BMP280(TELEM_WIRE);
#endif

#if ENV_INCLUDE_SHTC3
#include <Adafruit_SHTC3.h>
static Adafruit_SHTC3 SHTC3;
#endif

#if ENV_INCLUDE_SHT4X
#ifndef TELEM_SHT4X_ADDRESS
#define TELEM_SHT4X_ADDRESS 0x44
#endif
#include <SensirionI2cSht4x.h>
static SensirionI2cSht4x SHT4X;
#endif

#if ENV_INCLUDE_LPS22HB
#include <Arduino_LPS22HB.h>
LPS22HBClass LPS22HB(*TELEM_WIRE);
#endif

// ============================================================
// INA family (TI current/power monitors)
//
// All INA2xx parts share the 0x40-0x4F address range (INA3221:
// 0x40-0x43), so the address alone cannot tell which chip is
// present. By default each INA type scans its full range and the
// chip type is confirmed through its ID registers (see
// identifyINA() below).
//
// Defining TELEM_INAxxx_ADDRESS in build flags pins that type to
// a single address. For INA219, which has no ID registers, a
// pinned address also skips identification entirely.
// ============================================================

#if ENV_INCLUDE_INA3221
#ifdef TELEM_INA3221_ADDRESS
#define TELEM_INA3221_ADDR_FIRST  TELEM_INA3221_ADDRESS
#define TELEM_INA3221_ADDR_LAST   TELEM_INA3221_ADDRESS
#else
#define TELEM_INA3221_ADDR_FIRST  0x40
#define TELEM_INA3221_ADDR_LAST   0x43
#endif
#ifndef TELEM_INA3221_SHUNT_VALUE
#define TELEM_INA3221_SHUNT_VALUE 0.100 // most variants will have a 0.1 ohm shunts
#endif
#ifndef TELEM_INA3221_NUM_CHANNELS
#define TELEM_INA3221_NUM_CHANNELS 3
#endif
#include <Adafruit_INA3221.h>
static Adafruit_INA3221 INA3221;
#endif

#if ENV_INCLUDE_INA219
#ifdef TELEM_INA219_ADDRESS
#define TELEM_INA219_ADDR_FIRST   TELEM_INA219_ADDRESS
#define TELEM_INA219_ADDR_LAST    TELEM_INA219_ADDRESS
#define TELEM_INA219_PINNED       1  // trust the configured address, no ID check possible
#else
#define TELEM_INA219_ADDR_FIRST   0x40
#define TELEM_INA219_ADDR_LAST    0x4F
#endif
#include <Adafruit_INA219.h>
static Adafruit_INA219* ina219_dev = nullptr;  // created at runtime with the detected address
#endif

#if ENV_INCLUDE_INA260
#ifdef TELEM_INA260_ADDRESS
#define TELEM_INA260_ADDR_FIRST   TELEM_INA260_ADDRESS
#define TELEM_INA260_ADDR_LAST    TELEM_INA260_ADDRESS
#else
#define TELEM_INA260_ADDR_FIRST   0x40
#define TELEM_INA260_ADDR_LAST    0x4F
#endif
#include <Adafruit_INA260.h>
static Adafruit_INA260 INA260;
#endif

#if ENV_INCLUDE_INA226
#ifdef TELEM_INA226_ADDRESS
#define TELEM_INA226_ADDR_FIRST   TELEM_INA226_ADDRESS
#define TELEM_INA226_ADDR_LAST    TELEM_INA226_ADDRESS
#else
#define TELEM_INA226_ADDR_FIRST   0x40
#define TELEM_INA226_ADDR_LAST    0x4F
#endif
#ifndef TELEM_INA226_SHUNT_VALUE
#define TELEM_INA226_SHUNT_VALUE  0.100
#endif
#ifndef TELEM_INA226_MAX_AMP
#define TELEM_INA226_MAX_AMP      0.8
#endif
#include <INA226.h>
static INA226* ina226_dev = nullptr;  // created at runtime with the detected address
#endif

#if ENV_INCLUDE_INA228
#ifdef TELEM_INA228_ADDRESS
#define TELEM_INA228_ADDR_FIRST   TELEM_INA228_ADDRESS
#define TELEM_INA228_ADDR_LAST    TELEM_INA228_ADDRESS
#else
#define TELEM_INA228_ADDR_FIRST   0x40
#define TELEM_INA228_ADDR_LAST    0x4F
#endif
#ifndef TELEM_INA228_SHUNT_VALUE
#define TELEM_INA228_SHUNT_VALUE  0.015  // shunt resistor in ohm
#endif
#ifndef TELEM_INA228_MAX_AMP
#define TELEM_INA228_MAX_AMP      10.0   // expected max current in A
#endif
#include <INA228.h>
static INA228* ina228_dev = nullptr;  // created at runtime with the detected address
#endif

#if ENV_INCLUDE_MLX90614
#ifndef TELEM_MLX90614_ADDRESS
#define TELEM_MLX90614_ADDRESS 0x5A      // MLX90614 IR temperature sensor I2C address
#endif
#include <Adafruit_MLX90614.h>
static Adafruit_MLX90614 MLX90614;
#endif

#if ENV_INCLUDE_VL53L0X
#ifndef TELEM_VL53L0X_ADDRESS
#define TELEM_VL53L0X_ADDRESS 0x29      // VL53L0X time-of-flight distance sensor I2C address
#endif
#include <Adafruit_VL53L0X.h>
static Adafruit_VL53L0X VL53L0X;
#endif

#if ENV_INCLUDE_RAK12035
#ifndef TELEM_RAK12035_ADDRESS
#define TELEM_RAK12035_ADDRESS 0x20      // RAK12035 Soil Moisture sensor I2C address
#endif
#include "RAK12035_SoilMoisture.h"
static RAK12035_SoilMoisture RAK12035;
#endif

#if ENV_INCLUDE_GPS && defined(RAK_BOARD) && !defined(RAK_WISMESH_TAG)
#define RAK_WISBLOCK_GPS
#endif

#ifdef RAK_WISBLOCK_GPS
// PowerSaving variant
// -1 = no enable pin; out-of-range values are no-ops in pinMode/digitalWrite,
// while 0 would be a real GPIO (P0.00 = LFXO crystal on nRF52)
static uint32_t gpsResetPin = -1;

static bool i2cGPSFlag = false;
static bool serialGPSFlag = false;
#ifndef TELEM_RAK12500_ADDRESS
#define TELEM_RAK12500_ADDRESS   0x42     //RAK12500 Ublox GPS via i2c
#endif
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
static SFE_UBLOX_GNSS ublox_GNSS;

class RAK12500LocationProvider : public LocationProvider {
  long _lat = 0;
  long _lng = 0;
  long _alt = 0;
  int _sats = 0;
  long _epoch = 0;
  bool _fix = false;

  // PowerSaving variant
  int _pin_en = -1;
public:
  long getLatitude() override { return _lat; }
  long getLongitude() override { return _lng; }
  long getAltitude() override { return _alt; }
  long satellitesCount() override { return _sats; }
  bool isValid() override { return _fix; }
  long getTimestamp() override { return _epoch; }
  void sendSentence(const char * sentence) override { }
  void reset() override { }
  void begin() override { }
  void stop() override { }
  void loop() override {
    // PowerSaving variant
    static long next_gps_update = 0;
    if ((int32_t)(millis() - next_gps_update) >= 0) {
      if (ublox_GNSS.getGnssFixOk(8)) {
        _fix = true;
        _lat = ublox_GNSS.getLatitude(2) / 10;
        _lng = ublox_GNSS.getLongitude(2) / 10;
        _alt = ublox_GNSS.getAltitude(2);
        _sats = ublox_GNSS.getSIV(2);
      } else {
        _fix = false;
      }
      _epoch = ublox_GNSS.getUnixEpoch(2);

      next_gps_update = millis() + 1000;
    }
  }
  bool isEnabled() override { return true; }

  // PowerSaving variant
  void setPinEn(int pin_en) override { _pin_en = pin_en; }
  int getPinEn() override { return _pin_en; }
};

static RAK12500LocationProvider RAK12500_provider;
#endif

// ============================================================
// I2C bus scanner
// Probes every valid address and records which ones ACK.
// This runs before any sensor library is touched, so a missing
// or misbehaving device cannot stall or crash the boot sequence.
// ============================================================

static void scanI2CBus(TwoWire* wire, bool found[128]) {
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    wire->beginTransmission(addr);
    found[addr] = (wire->endTransmission() == 0);
  }
}

// ============================================================
// INA chip identification
//
// INA226 / INA260 / INA3221: Manufacturer ID at 0xFE, Die ID at 0xFF
// INA228:                    Manufacturer ID at 0x3E, Die ID at 0x3F
// INA219:                    no ID registers; recognized by its
//                            config register value (0x399F), which
//                            is both the power-on default and what
//                            Adafruit's default calibration writes.
//
// Reading an unsupported register on another chip returns an
// undefined value; the exact Die ID comparison rejects it.
//
// Results are cached per address, so each device in 0x40-0x4F is
// probed at most once, and only when an INA table entry asks.
// ============================================================

#if ENV_INCLUDE_INA219 || ENV_INCLUDE_INA226 || ENV_INCLUDE_INA228 || ENV_INCLUDE_INA260 || ENV_INCLUDE_INA3221
#define TELEM_HAS_INA 1
#endif

#ifdef TELEM_HAS_INA
#define TELEM_INA_ADDR_BASE  0x40
#define TELEM_INA_ADDR_COUNT 16

#define TELEM_TI_MANUFACTURER_ID 0x5449  // "TI"

enum InaType : uint8_t { INA_NONE, INA_219, INA_226, INA_228, INA_260, INA_3221 };

static bool    ina_probed[TELEM_INA_ADDR_COUNT] = {};
static InaType ina_type[TELEM_INA_ADDR_COUNT]   = {};

// Read a 16-bit big-endian register using plain register-pointer protocol
static bool readReg16(TwoWire* wire, uint8_t addr, uint8_t reg, uint16_t& out) {
  wire->beginTransmission(addr);
  wire->write(reg);
  if (wire->endTransmission() != 0) return false;
  if (wire->requestFrom(addr, (uint8_t)2) != 2) return false;
  uint8_t hi = wire->read();  // read separately: evaluation order matters
  uint8_t lo = wire->read();
  out = ((uint16_t)hi << 8) | lo;
  return true;
}

static InaType identifyINA(TwoWire* wire, uint8_t addr) {
  uint16_t mfg = 0, die = 0;

  // INA226 / INA260 / INA3221
  if (readReg16(wire, addr, 0xFE, mfg) && mfg == TELEM_TI_MANUFACTURER_ID) {
    readReg16(wire, addr, 0xFF, die);
    switch (die & 0xFFF0) {            // ignore revision nibble
      case 0x2260: return INA_226;
      case 0x2270: return INA_260;
      case 0x3220: return (addr <= 0x43) ? INA_3221 : INA_NONE;
    }
  }

  // INA228 (INA237/238 use the same ID layout but are not supported yet)
  if (readReg16(wire, addr, 0x3E, mfg) && mfg == TELEM_TI_MANUFACTURER_ID) {
    readReg16(wire, addr, 0x3F, die);
    switch (die & 0xFFF0) {            // ignore revision nibble
      case 0x2280: return INA_228;
    }
  }

  // INA219 fallback
  uint16_t cfg = 0;
  if (readReg16(wire, addr, 0x00, cfg) && cfg == 0x399F) return INA_219;

  return INA_NONE;
}

static InaType getINAType(TwoWire* wire, uint8_t addr) {
  if (addr < TELEM_INA_ADDR_BASE || addr >= TELEM_INA_ADDR_BASE + TELEM_INA_ADDR_COUNT) return INA_NONE;
  const uint8_t idx = addr - TELEM_INA_ADDR_BASE;
  if (!ina_probed[idx]) {
    ina_type[idx]   = identifyINA(wire, addr);
    ina_probed[idx] = true;
    MESH_DEBUG_PRINTLN("INA identify at %02X: type %d", addr, (int)ina_type[idx]);
  }
  return ina_type[idx];
}
#endif // TELEM_HAS_INA

// ============================================================
// Per-sensor match, init and query functions
//
// match(wire, address) — optional. Called for each detected
//   address in the entry's range before init(). Returns true if
//   the device at that address is this sensor type. A nullptr
//   match means an ACK on the address is enough.
//
// init(wire, address) — called only when the address was seen
//   on the bus (and match() passed). Returns 0 on failure, or the
//   number of telemetry channels the sensor will consume (1 for
//   all single-output sensors; INA3221 returns one per enabled
//   hardware channel; MLX90614 and RAK12035+calibration
//   return 2).
//
// query(channel, sub_channel, lpp) — called once per active
//   sensor entry during querySensors(). sub_channel is always
//   0 for single-output sensors.
// ============================================================

#if ENV_INCLUDE_AHTX0
static uint8_t init_ahtx0(TwoWire* wire, uint8_t addr) {
  return AHTX0.begin(wire, 0, addr) ? 1 : 0;
}
static void query_ahtx0(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  sensors_event_t humidity, temp;
  AHTX0.getEvent(&humidity, &temp);
  lpp.addTemperature(ch, temp.temperature);
  lpp.addRelativeHumidity(ch, humidity.relative_humidity);
}
#endif

#ifdef ENV_INCLUDE_BME680
static uint8_t init_bme680(TwoWire*, uint8_t addr) {
  // Wire was set in the static constructor; begin() takes address only.
  return BME680.begin(addr) ? 1 : 0;
}
static void query_bme680(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  if (BME680.performReading()) {
    lpp.addTemperature(ch, BME680.temperature);
    lpp.addRelativeHumidity(ch, BME680.humidity);
    const float pressure_hpa = BME680.pressure / 100.0f;
    lpp.addBarometricPressure(ch, pressure_hpa);
    lpp.addAltitude(ch, 44330.0f * (1.0f - powf(pressure_hpa / (float)TELEM_BME680_SEALEVELPRESSURE_HPA, 0.1903f)));
    lpp.addGenericSensor(ch, BME680.gas_resistance);
  }
}
#endif

#if ENV_INCLUDE_BME280
static uint8_t init_bme280(TwoWire* wire, uint8_t addr) {
  if (!BME280.begin(addr, wire)) return 0;
  BME280.setSampling(Adafruit_BME280::MODE_FORCED,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::FILTER_OFF,
                     Adafruit_BME280::STANDBY_MS_1000);
  return 1;
}
static void query_bme280(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  if (BME280.takeForcedMeasurement()) {
    lpp.addTemperature(ch, BME280.readTemperature());
    lpp.addRelativeHumidity(ch, BME280.readHumidity());
    lpp.addBarometricPressure(ch, BME280.readPressure() / 100);
    lpp.addAltitude(ch, BME280.readAltitude(TELEM_BME280_SEALEVELPRESSURE_HPA));
  }
}
#endif

#if ENV_INCLUDE_BMP280
static uint8_t init_bmp280(TwoWire*, uint8_t addr) {
  // BMP280 static instance was constructed with TELEM_WIRE; begin() uses it.
  return BMP280.begin(addr) ? 1 : 0;
}
static void query_bmp280(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  lpp.addTemperature(ch, BMP280.readTemperature());
  lpp.addBarometricPressure(ch, BMP280.readPressure() / 100);
  lpp.addAltitude(ch, BMP280.readAltitude(TELEM_BMP280_SEALEVELPRESSURE_HPA));
}
#endif

#if ENV_INCLUDE_SHTC3
static uint8_t init_shtc3(TwoWire* wire, uint8_t) {
  // Adafruit_SHTC3::begin() does not accept an address (fixed at 0x70).
  return SHTC3.begin(wire) ? 1 : 0;
}
static void query_shtc3(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  sensors_event_t humidity, temp;
  SHTC3.getEvent(&humidity, &temp);
  lpp.addTemperature(ch, temp.temperature);
  lpp.addRelativeHumidity(ch, humidity.relative_humidity);
}
#endif

#if ENV_INCLUDE_SHT4X
static uint8_t init_sht4x(TwoWire* wire, uint8_t addr) {
  // SensirionI2cSht4x::begin() does not probe the hardware; use serialNumber()
  // as the actual presence check since it performs a real I2C transaction.
  SHT4X.begin(*wire, addr);
  uint32_t serial = 0;
  return (SHT4X.serialNumber(serial) == 0) ? 1 : 0;
}
static void query_sht4x(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  float temperature, humidity;
  if (SHT4X.measureLowestPrecision(temperature, humidity) == 0) {
    lpp.addTemperature(ch, temperature);
    lpp.addRelativeHumidity(ch, humidity);
  }
}
#endif

#if ENV_INCLUDE_LPS22HB
static uint8_t init_lps22hb(TwoWire*, uint8_t) {
  // LPS22HBClass is constructed with the wire reference; begin() uses it.
  return LPS22HB.begin() ? 1 : 0;
}
static void query_lps22hb(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  lpp.addTemperature(ch, LPS22HB.readTemperature());
  lpp.addBarometricPressure(ch, LPS22HB.readPressure() * 10); // convert kPa to hPa
}
#endif

#if ENV_INCLUDE_INA3221
static bool match_ina3221(TwoWire* wire, uint8_t addr) {
  return getINAType(wire, addr) == INA_3221;
}
static uint8_t init_ina3221(TwoWire* wire, uint8_t addr) {
  if (!INA3221.begin(addr, wire)) return 0;
  for (int i = 0; i < TELEM_INA3221_NUM_CHANNELS; i++) {
    INA3221.setShuntResistance(i, TELEM_INA3221_SHUNT_VALUE);
  }
  // Each enabled hardware channel becomes its own telemetry channel.
  uint8_t enabled = 0;
  for (int i = 0; i < TELEM_INA3221_NUM_CHANNELS; i++) {
    if (INA3221.isChannelEnabled(i)) enabled++;
  }
  return enabled > 0 ? enabled : 1;
}
static void query_ina3221(uint8_t ch, uint8_t sub_ch, CayenneLPP& lpp) {
  // sub_ch is the index of the nth enabled hardware channel.
  uint8_t seen = 0;
  for (int i = 0; i < TELEM_INA3221_NUM_CHANNELS; i++) {
    if (INA3221.isChannelEnabled(i)) {
      if (seen == sub_ch) {
        float v = INA3221.getBusVoltage(i);
        float c = INA3221.getCurrentAmps(i);
        lpp.addVoltage(ch, v);
        lpp.addCurrent(ch, c);
        lpp.addPower(ch, v * c);
        return;
      }
      seen++;
    }
  }
}
#endif

#if ENV_INCLUDE_INA219
#ifdef TELEM_INA219_PINNED
#define TELEM_INA219_MATCH nullptr  // pinned address: skip identification
#else
static bool match_ina219(TwoWire* wire, uint8_t addr) {
  return getINAType(wire, addr) == INA_219;
}
#define TELEM_INA219_MATCH match_ina219
#endif
static uint8_t init_ina219(TwoWire* wire, uint8_t addr) {
  // Only keep the instance if begin() succeeds, so a failed attempt
  // at one address cannot leave a stale instance for the next one.
  Adafruit_INA219* dev = new Adafruit_INA219(addr);
  if (!dev->begin(wire)) { delete dev; return 0; }
  ina219_dev = dev;
  return 1;
}
static void query_ina219(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  lpp.addVoltage(ch, ina219_dev->getBusVoltage_V());
  lpp.addCurrent(ch, ina219_dev->getCurrent_mA() / 1000.0f);
  lpp.addPower(ch, ina219_dev->getPower_mW() / 1000.0f);
}
#endif

#if ENV_INCLUDE_INA260
static bool match_ina260(TwoWire* wire, uint8_t addr) {
  return getINAType(wire, addr) == INA_260;
}
static uint8_t init_ina260(TwoWire* wire, uint8_t addr) {
  return INA260.begin(addr, wire) ? 1 : 0;
}
static void query_ina260(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  lpp.addVoltage(ch, INA260.readBusVoltage() / 1000.0f);
  lpp.addCurrent(ch, INA260.readCurrent() / 1000.0f);
  lpp.addPower(ch, INA260.readPower() / 1000.0f);
}
#endif

#if ENV_INCLUDE_INA226
static bool match_ina226(TwoWire* wire, uint8_t addr) {
  return getINAType(wire, addr) == INA_226;
}
static uint8_t init_ina226(TwoWire* wire, uint8_t addr) {
  INA226* dev = new INA226(addr, wire);
  if (!dev->begin()) { delete dev; return 0; }
  dev->setMaxCurrentShunt(TELEM_INA226_MAX_AMP, TELEM_INA226_SHUNT_VALUE);
  ina226_dev = dev;
  return 1;
}
static void query_ina226(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  lpp.addVoltage(ch, ina226_dev->getBusVoltage());
  lpp.addCurrent(ch, ina226_dev->getCurrent_mA() / 1000.0f);
  lpp.addPower(ch, ina226_dev->getPower_mW() / 1000.0f);
}
#endif

#if ENV_INCLUDE_INA228
static bool match_ina228(TwoWire* wire, uint8_t addr) {
  return getINAType(wire, addr) == INA_228;
}
static uint8_t init_ina228(TwoWire* wire, uint8_t addr) {
  INA228* dev = new INA228(addr, wire);
  if (!dev->begin()) { delete dev; return 0; }
  dev->setMaxCurrentShunt(TELEM_INA228_MAX_AMP, TELEM_INA228_SHUNT_VALUE);
  ina228_dev = dev;
  return 1;
}
static void query_ina228(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  lpp.addVoltage(ch, ina228_dev->getBusVoltage());  // V
  lpp.addCurrent(ch, ina228_dev->getCurrent());     // A
  lpp.addPower(ch, ina228_dev->getPower());         // W
  lpp.addEnergy(ch, ina228_dev->getEnergy() / 3600.0f); // unit is Wh, lpp displays kWh
}
#endif

#if ENV_INCLUDE_MLX90614
static uint8_t init_mlx90614(TwoWire* wire, uint8_t addr) {
  return MLX90614.begin(addr, wire) ? 2 : 0;  // 2 channels: object temp, ambient temp
}
static void query_mlx90614(uint8_t ch, uint8_t sub_ch, CayenneLPP& lpp) {
  if (sub_ch == 0)
    lpp.addTemperature(ch, MLX90614.readObjectTempC());
  else
    lpp.addTemperature(ch, MLX90614.readAmbientTempC());
}
#endif

#if ENV_INCLUDE_VL53L0X
static uint8_t init_vl53l0x(TwoWire* wire, uint8_t addr) {
  return VL53L0X.begin(addr, false, wire) ? 1 : 0;
}
static void query_vl53l0x(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  VL53L0X_RangingMeasurementData_t measure;
  VL53L0X.rangingTest(&measure, false);
  lpp.addDistance(ch, measure.RangeStatus != 4 ? measure.RangeMilliMeter / 1000.0f : 0.0f);
}
#endif

#ifdef ENV_INCLUDE_BMP085
static uint8_t init_bmp085(TwoWire* wire, uint8_t) {
  return BMP085.begin(0, wire) ? 1 : 0;  // mode 0 = ULTRALOWPOWER
}
static void query_bmp085(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  lpp.addTemperature(ch, BMP085.readTemperature());
  lpp.addBarometricPressure(ch, BMP085.readPressure() / 100);
  lpp.addAltitude(ch, BMP085.readAltitude(TELEM_BMP085_SEALEVELPRESSURE_HPA * 100));
}
#endif

#if ENV_INCLUDE_RAK12035
static uint8_t init_rak12035(TwoWire* wire, uint8_t addr) {
  // RAK12035 requires setup() before begin().
  RAK12035.setup(*wire);
  if (!RAK12035.begin(addr)) return 0;
#ifdef ENABLE_RAK12035_CALIBRATION
  return 2;  // moisture channel + calibration channel
#else
  return 1;
#endif
}
static void query_rak12035(uint8_t ch, uint8_t sub_ch, CayenneLPP& lpp) {
  if (sub_ch == 0) {
    lpp.addTemperature(ch, RAK12035.get_sensor_temperature());
    lpp.addPercentage(ch, RAK12035.get_sensor_moisture());
  } else {
#ifdef ENABLE_RAK12035_CALIBRATION
    float cap = RAK12035.get_sensor_capacitance();
    float wet = RAK12035.get_humidity_full();
    float dry = RAK12035.get_humidity_zero();
    lpp.addFrequency(ch, cap);
    lpp.addTemperature(ch, wet);
    lpp.addPower(ch, dry);
    if (cap > dry) RAK12035.set_humidity_zero(cap);
    if (cap < wet) RAK12035.set_humidity_full(cap);
#endif
  }
}
#endif

#if ENV_INCLUDE_BME680_BSEC
static void bsec_load_state() {
  using namespace Adafruit_LittleFS_Namespace;
  File f = InternalFS.open(BSEC_STATE_FILE, FILE_O_READ);
  if (!f) return;
  uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
  f.read(state, BSEC_MAX_STATE_BLOB_SIZE);
  f.close();
  bsec_iaq.setState(state);
}

static void bsec_save_state() {
  using namespace Adafruit_LittleFS_Namespace;
  uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
  bsec_iaq.getState(state);
  InternalFS.remove(BSEC_STATE_FILE);
  File f = InternalFS.open(BSEC_STATE_FILE, FILE_O_WRITE);
  if (!f) return;
  f.write(state, BSEC_MAX_STATE_BLOB_SIZE);
  f.close();
}

static uint8_t init_bme680_bsec(TwoWire* wire, uint8_t addr) {
  bsec_iaq.begin(addr, *wire);
  if (bsec_iaq.bsecStatus != BSEC_OK) return 0;

  bsec_iaq.setConfig(bsec_config_iaq);
  if (bsec_iaq.bsecStatus != BSEC_OK) return 0;

  bsec_virtual_sensor_t outputs[] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
  };
  bsec_iaq.updateSubscription(outputs, 6, BSEC_SAMPLE_RATE_LP);
  if (bsec_iaq.bsecStatus != BSEC_OK) return 0;

  bsec_load_state();
  bsec_active = true;
  return 1;
}

static void query_bme680_bsec(uint8_t ch, uint8_t, CayenneLPP& lpp) {
  if (!bsec_data_ready) return;
  lpp.addTemperature(ch, bsec_temperature);
  lpp.addRelativeHumidity(ch, bsec_humidity);
  lpp.addBarometricPressure(ch, bsec_pressure_hpa);
  lpp.addAltitude(ch, 44330.0f * (1.0f - powf(bsec_pressure_hpa / (float)TELEM_BME680_SEALEVELPRESSURE_HPA, 0.1903f)));
  lpp.addGenericSensor(ch, (uint16_t)bsec_iaq_val);
  lpp.addAnalogInput(ch, (float)bsec_accuracy);
}
#endif

// ============================================================
// Sensor descriptor table
//
// Each entry maps an I2C address range to a sensor's match, init
// and query functions. Only entries whose ENV_INCLUDE_* guard is
// defined are compiled in. The sentinel at the end keeps the
// array non-empty regardless of which sensors are enabled.
//
// Fixed-address sensors use TELEM_ADDR(addr), i.e. a range of one.
// Bosch BMP/BME SDO selects 0x76 or 0x77; probe both, configured
// address first.
//
// Ordering here determines channel assignment at runtime:
// the first detected+initialized sensor gets channel 2, the
// next gets channel 3, and so on.
// ============================================================

struct SensorDef {
  uint8_t     addr_first;
  uint8_t     addr_last;
  const char* name;
  bool      (*match)(TwoWire* wire, uint8_t address);  // nullptr = address ACK is enough
  uint8_t   (*init)(TwoWire* wire, uint8_t address);
  void      (*query)(uint8_t channel, uint8_t sub_channel, CayenneLPP& telemetry);
};

#define TELEM_ADDR(addr) (uint8_t)(addr), (uint8_t)(addr)
#define TELEM_BOSCH_ALT_ADDR(addr) ((uint8_t)((addr) == 0x76 ? 0x77 : 0x76))

static const SensorDef SENSOR_TABLE[] = {
#if ENV_INCLUDE_AHTX0
  { TELEM_ADDR(TELEM_AHTX_ADDRESS),                       "AHT10/AHT20", nullptr, init_ahtx0,       query_ahtx0       },
#endif
#ifdef ENV_INCLUDE_BME680
  { TELEM_ADDR(TELEM_BME680_ADDRESS),                     "BME680",      nullptr, init_bme680,      query_bme680      },
  { TELEM_ADDR(TELEM_BOSCH_ALT_ADDR(TELEM_BME680_ADDRESS)), "BME680",    nullptr, init_bme680,      query_bme680      },
#endif
#if ENV_INCLUDE_BME680_BSEC
  { TELEM_ADDR(TELEM_BME680_ADDRESS),                     "BME680+BSEC", nullptr, init_bme680_bsec, query_bme680_bsec },
  { TELEM_ADDR(TELEM_BOSCH_ALT_ADDR(TELEM_BME680_ADDRESS)), "BME680+BSEC", nullptr, init_bme680_bsec, query_bme680_bsec },
#endif
#if ENV_INCLUDE_BME280
  { TELEM_ADDR(TELEM_BME280_ADDRESS),                     "BME280",      nullptr, init_bme280,      query_bme280      },
  { TELEM_ADDR(TELEM_BOSCH_ALT_ADDR(TELEM_BME280_ADDRESS)), "BME280",    nullptr, init_bme280,      query_bme280      },
#endif
#if ENV_INCLUDE_BMP280
  { TELEM_ADDR(TELEM_BMP280_ADDRESS),                     "BMP280",      nullptr, init_bmp280,      query_bmp280      },
  { TELEM_ADDR(TELEM_BOSCH_ALT_ADDR(TELEM_BMP280_ADDRESS)), "BMP280",    nullptr, init_bmp280,      query_bmp280      },
#endif
#if ENV_INCLUDE_SHTC3
  { TELEM_ADDR(0x70),                                     "SHTC3",       nullptr, init_shtc3,       query_shtc3       },
#endif
#if ENV_INCLUDE_SHT4X
  // Must stay before the INA entries: SHT4X shares 0x44/0x45 with the INA range.
  { TELEM_ADDR(TELEM_SHT4X_ADDRESS),                      "SHT4X",       nullptr, init_sht4x,       query_sht4x       },
#endif
#if ENV_INCLUDE_LPS22HB
  { TELEM_ADDR(0x5C),                                     "LPS22HB",     nullptr, init_lps22hb,     query_lps22hb     },
#endif
#if ENV_INCLUDE_INA3221
  { TELEM_INA3221_ADDR_FIRST, TELEM_INA3221_ADDR_LAST,    "INA3221",     match_ina3221,      init_ina3221, query_ina3221 },
#endif
#if ENV_INCLUDE_INA219
  { TELEM_INA219_ADDR_FIRST,  TELEM_INA219_ADDR_LAST,     "INA219",      TELEM_INA219_MATCH, init_ina219,  query_ina219  },
#endif
#if ENV_INCLUDE_INA260
  { TELEM_INA260_ADDR_FIRST,  TELEM_INA260_ADDR_LAST,     "INA260",      match_ina260,       init_ina260,  query_ina260  },
#endif
#if ENV_INCLUDE_INA226
  { TELEM_INA226_ADDR_FIRST,  TELEM_INA226_ADDR_LAST,     "INA226",      match_ina226,       init_ina226,  query_ina226  },
#endif
#if ENV_INCLUDE_INA228
  { TELEM_INA228_ADDR_FIRST,  TELEM_INA228_ADDR_LAST,     "INA228",      match_ina228,       init_ina228,  query_ina228  },
#endif
#if ENV_INCLUDE_MLX90614
  { TELEM_ADDR(TELEM_MLX90614_ADDRESS),                   "MLX90614",    nullptr, init_mlx90614,    query_mlx90614    },
#endif
#if ENV_INCLUDE_VL53L0X
  { TELEM_ADDR(TELEM_VL53L0X_ADDRESS),                    "VL53L0X",     nullptr, init_vl53l0x,     query_vl53l0x     },
#endif
#ifdef ENV_INCLUDE_BMP085
  { TELEM_ADDR(0x77),                                     "BMP085",      nullptr, init_bmp085,      query_bmp085      },
#endif
#if ENV_INCLUDE_RAK12035
  { TELEM_ADDR(TELEM_RAK12035_ADDRESS),                   "RAK12035",    nullptr, init_rak12035,    query_rak12035    },
#endif
  { 0, 0, nullptr, nullptr, nullptr, nullptr }  // sentinel — keeps the array non-empty
};

#undef TELEM_BOSCH_ALT_ADDR
#undef TELEM_ADDR

static const size_t SENSOR_TABLE_SIZE = (sizeof(SENSOR_TABLE) / sizeof(SENSOR_TABLE[0])) - 1;

// ============================================================
// begin() — scan the I2C bus, then initialize only what was
// found. A sensor whose address does not ACK during the scan
// is never touched by a library call, preventing hangs or
// crashes caused by absent or misbehaving hardware.
// ============================================================

bool EnvironmentSensorManager::begin() {
  #if ENV_INCLUDE_GPS
  #ifdef RAK_WISBLOCK_GPS
  rakGPSInit();
  #else
  initBasicGPS();
  #endif
  #endif

  #if ENV_PIN_SDA && ENV_PIN_SCL
    #ifdef NRF52_PLATFORM
  Wire1.setPins(ENV_PIN_SDA, ENV_PIN_SCL);
  Wire1.setClock(100000);
  Wire1.begin();
    #else
  Wire1.begin(ENV_PIN_SDA, ENV_PIN_SCL, 100000);
    #endif
  MESH_DEBUG_PRINTLN("Second I2C initialized on pins SDA: %d SCL: %d", ENV_PIN_SDA, ENV_PIN_SCL);
  #endif

  // Scan the I2C bus before touching any sensor library.
  bool detected[128] = {};
  scanI2CBus(TELEM_WIRE, detected);

  // Walk the sensor table and initialize only detected devices.
  _active_sensor_count = 0;
  for (size_t i = 0; i < SENSOR_TABLE_SIZE && _active_sensor_count < MAX_ACTIVE_SENSORS; i++) {
    const SensorDef& def = SENSOR_TABLE[i];
    // One static driver instance per type: an alternate address is a fallback, not a second device.
    bool already_active = false;
    for (int j = 0; j < _active_sensor_count; j++) {
      if (_active_sensors[j].query == def.query) { already_active = true; break; }
    }
    if (already_active) continue;

    // Try each detected address in the entry's range; the first one that
    // matches and initializes wins. uint16_t avoids wrap-around at 0xFF.
    bool candidate_seen = false;
    bool initialized    = false;
    for (uint16_t a = def.addr_first; a <= def.addr_last && !initialized; a++) {
      const uint8_t addr = (uint8_t)a;
      if (!detected[addr]) continue;
      if (def.match && !def.match(TELEM_WIRE, addr)) continue;
      candidate_seen = true;

      uint8_t n = def.init(TELEM_WIRE, addr);
      if (n == 0) {
        MESH_DEBUG_PRINTLN("%s found at %02X but failed to initialize", def.name, addr);
        continue;
      }
      MESH_DEBUG_PRINTLN("Found %s at address: %02X", def.name, addr);
      detected[addr] = false;  // consumed; later entries must not re-claim this device
      for (uint8_t sub = 0; sub < n && _active_sensor_count < MAX_ACTIVE_SENSORS; sub++) {
        _active_sensors[_active_sensor_count++] = { def.query, sub };
      }
      initialized = true;
    }

    if (!candidate_seen) {
      if (def.addr_first == def.addr_last) {
        MESH_DEBUG_PRINTLN("%s not detected at I2C address %02X", def.name, def.addr_first);
      } else {
        MESH_DEBUG_PRINTLN("%s not detected in I2C range %02X-%02X", def.name, def.addr_first, def.addr_last);
      }
    }
  }

  return true;
}

// ============================================================
// querySensors() — GPS stays on channel 1; each active sensor
// gets the next available channel in the order it was
// initialized.
// ============================================================

bool EnvironmentSensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  next_available_channel = TELEM_CHANNEL_SELF + 1;

  if (requester_permissions & TELEM_PERM_LOCATION && gps_active) {
    telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude);
  }

  if (requester_permissions & TELEM_PERM_ENVIRONMENT) {
    for (int i = 0; i < _active_sensor_count; i++) {
      _active_sensors[i].query(next_available_channel, _active_sensors[i].sub_channel, telemetry);
      next_available_channel++;
    }
  }

  return true;
}


int EnvironmentSensorManager::getNumSettings() const {
  int settings = 0;
  #if ENV_INCLUDE_GPS
    if (gps_detected) settings++;  // only show GPS setting if GPS is detected
  #endif
  return settings;
}

const char* EnvironmentSensorManager::getSettingName(int i) const {
  int settings = 0;
  #if ENV_INCLUDE_GPS
    if (gps_detected && i == settings++) {
      return "gps";
    }
  #endif
  return NULL;
}

const char* EnvironmentSensorManager::getSettingValue(int i) const {
  int settings = 0;
  #if ENV_INCLUDE_GPS
    if (gps_detected && i == settings++) {
      return gps_active ? "1" : "0";
    }
  #endif
  return NULL;
}

bool EnvironmentSensorManager::setSettingValue(const char* name, const char* value) {
  #if ENV_INCLUDE_GPS
  if (gps_detected && strcmp(name, "gps") == 0) {
    if (strcmp(value, "0") == 0) {
      // PowerSaving variant
      gps_active = false; // Disabled by CLI or App
      stop_gps();
    } else {
      // PowerSaving variant
      gps_active = true; // Enabled by CLI or App
      start_gps();
    }
    return true;
  }
  if (strcmp(name, "gps_interval") == 0) {
    uint32_t interval_seconds = atoi(value);
    gps_update_interval_sec = interval_seconds > 0 ? interval_seconds : 1;
    return true;
  }
  #endif
  return false;  // not supported
}

#if ENV_INCLUDE_GPS
void EnvironmentSensorManager::initBasicGPS() {

  Serial1.setPins(PIN_GPS_TX, PIN_GPS_RX);

  #ifdef GPS_BAUD_RATE
  Serial1.begin(GPS_BAUD_RATE);
  #else
  Serial1.begin(9600);
  #endif

  // Try to detect if GPS is physically connected to determine if we should expose the setting
  _location->begin();
  _location->reset();

  #ifndef PIN_GPS_EN
    MESH_DEBUG_PRINTLN("No GPS wake/reset pin found for this board. Continuing on...");
  #endif

  // Give the GPS time to power up and emit its first NMEA sentence. Cold-start
  // modules can take several seconds, so poll for up to ~5s instead of checking
  // once after a fixed 1s delay -- returning as soon as any data appears, so
  // fast modules add no boot delay (issue #2258).
#ifdef ENV_SKIP_GPS_DETECT
  gps_detected = true;
#else
  gps_detected = false;
  for (int i = 0; i < 20 && !gps_detected; i++) {
    delay(250);
    gps_detected = (Serial1.available() > 0);
  }
#endif

  if (gps_detected) {
    MESH_DEBUG_PRINTLN("GPS detected");
    #ifdef PERSISTANT_GPS
      gps_active = true;
      return;
    #endif
  } else {
    MESH_DEBUG_PRINTLN("No GPS detected");
  }
  _location->stop();
  gps_active = false; //Set GPS visibility off until setting is changed
}

// gps code for rak might be moved to MicroNMEALoactionProvider
// or make a new location provider ...
#ifdef RAK_WISBLOCK_GPS
void EnvironmentSensorManager::rakGPSInit(){

  Serial1.setPins(PIN_GPS_TX, PIN_GPS_RX);

  #ifdef GPS_BAUD_RATE
  Serial1.begin(GPS_BAUD_RATE);
  #else
  Serial1.begin(9600);
  #endif

  // search for the correct IO standby pin depending on socket used
  if (gpsIsAwake(WB_IO2)) {
    _location->setPinEn(WB_IO2); // WB_IO2 is the power switch for all sensor and IO slots
  } else {
    MESH_DEBUG_PRINTLN("No GPS found");
    gps_active = false;
    gps_detected = false;
    Serial1.end();
    return;
  }

  #ifndef FORCE_GPS_ALIVE // for use with repeaters, until GPS toggle is implimented
  //Now that GPS is found and set up, set to sleep for initial state
  stop_gps();
  #endif
}

bool EnvironmentSensorManager::gpsIsAwake(uint8_t ioPin){

  #if defined(ETHERNET_ENABLED) && defined(RAK_BOARD)
    if (ioPin == WB_IO2) {
      // WB_IO2 powers the Ethernet module on RAK baseboards.
      return false;
    }
  #endif

  #ifndef RAK_3401
  //set initial waking state
  pinMode(ioPin,OUTPUT);
  digitalWrite(ioPin,LOW);
  delay(500);
  digitalWrite(ioPin,HIGH);
  #endif

  // give gps time to power up
  delay(500);

  //Try to init RAK12500 on I2C
  if (ublox_GNSS.begin(Wire) == true){
    MESH_DEBUG_PRINTLN("RAK12500 GPS init correctly with pin %i",ioPin);
    ublox_GNSS.setI2COutput(COM_TYPE_UBX);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_IMES);
    ublox_GNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS);
    ublox_GNSS.setMeasurementRate(1000);
    ublox_GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
    gpsResetPin = ioPin;
    i2cGPSFlag = true;
    gps_active = true;
    gps_detected = true;

    _location = &RAK12500_provider;
    return true;
  } else if (Serial1.available()) { // RAK12501 (L76K) on UART
    MESH_DEBUG_PRINTLN("Serial GPS init correctly and is turned on");
    gpsResetPin = ioPin;  // WB_IO2
    serialGPSFlag = true;
    gps_active = true;
    gps_detected = true;
    return true;
  }

  #ifndef RAK_3401
  pinMode(ioPin, INPUT);
  #endif
  MESH_DEBUG_PRINTLN("GPS did not init with this IO pin... try the next");
  return false;
}
#endif

void EnvironmentSensorManager::start_gps() {
  // PowerSaving variant
  if (_location->isPowerSavingEnabled()) {
    gps_wake = true;           // gps_active is unchanged (true for GPS sleep, false for GPS off)
    _location->syncTime();     // Clear GPS data and force sync time
    _location->setNextSleep(); // Next time to off
  } else {
     gps_active = true;
     gps_wake = true;
  }

  #ifdef RAK_WISBLOCK_GPS
    #ifndef RAK_3401
    pinMode(gpsResetPin, OUTPUT);
    digitalWrite(gpsResetPin, HIGH);
    #endif
    return;
  #endif

  _location->begin();
  _location->reset();

#ifndef PIN_GPS_EN
  MESH_DEBUG_PRINTLN("Start GPS is N/A on this board. Actual GPS state unchanged");
#endif
}

void EnvironmentSensorManager::stop_gps() {
  // PowerSaving variant
  if (_location->isPowerSavingEnabled()) {
    gps_wake = false;          // gps_active is unchanged (true for GPS sleep, false for GPS off)
    _location->stopTimeSync(); // Stop time sync
    _location->setNextWake();  // Next time to on
  } else {
    gps_active = false;
    gps_wake = false;
  }

  #ifdef RAK_WISBLOCK_GPS
    #ifndef RAK_3401 // rak3401 shouldn't turn off WB_IO2 as it powers the PA
    pinMode(gpsResetPin, OUTPUT);
    digitalWrite(gpsResetPin, LOW);
    #endif
    return;
  #endif

  _location->stop();

  #ifndef PIN_GPS_EN
  MESH_DEBUG_PRINTLN("Stop GPS is N/A on this board. Actual GPS state unchanged");
  #endif
}
#endif // ENV_INCLUDE_GPS

#if ENV_INCLUDE_GPS || defined(ENV_INCLUDE_BME680_BSEC)
void EnvironmentSensorManager::loop() {

  #if ENV_INCLUDE_GPS
  static long next_gps_update = 0;

  // PowerSaving variant
  if (_location->isPowerSavingEnabled() && gps_detected && gps_active) {
    // Handle change in PowerSaving mode
    _location->updatePowerSavingSettings(gps_wake);

    if (gps_wake) {
      // GPS is awake: check whether it should sleep
      if ((int32_t)(millis() - _location->getNextSleep()) >= 0) {
        POWERSAVING_DEBUG_PRINTLN("GPS wake timeout. Enter sleep");
        stop_gps();
      } else if (!_location->waitingTimeSync()) {
        POWERSAVING_DEBUG_PRINTLN("GPS set. Enter sleep early");
        stop_gps();
      }
    } else {
      // GPS is asleep: check whether it should wake
      if ((int32_t)(millis() - _location->getNextWake()) >= 0) {
        POWERSAVING_DEBUG_PRINTLN("GPS sleep timeout. Wakeup.");
        start_gps();
      } else if (_location->waitingTimeSync()) {
        POWERSAVING_DEBUG_PRINTLN("CLI gps sync. Wakeup");
        start_gps();
      }
    }
  }

  if ((!_location->isPowerSavingEnabled() && gps_active) || (_location->isPowerSavingEnabled() && gps_wake)) {
    _location->loop();
  }

  // PowerSaving variant
  if ((int32_t)(millis() - next_gps_update) >= 0) {
    if((!_location->isPowerSavingEnabled() && gps_active) || (_location->isPowerSavingEnabled() && gps_wake)){
    #ifdef RAK_WISBLOCK_GPS
    if ((i2cGPSFlag || serialGPSFlag) && _location->isValid()) {
      node_lat = ((double)_location->getLatitude())/1000000.;
      node_lon = ((double)_location->getLongitude())/1000000.;
      MESH_DEBUG_PRINTLN("lat %f lon %f", node_lat, node_lon);
      node_altitude = ((double)_location->getAltitude()) / 1000.0;
      MESH_DEBUG_PRINTLN("lat %f lon %f alt %f", node_lat, node_lon, node_altitude);
    }
    #else
    if (_location->isValid()) {
      node_lat = ((double)_location->getLatitude())/1000000.;
      node_lon = ((double)_location->getLongitude())/1000000.;
      MESH_DEBUG_PRINTLN("lat %f lon %f", node_lat, node_lon);
      node_altitude = ((double)_location->getAltitude()) / 1000.0;
      MESH_DEBUG_PRINTLN("lat %f lon %f alt %f", node_lat, node_lon, node_altitude);
    }
    #endif

      // In powersaving mode, GPS is on and off. Only update data when GPS is on
      if(_location->isPowerSavingEnabled()) next_gps_update = millis() + (gps_update_interval_sec * 1000);
    }

    if(!_location->isPowerSavingEnabled()) next_gps_update = millis() + (gps_update_interval_sec * 1000);
  }
  #endif
  #if ENV_INCLUDE_BME680_BSEC
  if (bsec_active && bsec_iaq.run()) {
    uint8_t prev_accuracy = bsec_accuracy;
    bsec_temperature  = bsec_iaq.temperature;
    bsec_humidity     = bsec_iaq.humidity;
    bsec_pressure_hpa = bsec_iaq.pressure / 100.0f;
    bsec_iaq_val      = bsec_iaq.iaq;
    bsec_accuracy     = bsec_iaq.iaqAccuracy;
    bsec_data_ready   = true;

    if (bsec_accuracy == 3) {
      if (!bsec_first_save_done) {
        bsec_save_state();
        bsec_last_save_ms = millis();
        bsec_first_save_done = true;
      } else if ((millis() - bsec_last_save_ms) >= BSEC_SAVE_INTERVAL_MS) {
        bsec_save_state();
        bsec_last_save_ms = millis();
      }
    }
  }
  #endif  // ENV_INCLUDE_BME680_BSEC
}
#endif // ENV_INCLUDE_GPS || ENV_INCLUDE_BME680_BSEC
