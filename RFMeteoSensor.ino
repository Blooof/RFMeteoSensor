#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "printf.h"
#include <LowPower.h>

#define SLEEP_ENABLED 1
#define DEBUG_ENABLED 0
#define COOLDOWN 1000L
#define SEND_INTERVAL 60000L
#define POT A0
#define ZATVOR D8

RF24 radio(D5, D6);
Adafruit_BME280 bme;

RF24Network network(radio);

const uint16_t this_node = 02;
const uint16_t base_node = 00;

const int PINS[] = {D5, D6, A4, A5, D11, D12, D13, D8};

struct payload_t {
  int16_t magic;
  int16_t schemaAndVersion;
  int16_t temp;
  int16_t hum;
  int32_t pres;
  int16_t volt;
};

void setup(void) {
  analogReference(INTERNAL);
  pinMode(ZATVOR, OUTPUT);
  digitalWrite(ZATVOR, HIGH);
  delay(10);

  if (DEBUG_ENABLED) {
    Serial.begin(115200);
    printf_begin();
    Serial.println("Starting RFMeteoSensor");
  }

  if (!bme.begin()) {
    if (DEBUG_ENABLED) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }
    while (1);
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X2, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_X2);

  SPI.begin();
  radio.begin();
  delay(100);
  radio.setDataRate(RF24_1MBPS);
  radio.setCRCLength(RF24_CRC_8);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(105);
  network.begin(105, this_node);
  delay(100);
  if (DEBUG_ENABLED) {
    radio.printDetails();
  }
}

long lastSent;

void loop() {
  network.update();

  long now = millis();
  long diff = now - lastSent;
  if ((lastSent == 0 && diff >= COOLDOWN) || (diff >= SEND_INTERVAL)) {
    lastSent = now;

    bme.takeForcedMeasurement();

    sendData();

    if (SLEEP_ENABLED) {
      digitalWrite(ZATVOR, LOW);
      for (int pin = 0; pin < sizeof(PINS) / sizeof(PINS[0]); pin++) {
        digitalWrite(PINS[pin], LOW);
        pinMode(PINS[pin], INPUT);
      }
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }
  }
}

void sendData() {
  payload_t payload;
  payload.magic = 0xB10F;
  payload.schemaAndVersion = getSchemaVersion(1, 1);
  if (DEBUG_ENABLED) {
    payload.schemaAndVersion = getSchemaVersion(1025, 1);
  }
  float val = bme.readTemperature();
  payload.temp = isnan(val) ? (int16_t)0xFFFF : (int16_t)(val * 100);
  val = bme.readPressure();
  payload.pres = isnan(val) ? (int32_t)0xFFFFFFFFL : (int32_t)val;
  val = bme.readHumidity();
  payload.hum = isnan(val) ? (int16_t)0xFFFF : (int16_t)(val * 100);
  payload.volt = (int16_t)(readBatteryVoltage() * 100);

  RF24NetworkHeader header(base_node);
  int sz = sizeof(payload);
  if (DEBUG_ENABLED) {
    Serial.println(sz);
    Serial.print("Sending...");
  }
  bool ok = network.write(header, &payload, sz);
  if (DEBUG_ENABLED) {
    if (ok) {
      Serial.println("ok.");
    } else {
      Serial.println("failed.");
    }
    Serial.flush();
  }
}

float readBatteryVoltage() {
  return analogRead(POT) * 0.002652;
}

// 0 <= schema <= 1<<13-1 (8191)
// 0 <= version <= 1<<4-1 (15)
int16_t getSchemaVersion(int schema, int version) {
  return (schema << 4) | version;
}
