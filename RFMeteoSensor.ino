#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "printf.h"

#define POT A0

RF24 radio(D5, D6);
Adafruit_BME280 bme;

RF24Network network(radio);

const uint16_t this_node = 02;
const uint16_t base_node = 00;

const unsigned long interval = 60000;

unsigned long last_sent;
unsigned int packetId;

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
  
  Serial.begin(115200);
  printf_begin();
  Serial.println("RF24Network/examples/helloworld_tx/");

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X2, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_X2);

  SPI.begin();
  delay(200);
  radio.begin();
  delay(200);
  radio.setDataRate(RF24_1MBPS);
  radio.setCRCLength(RF24_CRC_8);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setChannel(105);
  delay(200);
  network.begin(105, this_node);
  delay(200);
  radio.printDetails();
}

void loop() {
  network.update();

  unsigned long now = millis();
  if (now - last_sent >= interval) {
    last_sent = now;

    payload_t payload = { 0xB10F, getSchemaVersion(1, 1)};
    payload.magic = 0xB10F;
    payload.schemaAndVersion = getSchemaVersion(1, 1);

    bme.takeForcedMeasurement();
    float val = bme.readTemperature();
    payload.temp = isnan(val) ? (int16_t)0xFFFF : (int16_t)(val * 100);
    val = bme.readPressure();
    payload.pres = isnan(val) ? (int32_t)0xFFFFFFFFL : (int32_t)val;
    val = bme.readHumidity();
    payload.hum = isnan(val) ? (int16_t)0xFFFF : (int16_t)(val * 100);
    payload.volt = (int16_t)(readBatteryVoltage() * 100);

    RF24NetworkHeader header(base_node);
    int sz = sizeof(payload);
    Serial.println(sz);
    Serial.print("Sending...");
    bool ok = network.write(header, &payload, sz);
    if (ok)
      Serial.println("ok.");
    else
      Serial.println("failed.");
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
