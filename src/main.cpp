#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "dht.h"

// ************************************************************************************************
//                                         mode select
// ************************************************************************************************
#define AFFAN_MODE
// #define MB_MODE
// #define MA_MODE
// #define MAB_MODE
// ************************************************************************************************
// ************************************************************************************************
// #define MY_DEBUG
// #define LED_STATUS_INDICATOR
#define DHT_MODULE

// sensor and LED pinout
const int BATTERY_SENSE_PIN = 17; // A3
const int SOLAR_SENSE_PIN = 18;   // A4
const int LED_PIN = 9;
#ifdef LED_STATUS_INDICATOR
const int LED_STATUS_PIN = A0;
#endif
#ifdef DHT_MODULE
const int DHT_PIN = 6;
DHT dht(DHT_PIN, DHT22);
#endif
#ifdef MAB_MODE
const int LED_PIN2 = A0;
#endif

// variable declaration
const float VOLTAGE_REFERENCE = 5.00;
bool ledState = LOW;
unsigned long previousMillis = 0;
#ifdef AFFAN_MODE
const float solarThreshold = 1.2;
const int nightIntensity = 9;
#endif
#ifdef MB_MODE
float solarThreshold = 1.6;
unsigned long onInterval = 500;
unsigned long offInterval = 2500;
const int nightIntensity = 10;
#endif
#ifdef MA_MODE
const float solarThreshold = 1.6;
unsigned long onInterval = 500;
unsigned long offInterval = 2500;
const int dayIntensity = 240;
const int nightIntensity = 200;
#endif
#ifdef MAB_MODE
bool ledState2 = LOW;
const float solarThreshold = 1.6;
unsigned long onInterval = 500;
unsigned long offInterval = 2500;
const int dayIntensity = 240;
const int nightIntensity = 180;
#endif

// battery-and-solar-monitoring -> affan-node1
static const PROGMEM u1_t NWKSKEY[16] = {0x25, 0x8E, 0x23, 0xB5, 0x7D, 0xF1, 0x6F, 0x0E, 0x6D, 0x48, 0x66, 0x79, 0x26, 0xA1, 0x89, 0xE3};
static const u1_t PROGMEM APPSKEY[16] = {0x72, 0xFB, 0x11, 0xDD, 0xD7, 0x40, 0x02, 0xB4, 0xA3, 0x22, 0xA5, 0xB7, 0x1D, 0xC9, 0x5B, 0x49};
static const u4_t DEVADDR = 0x26041DBB;

void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

static osjob_t sendjob;

const unsigned TX_INTERVAL = 900;

// ************************************************************************************************
//                                         pin mapping
// ************************************************************************************************
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 8, LMIC_UNUSED_PIN},
};

// ************************************************************************************************
//                                      prototype functions
// ************************************************************************************************
void onEvent(ev_t ev);
void do_send(osjob_t *j);
float readBat();
float readSol();
void printVoltage();

// ************************************************************************************************
//                                         arduino loop
// ************************************************************************************************
void setup()
{
#ifdef LED_STATUS_INDICATOR
  pinMode(LED_STATUS_PIN, OUTPUT);
#endif
  pinMode(LED_PIN, OUTPUT);

  while (!Serial)
    ; // wait for Serial to be initialized
  Serial.begin(115200);
#ifdef DHT_MODULE
  dht.begin();
#endif

  delay(1000); // per sample code on RF_95 test
  Serial.println(F("Starting"));

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF9, 14);

  // Start job
  do_send(&sendjob);
}

void loop()
{
#ifdef AFFAN_MODE
  float nowSolar = readSol();

  if (nowSolar > solarThreshold)
  {
    ledState = LOW;
    digitalWrite(LED_PIN, ledState);
  }
  else if (nowSolar < solarThreshold)
  {
    ledState = HIGH;
    analogWrite(LED_PIN, nightIntensity);
  }
#endif
#ifdef MB_MODE
  unsigned long currentMillis = millis();
  float nowSolar = readSol();
  if (nowSolar < solarThreshold)
  {
    if ((ledState == HIGH) && (currentMillis - previousMillis >= onInterval))
    {
      ledState = LOW;
      previousMillis = currentMillis;
      digitalWrite(LED_PIN, ledState);
    }
    else if ((ledState == LOW) && (currentMillis - previousMillis >= offInterval))
    {
      ledState = HIGH;
      previousMillis = currentMillis;
      analogWrite(LED_PIN, nightIntensity);
    }
  }
  else
  {
    ledState = LOW;
    digitalWrite(LED_PIN, ledState);
  }

#endif
// FIXME: code for MA light blinking
#ifdef MA_MODE
  unsigned long currentMillis = millis();
  float nowSolar = readSol();
  if (nowSolar < solarThreshold)
  {
    if ((ledState == HIGH) && (currentMillis - previousMillis >= onInterval))
    {
      ledState = LOW;
      previousMillis = currentMillis;
      digitalWrite(LED_PIN, ledState);
    }
    else if ((ledState == LOW) && (currentMillis - previousMillis >= offInterval))
    {
      ledState = HIGH;
      previousMillis = currentMillis;
      analogWrite(LED_PIN, nightIntensity);
    }
  }
  else
  {
    if ((ledState == HIGH) && (currentMillis - previousMillis >= onInterval))
    {
      ledState = LOW;
      previousMillis = currentMillis;
      digitalWrite(LED_PIN, ledState);
    }
    else if ((ledState == LOW) && (currentMillis - previousMillis >= offInterval))
    {
      ledState = HIGH;
      previousMillis = currentMillis;
      analogWrite(LED_PIN, dayIntensity);
    }
  }
#endif
// FIXME: code for MAB light blinking
#ifdef MAB_MODE
  unsigned long currentMillis = millis();
  float nowSolar = readSol();
  if (nowSolar < solarThreshold)
  {
    if ((ledState == HIGH) && (currentMillis - previousMillis >= onInterval))
    {
      ledState = LOW;
      previousMillis = currentMillis;
      digitalWrite(LED_PIN, ledState);
    }
    else if ((ledState == LOW) && (currentMillis - previousMillis >= offInterval))
    {
      ledState = HIGH;
      previousMillis = currentMillis;
      analogWrite(LED_PIN, nightIntensity);
    }
  }
  else
  {
    if ((ledState == HIGH) && (currentMillis - previousMillis >= onInterval))
    {
      ledState = LOW;
      previousMillis = currentMillis;
      digitalWrite(LED_PIN, ledState);
    }
    else if ((ledState == LOW) && (currentMillis - previousMillis >= offInterval))
    {
      ledState = HIGH;
      previousMillis = currentMillis;
      analogWrite(LED_PIN, dayIntensity);
    }
  }
#endif

  os_runloop_once();
}

// ************************************************************************************************
//                                      object oriented functions
// ************************************************************************************************

void onEvent(ev_t ev)
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    break;
  case EV_RFU1:
    Serial.println(F("EV_RFU1"));
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.dataLen)
    {
      // data received in rx slot after tx
      Serial.print(F("Received "));
      Serial.print(LMIC.dataLen);
      Serial.print(F(" bytes of payload: 0x"));
      for (int i = 0; i < LMIC.dataLen; i++)
      {
        if (LMIC.frame[LMIC.dataBeg + i] < 0x10)
        {
          Serial.print(F("0"));
        }
        Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);

        if (i == 0) //check the first byte
        {
          if (LMIC.frame[LMIC.dataBeg + 0] == 0x00)
          {
            Serial.print(F(" Yes!!!! "));
          }
        }
      }
      Serial.println();
    }

    /* Schedule next transmission */
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    /* data received in ping slot */
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  default:
    Serial.println(F("Unknown event"));
    break;
  }
}

void do_send(osjob_t *j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    // Prepare upstream data transmission at the next possible time.
    uint16_t b = readBat() * 100.00;
    uint16_t s = readSol() * 100.00;
#ifdef DHT_MODULE
    uint16_t t = dht.readTemperature() * 100.00;
    uint16_t h = dht.readHumidity() * 100.00;
    byte buffer[8];
#else
    byte buffer[4];
#endif
    buffer[0] = b >> 8;
    buffer[1] = b;
    buffer[2] = s >> 8;
    buffer[3] = s;
#ifdef DHT_MODULE
    buffer[4] = t >> 8;
    buffer[5] = t;
    buffer[6] = h >> 8;
    buffer[7] = h;
#endif
    LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
    printVoltage();

    // Serial.print("byte for readbat() ");
    // Serial.println(b);
    // Serial.print("byte for readsol() ");
    // Serial.println(s);

    // for (int i = 0; i < 4; i++)
    // {
    //   Serial.print("Payload ");
    //   Serial.print(i);
    //   Serial.print(": ");
    //   Serial.println(buffer[i]);
    //   //Serial.print(buffer[i]);
    // }
    // Serial.println(F(""));
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

float readBat()
{
  float sensorReading = 0.00;
  float temp = 0.00;

  for (int i = 0; i < 1000; i++)
  {
    temp = analogRead(BATTERY_SENSE_PIN);
    sensorReading += temp;
  }
  float batteryV = (sensorReading / 1000.00) * (VOLTAGE_REFERENCE / 1023.00) * 2.00;
  return batteryV;
}

float readSol()
{
  float sensorReading2 = 0.00;
  float temp2 = 0.00;

  for (int i = 0; i < 1000; i++)
  {
    temp2 = analogRead(SOLAR_SENSE_PIN);
    sensorReading2 += temp2;
  }
  float solarV = (sensorReading2 / 1000.00) * (VOLTAGE_REFERENCE / 1023.00) * 2.00;
  return solarV;
}

void printVoltage()
{
  int battSensorValue = analogRead(BATTERY_SENSE_PIN);
  int solSensorValue = analogRead(SOLAR_SENSE_PIN);

  float batteryV = battSensorValue * (VOLTAGE_REFERENCE / 1023.00) * 2.00;
  float solarV = solSensorValue * (VOLTAGE_REFERENCE / 1023.00) * 2.00;

  Serial.println("");
  Serial.println("****************************************************");
  Serial.print("Battery sensor reading: ");
  Serial.print(battSensorValue);
  Serial.print(" | ");
  Serial.print("Battery Voltage: ");
  Serial.print(batteryV);
  Serial.println(" V");
  Serial.print("Solar sensor reading: ");
  Serial.print(solSensorValue);
  Serial.print(" | ");
  Serial.print("Solar Voltage: ");
  Serial.print(solarV);
  Serial.println(" V");
  Serial.println("****************************************************");
  Serial.println("");
}