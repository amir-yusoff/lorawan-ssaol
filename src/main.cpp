#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#define MY_DEBUG

int BATTERY_SENSE_PIN = A3;
int SOLAR_SENSE_PIN = A4;
int LED_STATUS_PIN = A0;

static const PROGMEM u1_t NWKSKEY[16] = {0x7A, 0xE0, 0xBC, 0xC9, 0x49, 0x6C, 0x51, 0x76, 0x38, 0x24, 0x11, 0xBA, 0xE1, 0x1F, 0x5E, 0xAB};
static const u1_t PROGMEM APPSKEY[16] = {0x65, 0x9A, 0x07, 0xFE, 0x1B, 0x89, 0x10, 0xD9, 0x76, 0xF5, 0x71, 0xCB, 0x28, 0xD4, 0x0F, 0xE4};
static const u4_t DEVADDR = 0x26041723;

void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

static osjob_t sendjob;

const unsigned TX_INTERVAL = 20;

// ************************************************************************************************
//                                         pin mapping
// ************************************************************************************************
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = { 8, 7 , LMIC_UNUSED_PIN },
};

// ************************************************************************************************
//                                      prototype functions
// ************************************************************************************************
void onEvent(ev_t ev);
void do_send(osjob_t *j);
float readBat();
float readSol();

// ************************************************************************************************
//                                         arduino loop
// ************************************************************************************************
void setup()
{
  pinMode(LED_STATUS_PIN, OUTPUT);
  while (!Serial)
    ; // wait for Serial to be initialized
  Serial.begin(115200);
  delay(100); // per sample code on RF_95 test
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
  unsigned long now;
  now = millis();
  if ((now & 512) != 0)
  {
    digitalWrite(LED_STATUS_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_STATUS_PIN, LOW);
  }

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
    uint16_t b = readBat() * 100;
    uint16_t s = readSol() * 100;
    byte buffer[4];
    buffer[0] = b >> 8;
    buffer[1] = b;
    buffer[2] = s >> 8;
    buffer[3] = s;
    LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

float readBat()
{
  int battSensorValue = analogRead(BATTERY_SENSE_PIN);
  //float batteryV = battSensorValue * 0.006383;
  float batteryV = battSensorValue * (5.0 / 1023.0) * 2;

#ifdef MY_DEBUG
  Serial.print("Sensor reading: ");
  Serial.print(battSensorValue);
  Serial.print(" | ");
  Serial.print("Battery Voltage: ");
  Serial.print(batteryV);
  Serial.println(" V");
#endif

  return batteryV;
}

float readSol()
{
  int solSensorValue = analogRead(SOLAR_SENSE_PIN);
  float solarV = solSensorValue * (5.0 / 1023.0) * 2;

#ifdef MY_DEBUG
  Serial.print("Sensor reading: ");
  Serial.print(solSensorValue);
  Serial.print(" | ");
  Serial.print("Solar Voltage: ");
  Serial.print(solarV);
  Serial.println(" V");
#endif

  return solarV;
}