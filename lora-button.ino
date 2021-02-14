
// simple sketch for Lora Button board by Matias Meier / Iotdevices
// see here for details: https://www.tindie.com/products/iotdevices/lora-buttoninterrupt-node-arduino-compatible/ and https://owncloud.matias.ch/index.php/s/nqF8Y4FDsCzGJ29
// 
// this code was initially published here: 
// and is based on my mini-lora example https://github.com/crox-net/mini-lora-examples, which in turn is based on the sample code that comes with the Arduino LMIC library.
// see also README

#define DEBUG 0
#define LEDON 0

#define DISABLE_BEACONS 1

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <CayenneLPP.h>

// max. size of the buffer, see doc.
CayenneLPP lpp(12);

// see ttn_secrets_template.h
#include "ttn_secrets_button1.h"
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = 6,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,
  .dio = {2, 3, 4},
};

boolean btnPressed = 0;

#define LED 13

// voltage divider for battery measurement, see doc.
const float AVCC = 1.077;  // needs to be measured for each board, see doc.
const int8_t BATRGND = 1;  // Resistor value in Mega-Ohm to GND (R4)
const float BATRRAW = 4.7; // Resistor value in Mega-Ohm to RAW (R3)
const int batteryPin = A0;
const int enableVDPin = PD7;

// Wakeup from sleep with button at A2
#include "LowPower.h"  // https://github.com/rocketscream/Low-Power
#include "PinChangeInterrupt.h" // https://github.com/NicoHood/PinChangeInterrupt
const int wakeUpPin = A2;

void buttonPressed() {
  // we set this flag when the button was pressed (or released actually)
  btnPressed = 1;
}

void onEvent (ev_t ev) {
    #if DEBUG
    Serial.print(os_getTime());
    Serial.print(": ");
    #endif
    switch(ev) {
        #if DEBUG
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
        #endif
        case EV_JOINED:
            #if DEBUG
            Serial.println(F("EV_JOINED"));
            #endif
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        #if DEBUG
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        #endif
        case EV_TXCOMPLETE:
            #if DEBUG
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            #endif
            if (LMIC.txrxFlags & TXRX_ACK) {
              #if DEBUG
              Serial.println(F("Received ack"));
              #endif
            }
            if (LMIC.dataLen) {
              #if DEBUG
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              #endif
            }
            delay(50);
            // now we sleep
            #if DEBUG
            Serial.println("going to sleep");
            delay(100);
            #endif
            LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
            // we land here once we "wake up"
            #if DEBUG
            delay(100);
            Serial.println("just woke up!");
            delay(100);
            #endif
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime(), do_send);
            break;
        #if DEBUG
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        #endif
        default:
            #if DEBUG
            Serial.println(F("Unknown event"));
            #endif
            break;
    }
}

float read_vd() {

    #if DEBUG
    Serial.println("read_vd");
    #endif

    pinMode(enableVDPin, OUTPUT);   //set PD7 pin to output/low to enable the voltage divider
    digitalWrite(enableVDPin, LOW);
   
    // the initial readings are not accurate, so we discard the first 4...
    int batteryReading = 0;
    batteryReading = analogRead(batteryPin);
    for (int i = 0; i < 3; i++) {
      delay(10);
      batteryReading += analogRead(batteryPin);
    }
    #if DEBUG
    Serial.print("batteryReading(4): ");
    Serial.print(batteryReading);
    Serial.println();
    #endif

    // ... then we average out the next 8 samples
    batteryReading = 0;
    for (int i = 0; i < 8; i++) {
      delay(10);
      batteryReading += analogRead(batteryPin);
    }

    //set pin as input to disable voltage divider after reading is done
    pinMode(enableVDPin, INPUT);
    
    float volts = ((batteryReading / 8.0 * AVCC / 1024.0) * (BATRRAW + BATRGND) / BATRGND);
        
    #if DEBUG
    Serial.print("batteryReading(8): ");
    Serial.print(batteryReading);
    Serial.println();
    Serial.print("volts*100: ");
    Serial.print(volts*100);
    Serial.println();
    #endif

    // we need to round the results to two decimals, because the rest gets stripped by the Cayenne LPP library
    int v100 = round(volts*100);
    volts = v100/100.0;

    #if DEBUG
    Serial.print("volts: ");
    Serial.print(volts);
    Serial.println();
    #endif

    return volts;
}

void prepare_payload() {

    #if DEBUG
    Serial.println("prepare_payload");
    #endif

    lpp.reset();

    lpp.addDigitalInput(1, btnPressed);  // useful to know whether the transmission was triggered by a pin change or rather by a board reset
    btnPressed = 0;

    lpp.addDigitalInput(2, digitalRead(wakeUpPin));  // useful when used eg as a door sensor
    
    //lpp.addVoltage(3, read_vd());  // looks like this extended type is not supported by TTN
    lpp.addAnalogInput(3, read_vd());

    #if DEBUG
    Serial.println("lpp buffer is now:");
    uint8_t* buf = lpp.getBuffer();
    for (int i = 0; i < lpp.getSize(); i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    #endif
}

void do_send(osjob_t* j){
    #if DEBUG
    Serial.println("do_send");
    #endif
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        #if DEBUG
        Serial.println(F("OP_TXRXPEND, not sending"));
        #endif
    } else {
        // Prepare upstream data transmission at the next possible time.
        prepare_payload();
        if (lpp.getSize() > 0) {
          LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
          #if DEBUG
          Serial.println(F("Packet queued"));
          #endif
        } else {
          #if DEBUG
          Serial.println(F("lpp size <= 0, sending 'ping' message"));
          #endif
          static uint8_t mydata[] = "ping";
          LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
          #if DEBUG
          Serial.println(F("Packet queued"));
          #endif
        }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


void setup() {
  Serial.begin(9600);
  Serial.println(F("Starting"));
  
  // LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  // "wake up" pin
  pinMode(wakeUpPin, INPUT);
  attachPCINT(digitalPinToPCINT(wakeUpPin), buttonPressed, HIGH);

  // VD for reading battery voltage
  analogReference(INTERNAL);
  pinMode(batteryPin, INPUT);
  pinMode(enableVDPin, INPUT); // set as input to disable the voltage divier

  #if LEDON
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  delay(50);
  #endif
  
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // add some tolerance...
  LMIC_setClockError (MAX_CLOCK_ERROR * 10 / 100);

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
