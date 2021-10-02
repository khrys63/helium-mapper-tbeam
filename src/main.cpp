/*****************************************
* ESP32 GPS VKEL 9600 Bds
******************************************/
#include <stdint.h>
#include <TinyGPS++.h>  
#include "screen.h"
#include <lmic.h>
#include <hal/hal.h>
#include "config.h"
#include <WiFi.h>
#include "gps.h"
             
int nbloop;
uint8_t txBuffer[10];
gps gpsSensor;
bool goForMapping;
bool initiedGPS;

void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = (1*25);

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void displayInfo(){
    Serial.print("Latitude  : ");
    Serial.println(gpsSensor.getLocation().lat(), 5);
    Serial.print("Longitude : ");
    Serial.println(gpsSensor.getLocation().lng(), 4);
    Serial.print("Satellites: ");
    Serial.println(gpsSensor.getSatellites().value());
    Serial.print("Altitude  : ");
    Serial.print(gpsSensor.getAltitude().feet() / 3.2808);
    Serial.println("M");
    Serial.print("Time      : ");
    Serial.print(gpsSensor.getTime().hour());
    Serial.print(":");
    Serial.print(gpsSensor.getTime().minute());
    Serial.print(":");
    Serial.println(gpsSensor.getTime().second());
    Serial.print("Loop: ");
    Serial.println(nbloop);
    Serial.println("**********************");

    screen_print("Lat : ");
    char lat[8];  
    dtostrf(gpsSensor.getLocation().lat(), 6, 2, lat);
    screen_print(lat);
    screen_print("\n");

    screen_print("Lng : ");
    char lng[8];  
    dtostrf(gpsSensor.getLocation().lng(), 6, 2, lng);
    screen_print(lng);
    screen_print("\n");

    screen_print("Alt : ");
    char alt[8];  
    dtostrf(gpsSensor.getAltitude().feet() / 3.2808, 6, 2, alt);
    screen_print(alt);
    screen_print("\n");

    screen_print("Nb sat : ");
    char sat[8];  
    dtostrf(gpsSensor.getSatellites().value(), 6, 2, sat);
    screen_print(sat);
    screen_print("\n");

    screen_update();
}

void do_send(osjob_t* j){
    // Payload to send (uplink)

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      // Prepare upstream data transmission at the next possible time.
      // first time, just joining
      if (!goForMapping){
        //connecting to network
        Serial.println("Connecting to network");
        LMIC_setTxData2(2, txBuffer, sizeof(txBuffer), 1);
      }else{
        LMIC_setDrTxpow(DR_SF9,14);
        if (!initiedGPS){
          Serial.println(F("Init GPS"));
          screen_print ("Init GPS....", 0, 40);
          screen_update();
          initiedGPS = true;
          gpsSensor.init();
        }
        //mapping active
        if (gpsSensor.checkGpsFix()){
          nbloop++;

          gpsSensor.buildPacket(txBuffer);
          
          displayInfo();
          
          LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);

          Serial.println(F("Sending uplink packet..."));
        } else {
          //try again in 3 seconds
          os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
        }
      }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(9600); //115200);
  delay(2500);

  Serial.println(F("Setup"));

  //Turn off WiFi and Bluetooth
  Serial.println(F("Kill Wifi & BT"));
  WiFi.mode(WIFI_OFF);
  btStop();

  initiedGPS=false;

  if (screenPresent){
    screen_setup();
    screen_show_logo();
    screen_update();
  }
  delay(5000);

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);
  LMIC_setAdrMode(0);
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  //LMIC_setDrTxpow(DR_SF11,14);
  LMIC_setDrTxpow(DR_SF12,16);
  //LMIC_setDrTxpow(DR_SF9,14);

  // Start GPS job
  do_send(&sendjob); 
}

void onEvent (ev_t ev) {
    Serial.println(F("onEvent"));
    
    Serial.print(os_getTime());
    Serial.print(": ");
            
    switch(ev) {
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

            if (LMIC.txrxFlags & TXRX_ACK) {
              Serial.println(F("Received ack"));
              screen_print ("Received ACK !", 0, 20);
              screen_update();
            }

            if (LMIC.dataLen) {
              // data received in rx slot after tx
              Serial.print(F("Data Received: "));
              Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              Serial.println();
              Serial.println(LMIC.rssi);
            }
            
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING: -> Joining..."));
            screen_clear();
            screen_print( "Joining....",0,0);
            screen_update();
            break;
        case EV_JOINED: {
              Serial.println(F("EV_JOINED"));
              screen_clear();
              screen_print( "Joined !", 0,0);
              screen_update();
              goForMapping=true;
              // Disable link check validation (automatically enabled
              // during join, but not supported by TTN at this time).
              LMIC_setLinkCheckMode(0);
            }
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
        default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void loop(){
  os_runloop_once();
}
