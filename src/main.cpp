/*****************************************
* ESP32 GPS VKEL 9600 Bds
******************************************/
#include <stdint.h>
#include <TinyGPS++.h>  
#include "screen.h"
#include "lmic.h"
#include <hal/hal.h>
#include "config.h"

TinyGPSPlus gps;                            
HardwareSerial GPSSerial(1);                
int nbloop;
uint8_t txBuffer[10];

void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = (1*60);

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void buildPacket(TinyGPSPlus tGps, uint8_t txBuffer[10]){
  uint32_t LatitudeBinary, LongitudeBinary;
  uint16_t altitudeGps;
  uint8_t hdopGps;
  uint8_t satGps;

  LatitudeBinary = ((tGps.location.lat() + 90) / 180.0) * 16777215;
  LongitudeBinary = ((tGps.location.lng() + 180) / 360.0) * 16777215;

  txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;

  txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;

  altitudeGps = tGps.altitude.meters();
  txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  hdopGps = tGps.hdop.value()/10;
  txBuffer[8] = hdopGps & 0xFF;

  satGps = tGps.satellites.value();
  txBuffer[9] = satGps & 0xFF;
}

void encode(){      
    int previousMillis = millis();

    while((previousMillis + 1000) > millis()){
        while (GPSSerial.available() ){
            char data = GPSSerial.read();
            gps.encode(data);
        }
    }
}

bool checkGpsFix(){
  encode();
  if (gps.location.isValid() && 
      gps.location.age() < 2000 &&
      gps.hdop.isValid() &&
      gps.hdop.value() <= 300 &&
      gps.hdop.age() < 2000 &&
      gps.altitude.isValid() && 
      gps.altitude.age() < 2000 ){
    Serial.println("Valid gps Fix.");
    return true;
  } else {
    Serial.println("No gps Fix.");
    Serial.println(gps.location.lat());
    return false;
  }
}

void do_send(osjob_t* j){
    // Payload to send (uplink)

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
      if (checkGpsFix()){
        nbloop++;

        Serial.print("Latitude  : ");
        Serial.println(gps.location.lat(), 5);
        Serial.print("Longitude : ");
        Serial.println(gps.location.lng(), 4);
        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());
        Serial.print("Altitude  : ");
        Serial.print(gps.altitude.feet() / 3.2808);
        Serial.println("M");
        Serial.print("Time      : ");
        Serial.print(gps.time.hour());
        Serial.print(":");
        Serial.print(gps.time.minute());
        Serial.print(":");
        Serial.println(gps.time.second());
        Serial.print("Loot: ");
        Serial.println(nbloop);
        Serial.println("**********************");

        buildPacket(gps, txBuffer);
        LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);

        Serial.println(F("Sending uplink packet..."));
      } else {
        //try again in 3 seconds
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
      }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(9600); //115200);
  delay(2500);
  GPSSerial.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX
  GPSSerial.setTimeout(2);

  Serial.println(F("Setup"));

  screen_setup();
  screen_show_logo();
  screen_update();
  delay(5000);

  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
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
  LMIC_setDrTxpow(DR_SF9,14);

  // Start job
  do_send(&sendjob); 
}

void onEvent (ev_t ev) {
    Serial.println(F("onEvent"));
    
    Serial.print(os_getTime());
    Serial.print(": ");
    screen_clear();
            
    switch(ev) {
           case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

            if (LMIC.txrxFlags & TXRX_ACK) {
              Serial.println(F("Received ack"));
              screen_print ("Received ACK.", 0, 20);
            }

            if (LMIC.dataLen) {
              // data received in rx slot after tx
              Serial.print(F("Data Received: "));
              Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              Serial.println();
              Serial.println(LMIC.rssi);
            }

            screen_print("Lat : ");
            char lat[8];  
            dtostrf(gps.location.lat(), 6, 2, lat);
            screen_print(lat);
            screen_print("\n");

            screen_print("Lng : ");
            char lng[8];  
            dtostrf(gps.location.lng(), 6, 2, lng);
            screen_print(lng);
            screen_print("\n");

            screen_print("Alt : ");
            char alt[8];  
            dtostrf(gps.altitude.feet() / 3.2808, 6, 2, alt);
            screen_print(alt);
            screen_print("\n");

            screen_print("Nb sat : ");
            char sat[8];  
            dtostrf(gps.satellites.value(), 6, 2, sat);
            screen_print(sat);
            screen_print("\n");

            screen_print("Loop : ");
            char val[8];  
            dtostrf(nbloop, 6, 2, val);
            screen_print(val);
            screen_print("\n");

            screen_update();
            
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING: -> Joining..."));
            screen_print( "Joining....",0,0);
            screen_update();
            break;
        case EV_JOINED: {
              Serial.println(F("EV_JOINED"));
              screen_print( "Joined!", 0,0);
              screen_update();
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
