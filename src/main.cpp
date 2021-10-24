/*****************************************
* ESP32 GPS VKEL 9600 Bds
******************************************/
#include <stdint.h>
#include "screen.h"
#include <lmic.h>
#include <hal/hal.h>
#include "config.h"
#include <WiFi.h>
#include "gps.h"
#include "credentials.h"
#include "axp20x.h"

AXP20X_Class axp;
bool pmu_irq = false;
String baChStatus = "No charging";

int nbloop;
uint8_t txBuffer[10];
gps gpsSensor;
bool goForMapping;
bool initiedGPS;
bool fixedGPS;
bool ssd1306_found = false;
bool axp192_found = false;

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

    screen_clear();
    screen_update();

    char buffer[20];

    // Message count
    snprintf(buffer, sizeof(buffer), "#%03d", nbloop);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawString(0, 2, buffer);

    // Hour
    gpsSensor.gps_time(buffer, sizeof(buffer));
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(display->getWidth()/2, 2, buffer);

    // Satellite count
    display->setTextAlignment(TEXT_ALIGN_RIGHT);
    display->drawString(display->getWidth() - SATELLITE_IMAGE_WIDTH - 4, 2, itoa(gpsSensor.getSatellites().value(), buffer, 10));
    display->drawXbm(display->getWidth() - SATELLITE_IMAGE_WIDTH, 0, SATELLITE_IMAGE_WIDTH, SATELLITE_IMAGE_HEIGHT, SATELLITE_IMAGE);

    screen_print("Lat : ",0,21);
    char lat[8];  
    dtostrf(gpsSensor.getLocation().lat(), 6, 2, lat);
    screen_print(lat, 37,21);

    screen_print("Lng : ",0,30);
    char lng[8];  
    dtostrf(gpsSensor.getLocation().lng(), 6, 2, lng);
    screen_print(lng, 34,30);

    screen_print("Alt : ",0,39);
    char alt[8];  
    dtostrf(gpsSensor.getAltitude().feet() / 3.2808, 6, 2, alt);
    screen_print(alt, 39,39);

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
          screen_print ("Init GPS....", 0, 20);
          screen_update();
          initiedGPS = true;
          gpsSensor.init();
        }
        //mapping active
        if (gpsSensor.checkGpsFix()){
          fixedGPS=true;
          nbloop++;

          gpsSensor.buildPacket(txBuffer);
          
          displayInfo();
          
          LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);

          Serial.println(F("Sending uplink packet..."));
        } else {
          Serial.println("No gps Fix.");
          if (fixedGPS){
            screen_clear();
            screen_update();
            screen_print("GPS Lost !",0,0);
            screen_update();
          }
          //try again in 3 seconds
          os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
        }
      }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void scanI2Cdevice(void) {
    byte err, addr;
    int nDevices = 0;
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16){
                Serial.print("0");
            }
            Serial.print(addr, HEX);
            Serial.println(" !");
            nDevices++;

            if (addr == SSD1306_ADDRESS) {
                ssd1306_found = true;
                Serial.println("ssd1306 display found");
            }
            if (addr == AXP192_SLAVE_ADDRESS) {
                axp192_found = true;
                Serial.println("axp192 PMU found");
            }
        } else if (err == 4) {
            Serial.print("Unknow error at address 0x");
            if (addr < 16){
                Serial.print("0");
            }
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0){
        Serial.println("No I2C devices found\n");
    }else{
        Serial.println("done\n");
    }
}

void axp192Init() {
    if (axp192_found) {
        if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
            Serial.println("AXP192 Begin PASS");
        } else {
            Serial.println("AXP192 Begin FAIL");
        }
        // axp.setChgLEDMode(LED_BLINK_4HZ);
        Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");
        Serial.println("----------------------------------------");

        axp.setPowerOutPut(AXP192_LDO2, AXP202_ON); // LORA radio
        axp.setPowerOutPut(AXP192_LDO3, AXP202_ON); // GPS main power
        axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
        axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
        axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
        axp.setDCDC1Voltage(3300); // for the OLED power

        Serial.printf("DCDC1: %s\n", axp.isDCDC1Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC2: %s\n", axp.isDCDC2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO2: %s\n", axp.isLDO2Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("LDO3: %s\n", axp.isLDO3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("DCDC3: %s\n", axp.isDCDC3Enable() ? "ENABLE" : "DISABLE");
        Serial.printf("Exten: %s\n", axp.isExtenEnable() ? "ENABLE" : "DISABLE");

        pinMode(PMU_IRQ, INPUT_PULLUP);
        attachInterrupt(PMU_IRQ, [] {
            pmu_irq = true;
        }, FALLING);

        axp.adc1Enable(AXP202_BATT_CUR_ADC1, 1);
        axp.enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_BATT_REMOVED_IRQ | AXP202_BATT_CONNECT_IRQ, 1);
        axp.clearIRQ();

        if (axp.isChargeing()) {
            baChStatus = "Charging";
        }
    } else {
        Serial.println("AXP192 not found");
    }
}

void setup() {
  Serial.begin(9600);
  delay(2500);

  Serial.println(F("Setup"));

  initiedGPS=false;
  fixedGPS=false;
  nbloop=0;

  //Turn off WiFi and Bluetooth
  Serial.println(F("Kill Wifi & BT"));
  WiFi.mode(WIFI_OFF);
  btStop();

  scanI2Cdevice();
  axp192Init();

  if (ssd1306_found) screen_setup();
  screen_show_logo();
  screen_update();
  delay(LOGO_DELAY);

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

  goForMapping = false;
  // Start GPS job
  do_send(&sendjob); 
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
            
    switch(ev) {
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

            if (LMIC.txrxFlags & TXRX_ACK) {
              Serial.println(F("Received ack"));
              screen_print ("Received ACK !", 0, 10);
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
            screen_print( "Joining Helium ...",0,0);
            screen_update();
            break;
        case EV_JOINED: {
              Serial.println(F("EV_JOINED"));
              screen_clear();
              screen_print( "Joined Helium !", 0,0);
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
