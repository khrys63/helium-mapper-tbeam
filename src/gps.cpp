#include "gps.h"
#include "config.h"

HardwareSerial GPSSerial(GPS_SERIAL_NUM);

void gps::init(){  
    GPSSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    GPSSerial.setTimeout(2);
}

void gps::encode(){       
    int previousMillis = millis();

    while((previousMillis + 1000) > millis()){
        while (GPSSerial.available() ){
            char data = GPSSerial.read();
            tGps.encode(data);
        }
    }
}

void gps::buildPacket(uint8_t txBuffer[10]){  
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

bool gps::checkGpsFix(){
    encode();
    if (tGps.location.isValid() && 
        tGps.location.age() < 2000 &&
        tGps.hdop.isValid() &&
        tGps.hdop.value() <= 300 &&
        tGps.hdop.age() < 2000 &&
        tGps.altitude.isValid() && 
        tGps.altitude.age() < 2000 ){
        return true;
    } else {
        return false;
    }
}

TinyGPSAltitude gps::getAltitude(){
    return tGps.altitude;
}

TinyGPSInteger gps::getSatellites(){
    return tGps.satellites;
}

TinyGPSLocation gps::getLocation(){
    return tGps.location;
}

TinyGPSTime gps::getTime(){
    return tGps.time;
}