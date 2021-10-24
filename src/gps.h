#pragma once

#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define GPS_TX 12
#define GPS_RX 15

class gps
{
    public:
        void init();
        bool checkGpsFix();
        void buildPacket(uint8_t txBuffer[9]);
        void encode();
        void screenPrint();
        TinyGPSAltitude getAltitude();
        TinyGPSInteger getSatellites();
        TinyGPSLocation getLocation();
        TinyGPSTime getTime();
        void gps_time(char * buffer, uint8_t size);

    private:
        uint32_t LatitudeBinary, LongitudeBinary;
        uint16_t altitudeGps;
        uint8_t hdopGps;
        char t[32]; // used to sprintf for Serial output
        TinyGPSPlus tGps;
};
