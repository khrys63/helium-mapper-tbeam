# Helium coverage mapper with TTGO T-Beam ESP32 device

This is a Helium device based on the TTGO T-Beam development platform using the SSD1306 I2C OLED display. 

:warning: This code works with TTGO T-Beam Rev T22_V07 :warning:

## Setup

1. Use plateformio and setup up a new project based on TTGO T-BEAM board .

2. Install the Arduino IDE libraries:
- mikalhart/TinyGPSPlus@^1.0.2
- thingpulse/ESP8266 and ESP32 OLED driver for SSD1306 displays@^4.2.0
- matthijskooijman/IBM LMIC framework@^1.5.1

3. Edit include/config.h with your device information.
- Device EUI from the console in LSB mode.
- Application EUI from console in LSB mode
- Application Key. This key should be in big endian format (or, since it is not really a number but a block of memory, endianness does not really apply). In practice, a key taken from ttnctl can be copied as-is. Anyway its in MSB mode.

4. Add Helium Mapper integration to your Application 

For Helium mapper https://mappers.helium.com
```C
function Decoder(bytes, port) { 
 var decodedPayload = {};
 
 decodedPayload.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
 decodedPayload.latitude = (decodedPayload.latitude / 16777215.0 * 180) - 90;

 decodedPayload.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
 decodedPayload.longitude = (decodedPayload.longitude / 16777215.0 * 360) - 180;
 
  var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
  var sign = bytes[6] & (1 << 7);
  if(sign)
  {
      decodedPayload.altitude = 0xFFFF0000 | altValue;
  }
  else
  {
      decodedPayload.altitude = altValue;
  }
  
 decodedPayload.accuracy= 10;

 return decodedPayload;
}
```

For Cargo integration https://cargo.helium.com
```C
function Decoder(bytes, port) {
    var decoded = {};

    decoded.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;
  
    decoded.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;
  
    var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
    var sign = bytes[6] & (1 << 7);
    if(sign)
    {
        decoded.altitude = 0xFFFF0000 | altValue;
    }
    else
    {
        decoded.altitude = altValue;
    }
  
    decoded.hdop = bytes[8] / 10.0;

    return decoded;
}
```

5. Build and upload on device

6. Turn on the device and once a GPS lock is acquired, the device will start sending data to Helium Mapper and Cargo.

## Data transfered

The device transmit 10 bytes every minute :

Latitude on 3 bytes

Longitude on 3 bytes

Altitude on 2 bytes

hdop on 1 byte

Satellites on 1 byte


Latitude value transmitted is ((lat() + 90) / 180.0) * 16777215
You must decode it with reverse operation :
```C
    val= ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];  
    val = (val / 16777215.0 * 180) - 90
```

Longitude value transmitted is ((lng() + 180) / 360.0) * 16777215
You must decode it with reverse operation :
```C
    val = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    val = (val / 16777215.0 * 360) - 180;
```

## Have a fun

:)