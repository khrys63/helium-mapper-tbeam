#include <stdint.h>

// -----------------------------------------------------------------------------
// BOARD
// -----------------------------------------------------------------------------

#define T_BEAM_V07  // AKA Rev0 (first board released)
//#define T_BEAM_V10  // AKA Rev1 (second board released)

// -----------------------------------------------------------------------------
// GPS
// -----------------------------------------------------------------------------

#define GPS_SERIAL_NUM  1
#define GPS_BAUDRATE    9600

#if defined(T_BEAM_V07)
#define GPS_RX_PIN      12
#define GPS_TX_PIN      15
#elif defined(T_BEAM_V10)
#define GPS_RX_PIN      34
#define GPS_TX_PIN      12
#endif

// -----------------------------------------------------------------------------
// OLED
// -----------------------------------------------------------------------------

#define SSD1306_ADDRESS 0x3C
#define SCREEN_HEADER_HEIGHT    14
#define I2C_SDA         21
#define I2C_SCL         22
#define LOGO_DELAY      5000 
