#ifndef _def_H
#define _def_H

// DEVICE ADRESS
#define MODULE_ADDRESS    0x1C

// MODULE COMMANDS
#define CMD_SLEEP         0x01
#define CMD_WAKEUP        0x02
#define CMD_READMASK      0x03
#define CMD_WRITEMASK     0x04
#define CMD_READEOC       0x05
#define CMD_READDATA      0x06
#define CMD_READEMI       0x07
#define CMD_WRITEEMI      0x08
#define CMD_READID        0x09
#define CMD_DEEPSLEEP     0x0A

// #define HTPA_8x8d
#define HTPA_32x32d

#ifdef HTPA_8x8d
#define PIXEL_PER_COLUMN  8
#define PIXEL_PER_ROW     8
#define NR_OF_BLOCKS      1
#endif

#ifdef HTPA_16x16d
#define PIXEL_PER_COLUMN  16
#define PIXEL_PER_ROW     16
#define NR_OF_BLOCKS      2
#endif

#ifdef HTPA_32x32d          // CK added for 32x32
#define PIXEL_PER_COLUMN  32
#define PIXEL_PER_ROW     32
#define NR_OF_BLOCKS      4
#endif


#define CLK               21
#define BIAS              5
#define BPA               12
#define MBIT              12
#define PU                0x88

#define CLOCK_EEPROM      1000000
#define SERIAL_BAUDRATE   115200
#define NR_OF_MAX_POLLS   150//250
#define ABSNULL           273.15

// WIFI SETTING
#ifdef HTPA_8x8d
  #define UDP_PACKET_LENGTH 262
  #define LAST_UDP_PACKET_LENGTH 262
  #define ACCESSPOINTNAME "HTPA8x8dULC_Demomaster"
#elif defined HTPA_16x16d
  #define UDP_PACKET_LENGTH 780
  #define LAST_UDP_PACKET_LENGTH 780
  #define ACCESSPOINTNAME "HTPA16x16dULC_Demomaster"
#elif defined HTPA_32x32d // CK added for 32x32
  #define UDP_PACKET_LENGTH 1292
  #define LAST_UDP_PACKET_LENGTH 1288
  #define ACCESSPOINTNAME "HTPA32x32dULC_Demomaster"
#else
  #error "Undefinied arraytype"
#endif
#define ACCESSPOINTKEY "heimannsensor"

#endif // for #ifndef  _def_H
