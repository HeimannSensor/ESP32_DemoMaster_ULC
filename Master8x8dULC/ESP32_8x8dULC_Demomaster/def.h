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

#define PIXEL_PER_COLUMN  8
#define PIXEL_PER_ROW     8

#define CLK               21
#define BIAS              5
#define BPA               12
#define MBIT              12
#define PU                0x88

#define CLOCK_EEPROM      400000
#define NR_OF_MAX_POLLS   250
#define ABSNULL           273.15

// WIFI SETTING
#define UDP_PACKET_LENGTH 262
#define ACCESSPOINTNAME "HTPAd8x8L2k1"
#define ACCESSPOINTKEY "heimannsensor"

#endif // for #ifndef  _def_H
