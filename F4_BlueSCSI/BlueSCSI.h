#ifndef __BLUESCSI_H__
#define __BLUESCSI_H__

#include <Arduino.h> // For Platform.IO
#include <SdFat.h>

// SCSI config
#define MAX_SCSIID  7          // Maximum number of supported SCSI-IDs (The minimum is 0)
#define MAX_SCSILUN 8          // Maximum number of LUNs supported     (The minimum is 0)
#define NUM_SCSIID  MAX_SCSIID // Number of enabled SCSI IDs
#define NUM_SCSILUN 1          // Number of enabled LUNs
#define READ_PARITY_CHECK 0    // Perform read parity check (unverified)
#define DEFAULT_SCSI_ID 1
#define DEFAULT_SCSI_LUN 0
#define SCSI_BUF_SIZE 512      // Size of the SCSI Buffer
#define HDD_BLOCK_SIZE 512
#define OPTICAL_BLOCK_SIZE 2048

// HDD format
#define MAX_BLOCKSIZE 4096     // Maximum BLOCK size

// LED ERRORS
#define ERROR_FALSE_INIT  3
#define ERROR_NO_SDCARD   5

enum SCSI_DEVICE_TYPE
{
  SCSI_DEVICE_HDD,
  SCSI_DEVICE_OPTICAL,
};

#define CDROM_RAW_SECTORSIZE    2352
#define CDROM_COMMON_SECTORSIZE 2048

#define MAX_SCSI_COMMAND  0xff
#define SCSI_COMMAND_HANDLER(x) static byte x(SCSI_DEVICE *dev, const byte *cdb)

#if DEBUG == 1
#define serial Serial2 // PA2 TX, PA3 RX
#define LOG(XX)      serial.print(XX)
#define LOGHEX(XX)   serial.print(XX, HEX)
#define LOGDEC(XX)   serial.print(XX, DEC)
#define LOGBIN(XX)   serial.print(XX, BIN)
#define LOGN(XX)     serial.println(XX)
#define LOGHEXN(XX)  serial.println(XX, HEX)
#define LOGDECN(XX)  serial.println(XX, DEC)
#define LOGBIN_N(XX) serial.println(XX, BIN)
#else
#define LOG(XX)       //serial.print(XX)
#define LOGHEX(XX)    //serial.print(XX, HEX)
#define LOGDEC(XX)    //serial.print(XX, DEC)
#define LOGBIN(XX)    //serial.print(XX, BIN)
#define LOGN(XX)      //serial.println(XX)
#define LOGHEXN(XX)   //serial.println(XX, HEX)
#define LOGDECN(XX)   //serial.println(XX, DEC)
#define LOGBIN_N(XX)  //serial.println(XX, BIN)
#endif

#define active   1
#define inactive 0
#define high 0
#define low 1

#define isHigh(XX) ((XX) == high)
#define isLow(XX) ((XX) != high)

//#define DB0       PB8     // SCSI:DB0
//#define DB1       PB9     // SCSI:DB1
//#define DB2       PB10    // SCSI:DB2
//#define DB3       PB2     // SCSI:DB3
//#define DB4       PB12    // SCSI:DB4
//#define DB5       PB13    // SCSI:DB5
//#define DB6       PB14    // SCSI:DB6
//#define DB7       PB15    // SCSI:DB7
//#define DBP       PB0     // SCSI:DBP

#define ATN       PA8      // SCSI:ATN
#define BSY       PA9      // SCSI:BSY
#define ACK       PA10     // SCSI:ACK
#define RST       PA15     // SCSI:RST
#define MSG       PB3      // SCSI:MSG
#define SEL       PB4      // SCSI:SEL
#define CD        PB5      // SCSI:C/D
#define REQ       PB6      // SCSI:REQ
#define IO        PB7      // SCSI:I/O

#define LED       PC13     // LED
#define LED2      PB1      // Driven LED

// Image Set Selector
#define IMAGE_SELECT1   PC14
#define IMAGE_SELECT2   PC15

// GPIO register port
#define PAREG GPIOA->regs
#define PBREG GPIOB->regs
#define PCREG GPIOC->regs

// LED control
#define LED_ON()       PCREG->BSRR = 0x20000000; PBREG->BSRR = 0x00000002;
#define LED_OFF()      PCREG->BSRR = 0x00002000; PBREG->BSRR = 0x00020000;

// Virtual pin (Arduio compatibility is slow, so make it MCU-dependent)
#define PA(BIT)       (BIT)
#define PB(BIT)       (BIT+16)

// Virtual pin decoding
#define GPIOREG(VPIN)     ((VPIN) >= 16 ? PBREG : PAREG)
#define BITMASK(VPIN)     (1 << ((VPIN) & 15))

#define vATN       PA(8)      // SCSI:ATN
#define vBSY       PA(9)      // SCSI:BSY
#define vACK       PA(10)     // SCSI:ACK
#define vRST       PA(15)     // SCSI:RST
#define vMSG       PB(3)      // SCSI:MSG
#define vSEL       PB(4)      // SCSI:SEL
#define vCD        PB(5)      // SCSI:C/D
#define vREQ       PB(6)      // SCSI:REQ
#define vIO        PB(7)      // SCSI:I/O
#define vSD_CS     PA(4)      // SDCARD:CS

// SCSI output pin control: opendrain active LOW (direct pin drive)
// inactive = physical high, logical 0
// active = physical low, logical 1
#define SCSI_OUT(VPIN,ACTIVE) { GPIOREG(VPIN)->BSRR = BITMASK(VPIN) << ((ACTIVE) ? 16 : 0); }

// SCSI input pin check (inactive=0,active=1)
#define SCSI_IN(VPIN) ((~GPIOREG(VPIN)->IDR >> (VPIN & 15)) & 1)

#define NOP(x) for(unsigned _nopcount = x; _nopcount; _nopcount--) { asm("NOP"); }

/* SCSI Timing delays */
// Due to limitations in timing granularity all of these are "very" rough estimates
#define SCSI_BUS_SETTLE() NOP(30);                            // spec 400ns ours ~420us
#define SCSI_DATA_RELEASE() NOP(30);                          // spec 400ns ours ~420us
#define SCSI_HOLD_TIME() asm("NOP"); asm("NOP"); asm("NOP");  // spec 45ns ours ~42ns
#define SCSI_DESKEW() // asm("NOP"); asm("NOP"); asm("NOP");     // spec 45ns ours ~42ns
#define SCSI_CABLE_SKEW() // asm("NOP");                         // spec 10ns ours ~14ns
#define SCSI_RESET_HOLD() asm("NOP"); asm("NOP");             // spec 25ns ours ~28ns
#define SCSI_DISCONNECTION_DELAY() NOP(15);                   // spec 200ns ours ~210ns

/* SCSI phases
+=============-===============-==================================-============+
|    Signal   |  Phase name   |       Direction of transfer      |  Comment   |
|-------------|               |                                  |            |
| MSG|C/D|I/O |               |                                  |            |
|----+---+----+---------------+----------------------------------+------------|
|  0 | 0 | 0  |  DATA OUT     |       Initiator to target     \  |  Data      |
|  0 | 0 | 1  |  DATA IN      |       Initiator from target   /  |  phase     |
|  0 | 1 | 0  |  COMMAND      |       Initiator to target        |            |
|  0 | 1 | 1  |  STATUS       |       Initiator from target      |            |
|  1 | 0 | 0  |  *            |                                  |            |
|  1 | 0 | 1  |  *            |                                  |            |
|  1 | 1 | 0  |  MESSAGE OUT  |       Initiator to target     \  |  Message   |
|  1 | 1 | 1  |  MESSAGE IN   |       Initiator from target   /  |  phase     |
|-----------------------------------------------------------------------------|
| Key:  0 = False,  1 = True,  * = Reserved for future standardization        |
+=============================================================================+ 
*/
// SCSI phase change as single write to port B
#define SCSIPHASEMASK(MSGACTIVE, CDACTIVE, IOACTIVE) ((BITMASK(vMSG)<<((MSGACTIVE)?16:0)) | (BITMASK(vCD)<<((CDACTIVE)?16:0)) | (BITMASK(vIO)<<((IOACTIVE)?16:0)))

#define SCSI_PHASE_DATAOUT SCSIPHASEMASK(inactive, inactive, inactive)
#define SCSI_PHASE_DATAIN SCSIPHASEMASK(inactive, inactive, active)
#define SCSI_PHASE_COMMAND SCSIPHASEMASK(inactive, active, inactive)
#define SCSI_PHASE_STATUS SCSIPHASEMASK(inactive, active, active)
#define SCSI_PHASE_MESSAGEOUT SCSIPHASEMASK(active, active, inactive)
#define SCSI_PHASE_MESSAGEIN SCSIPHASEMASK(active, active, active)

#define SCSI_PHASE_CHANGE(MASK) { PBREG->BSRR = (MASK); }

static const uint32_t scsiDbOutputRegOr = 0x55150011;
static const uint32_t scsiDbInputOutputAnd = 0x00C0FFCC;
// Put DB and DP in output mode
#define SCSI_DB_OUTPUT() { PBREG->MODER = (PBREG->MODER & scsiDbInputOutputAnd) | scsiDbOutputRegOr; }

// Put DB and DP in input mode
#define SCSI_DB_INPUT()  { PBREG->MODER = (PBREG->MODER & scsiDbInputOutputAnd); }

#if XCVR == 1

#define TR_TARGET        PA0   // Target Transceiver Control Pin
#define TR_DBP           PA1   // Data Pins Transceiver Control Pin
#define TR_INITIATOR     PA2   // Initiator Transciever Control Pin

#define vTR_TARGET       PA(0) // Target Transceiver Control Pin
#define vTR_DBP          PA(1) // Data Pins Transceiver Control Pin
#define vTR_INITIATOR    PA(2) // Initiator Transciever Control Pin

#define TR_INPUT 1
#define TR_OUTPUT 0

// Transceiver control definitions
#define TRANSCEIVER_IO_SET(VPIN,TR_INPUT) { GPIOREG(VPIN)->BSRR = BITMASK(VPIN) << ((TR_INPUT) ? 16 : 0); }

static const uint32_t SCSI_TARGET_PORTA_AND = 0xFFF3FFFF;  // Sets input mode when AND-ed against MODER
static const uint32_t SCSI_TARGET_PORTA_OR = 0x00040000;  // Sets output mode when AND+OR against MODER
static const uint32_t SCSI_TARGET_PORTB_AND = 0xFFFF033F;  // Sets input mode when AND-ed against MODER
static const uint32_t SCSI_TARGET_PORTB_OR = 0x00005440;  // Sets output mode when AND+OR against MODER

// Turn on the output only for BSY
#define SCSI_BSY_ACTIVE()      { PAREG->MODER = (PAREG->MODER & SCSI_TARGET_PORTA_AND) | SCSI_TARGET_PORTA_OR; PBREG->MODER = (PBREG->MODER & SCSI_TARGET_PORTB_AND) | SCSI_TARGET_PORTB_OR; SCSI_OUT(vBSY, active) }

// BSY,REQ,MSG,CD,IO Turn off output, BSY is the last input
#define SCSI_TARGET_INACTIVE() { PAREG->MODER = (PAREG->MODER & SCSI_TARGET_PORTA_AND); PBREG->MODER = (PBREG->MODER & SCSI_TARGET_PORTB_AND); TRANSCEIVER_IO_SET(vTR_TARGET,TR_INPUT); }

#define SCSI_TARGET_ACTIVE()   { PAREG->MODER = (PAREG->MODER & SCSI_TARGET_PORTA_AND) | SCSI_TARGET_PORTA_OR; PBREG->MODER = (PBREG->MODER & SCSI_TARGET_PORTB_AND) | SCSI_TARGET_PORTB_OR; }

#else

// Turn on the output only for BSY
#define SCSI_BSY_ACTIVE()      { pinMode(BSY, OUTPUT_OPEN_DRAIN); SCSI_OUT(vBSY,  active) }

// BSY,REQ,MSG,CD,IO Turn off output, BSY is the last input
#define SCSI_TARGET_INACTIVE() { SCSI_OUT(vREQ,inactive); SCSI_PHASE_CHANGE(SCSI_PHASE_DATAOUT); SCSI_OUT(vBSY,inactive); pinMode(BSY, INPUT); }

// BSY,REQ,MSG,CD,IO Turn on the output (no change required for OD)
#define SCSI_TARGET_ACTIVE()   { }

#endif

// HDDimage file
#define HDIMG_ID_POS  2                 // Position to embed ID number
#define HDIMG_LUN_POS 3                 // Position to embed LUN numbers
#define HDIMG_BLK_POS 5                 // Position to embed block size numbers
#define MAX_FILE_PATH 32                // Maximum file name length

/*
 *  Data byte to BSRR register setting value and parity table
*/
/**
 * BSRR register generator
 * Totally configurable for which pin is each data bit, which pin is PTY, and which pin is REQ.
 * The only requirement is that data and parity pins are in the same GPIO block.
 * REQ can be specified as -1 to ignore, as it doesn't have to be in the same GPIO block.
 * This is dramatically slower than the original static array, but is easier to configure
 */
static uint32_t genBSRR(uint32_t data) {
  uint8_t masks[] = {0UL, 1UL, 2UL, 3UL, 4UL, 5UL, 6UL, 7UL};
  // Positions array indicates which bit position each data bit goes in
  // positions[0] is for data bit 0, position[1] for data bit 1, etc
  // DB0, DB1, DB2, DB4, DB5, DB6, DB7 in order
  uint8_t positions[] = {8UL, 9UL, 10UL, 2UL, 12UL, 13UL, 14UL, 15UL};
  uint8_t dbpPosition = 0UL;
  int reqPosition = 6;
  uint8_t bitsAsserted = 0;

  uint32_t output = 0x00000000;
  for (int i = 0; i < 8; i++) {
    if (data & (0x1 << masks[i])) {
      // There's a one in this bit position, BSRR reset
      output |= 0x1 << (positions[i] + 16);
      bitsAsserted++;
    } else {
      // There's a 0 in this bit position, BSRR set high
      output |= (0x1 << positions[i]);
    }
  }

  // Set the parity bit
  if (bitsAsserted % 2 == 0) {
    // Even number of bits asserted, Parity asserted (0, low, BSRR reset)
    output |= 0x01 << (dbpPosition + 16);
  } else {
    // Odd number of bits asserted, Parity deasserted (1, high, BSRR set)
    output |= 0x01 << dbpPosition;
  }

  // BSRR set REQ if specified
  // Only set > 0 if it's in the same GPIO block as DB and DBP
  if (reqPosition >= 0) {
    output |= 0x01 << reqPosition;
  }

  return output;
}

// BSRR register control value that simultaneously performs DB set, DP set, and REQ = H (inactrive)
uint32_t db_bsrr[256];
// Parity bit acquisition
#define PARITY(DB) (db_bsrr[DB]&1)

// #define GET_CDB6_LBA(x) ((x[2] & 01f) << 16) | (x[3] << 8) | x[4]
#define READ_DATA_BUS() (byte)((~(uint32_t)GPIOB->regs->IDR)>>8)



struct SCSI_INQUIRY_DATA
{
  union
  {
  struct {
    // bitfields are in REVERSE order for ARM
    // byte 0
    byte peripheral_device_type:5;
    byte peripheral_qualifier:3;
    // byte 1
    byte reserved_byte2:7;
    byte rmb:1;
    // byte 2
    byte ansi_version:3;
    byte always_zero_byte3:5;
    // byte 3
    byte response_format:4;
    byte reserved_byte4:2;
    byte tiop:1;
    byte always_zero_byte4:1;
    // byte 4
    byte additional_length;
    // byte 5-6
    byte reserved_byte5;
    byte reserved_byte6;
    // byte 7
    byte sync:1;
    byte always_zero_byte7_more:4;
    byte always_zero_byte7:3;
    // byte 8-15
    char vendor[8];
    // byte 16-31
    char product[16];
    // byte 32-35
    char revision[4];
    // byte 36
    byte release;
    // 37-46
    char revision_date[10];
  };
  // raw bytes
  byte raw[64];
  };
};

// HDD image
typedef __attribute__((aligned(4))) struct _SCSI_DEVICE
{
	FsFile        *m_file;                 // File object
	uint64_t      m_fileSize;             // File size
	uint16_t      m_blocksize;            // SCSI BLOCK size
  uint16_t      m_rawblocksize;
  uint8_t       m_type;                 // SCSI device type
  uint32_t      m_blockcount;           // blockcount
  bool          m_raw;                  // Raw disk
  SCSI_INQUIRY_DATA *inquiry_block;      // SCSI information
  uint8_t       m_senseKey;               // Sense key
  uint16_t      m_additional_sense_code;  // ASC/ASCQ 
  bool          m_mode2;                  // MODE2 CDROM
  uint8_t       m_offset;                 // ISO offset for missing sync header
} SCSI_DEVICE;


#endif
