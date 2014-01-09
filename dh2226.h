//////////////////////////////////////////////////////////////////////////////////////
// File      : dh2226.h
// Product   : Sensoray 2226
// Function  : Include for both D[evice] and H[ost] software to keep things in sync
// Copyright : Copyright (C) Sensoray, 2009-2013
//////////////////////////////////////////////////////////////////////////////////////

#ifndef DH2226ED1_H
#define DH2226ED1_H

// Firmware version number.
// the last version using the FX2 chip is 0x5c.  We reserve 0x5d-0x6f for the
// unlikely case the fx2 based USB firmware is updated. 
// The ARM based USB firmware starts at 0x70 to 0x7f

// Additionally, the ARM based USB has a different "USB firmware" number.
// To further differentiate from the FX2.  This firmware number is returned
// in a vendor command as S2226_USB_FW_VER  
// USB FW 0x20-0x2f is RevA board with usb serial port
// USB FW 0x30+ is RevB board with USB serial port

#define S2226_ARM_FW_VERSION (0x00000225)//(big endian always)
#define S2226_USB_FW_VER 0x30

#ifndef __ASSEMBLY__
#include "types2226.h"

#define DEFAULT_DUTY_CYCLE 5500   // not 50%.  make slightly higher than field
                                  // Also the PWM has some internal input resistance.
#define MAX_DUTY_CYCLE     10000

// default location/sizes in flash
// H51 firmwares have been 698-701k long.  We do not anticipate any more H51 firmware releases.
// It is therefor save to put the serial number (no longer in Cypress FX2 EEPROM for Rev2.0)
// at location 0x140000
#define SERIAL_NUM_LOCATION_REVA  0x140000// (needs 2k of space for easy backward compatibility)
#define H51_FW_LOCATION_REVA      0x90000
#define H51_FW_SIZE_V1            717044  // 2008/11/27
#define H51_FW_SIZE_V2            714000  // 2009/05/25
#define FPGA_FW_LOCATION_REVA     0x10000

// 140000 to 14ffff reserved for future use (overlay)
#define SERIAL_NUM_LOCATION_REVB  0x138000 // (needs 8k (0x2000 hex) of space for easy backward compatibility)
#define H51_FW_LOCATION_REVB      0x150000 // 
#define FPGA_FW_LOCATION_REVB     0x10000 // (ends 132f6c)
#define FPGA_FW_SIZE_REVB         1191788 // 122f6c

#ifdef REVA
#define SERIAL_NUM_LOCATION   SERIAL_NUM_LOCATION_REVA
#define H51_FW_LOCATION       H51_FW_LOCATION_REVA
#define FPGA_FW_LOCATION      FPGA_FW_LOCATION_REVA
#define FPGA_FW_SIZE          FPGA_FW_SIZE_REVA
#undef FPGA_FW_SIZE_REVB
#else
#define SERIAL_NUM_LOCATION   SERIAL_NUM_LOCATION_REVB
#define H51_FW_LOCATION       H51_FW_LOCATION_REVB
#define FPGA_FW_LOCATION      FPGA_FW_LOCATION_REVB
#define FPGA_FW_SIZE          FPGA_FW_SIZE_REVB
#endif


//#define FPGA_FW_SIZE         510856

// attributes
#define ATTR_ARM_FW_VERSION    0  //[R] current ARM firmware version (sends out S2226_ARM_FW_VERSION)
// vsync no longer exported.  Not efficient or necessary.  handle in ARM firmware only.
//#define ATTR_VSYNC_EXPORTED    1  //[RW] If 1 vsync is sent out on the interrupt endpoint, if 0 vsync is handled internally (default 0)
// most messages from H51 are not sent out.  Fatal H51 errors will be sent out
// obsoleted #define ATTR_MESSAGES_EXPORTED 2  //[RW] If 1 asynchronous message are sent out on the interrupt endpoint. (default 0)
#define ATTR_CMD_WAITVSYNC     3  //[RW] If 1, wait for vsync(with timeuot) before issuing commands, if 0 just send command (default 1)
#define ATTR_LAST_MESSAGE      4  //[R] reading this attribute causes the ARM to output the last received asynchronous message.
#define ATTR_VSYNC_COUNT       5  //[R] current vsync IRQ counter
#define ATTR_TCPU_COUNT        6  //[R] current tcpu IRQ counter
#define ATTR_TACK_COUNT        7  //[R] current tcpu IRQ counter
#define ATTR_IRQ_COUNT         8  //[R] current IRQ IRQ counter
#define ATTR_MSG_COUNT         9  //[R] number of asynchronous messages received(not vsyncs)
#define ATTR_CMD_COUNT         10 //[R] number of commands sent
#define ATTR_INT_EP_PKTEND     11 //[RW] 0=no PACKET end allowed(default for now), 1=short packets allowed(needs debugged)
#define ATTR_INT_EP_SIZE       12 //[RW} default=512

#ifndef WIN32
#define ATTR_INPUT             13 //[RW] new input.
#else
#define S2226_ATTR_INPUT       13
#endif

#define ATTR_VERIFY_INPUT      14 //[RW] verify valid inputs before talking to H51 on FW load and encode start (default 1)
#define ATTR_FAST_NOTIFY       15 //[RW] fast respond to notification message (1 in interrupt during streaming(2256 style), 0 in commandTask)
#define ATTR_ERR_COUNT         16 //[R] total err interrupts from H51
#define ATTR_LAST_ERR          17 //[R] fetch last err onto interrupt endpoint (12 bytes long)
#define ATTR_ADVANCED          18 //[RW] [12] don't wait for vsync on encode start (keep default as 1-set)
#define ATTR_H51_OVERFLOW       19 //[R]
#define ATTR_INPUT_DELAY        20 //[RW] delay between input set and H51 load for ATTR_INPUT change
#define ATTR_FPGA_TS            21 //[RW] read: gets FPGA timestamp. (includes latching) write: resets FPGA timestamp
#define ATTR_ARM_TS             22 //[RW] read: gets ARM timestamp. write: resets timestamp
#define ATTR_ARM_FPGA_TS        23 //[RW] read: gets ARM & FPGA timestamp pair. write: resets both timestamps
// removed debug attributes below to save code size
//#define ATTR_IRQOUT_TS          21
//#define ATTR_IRQIN_TS           22
//#define ATTR_IRQMAXDIFF_TS      23
#define ATTR_SNAPCTRL         22 // snapshot control (locks raw pipe)
#define ATTR_SUPPLEMENTAL_FPGA 23 // Use supplemental FPGA image at 0x200000
#define ATTR_SCALE_SINGLE       24 // Note: reused from ATTR_TCPU_TS

#define ATTR_SCALE_RAW          25 // If set to 1,  MPEG Scaler uses the values in SCALE_X, SCALE_Y, SCALE_SINGLE
                                   // otherwise, MPEG Scaler is set to downscale the output for composite channels


#define ATTR_IRQERR_TS          26  // still useful
//#define ATTR_IRQ_PENDING        27 //[0]polled irq present for long period of time, [1]: H51 INT pin state
//#define ATTR_CPSR               28 // only if IRQ pending[0] set 
#define ATTR_SCALED_OUTPUT      30 // 0-unscaled, 1-scaled output

#define ATTR_CROP_TOP           33 // for scalar
#define ATTR_CROP_BOTTOM        34 // for scalar

// NEW with version 0x53
#define ATTR_CLOCKMOD_PRESENT   35 // default is to assume clock mod is present.  This setting allows the use of older
                                   // boards without the clock mod.

#define ATTR_MPEG_SCALER        36 // (reset MPEG scaler)
#define ATTR_BYPASS_MODE        37 // 1-enable bypass audio and video (note calling ATTR_INPUT will OVERRIDE this setting!)
#define ATTR_FPGA_RESET_N       38 // internal use only (write only)

#define ATTR_SCALE_X            39 // for preview or scalar (default 240)
#define ATTR_SCALE_Y            40 // for preview or scalar (default 160)
#define ATTR_CROP_LEFT          41 // for preview or scalar (default 0)
#define ATTR_CROP_RIGHT         42 // for preview or scalar (default 0)
#define ATTR_H51_RESET          43 // (write only) Experimental Internal use only.  Use with care.
                                   // This was added to possibly recover from the dread 0xE5 H51 crash
                                   // It has been found that sending a file recorded with 1080i 59.94Hz 
                                   // To the H51 with the H51 configured as 720P 50Hz WILL CRASH the H51
                                   // encoder.
#define ATTR_AUDH51_MASTER		44 // change the audio direction (use instead of manually setting AIC33 and FPGA
								   // registers).  1 if H51 master (default for legacy operation) 0 if H51 slave(
                                   // preferred value for latest firmware).


#define ATTR_DUTY              46 // [rw] returns current duty cycle, sets duty cycle

// Note: try not to increase maximum below to keep code size down.  Re-use some of the unused values
#define NUM_ATTRS            (ATTR_DUTY+1)

// CLK1 == 74.25MHz  (use for 60i or 50i)
// CLK2 == 74.1758MHz (use for 59.94i)
// for ATTR_INPUT
#define INPUT_COMP0_480I        0 
#define INPUT_COMP0_576I        1 
#define INPUT_SVIDEO0_480I      2 
#define INPUT_SVIDEO0_576I      3 
#define INPUT_SDI_480I          4 
#define INPUT_SDI_480I_CB       5 
#define INPUT_SDI_576I          6 
#define INPUT_SDI_576I_CB       7 
#define INPUT_SDI_1080I_50      8  //encode HD at 74.25 MHz (50i)  
#define INPUT_SDI_1080I_5994    9  //encode HD at 74.1758 MHz (59.94i)  
#define INPUT_SDI_1080I_60      10//encode HD at 74.25 MHz (60i)  
#define INPUT_SDI_1080I_60_CB   11
#define INPUT_SDI_720P_50       12
#define INPUT_SDI_720P_5994     13
#define INPUT_SDI_720P_60       14
#define INPUT_SDI_720P_60_CB    15
// more encoder inputs are below ( for backward compatibility)
#define INPUT_H51_SD_480I       16 //DECODE only, SD stream
#define INPUT_H51_HD_1080I_60   17 //DECODE only, HD stream
#define INPUT_H51_HD_720P_60    18 //DECODE only, HD stream
#define INPUT_H51_SD_576I       19 //DECODE only, SD stream
#define INPUT_H51_HD_1080I_50   20 //DECODE only, HD stream
#define INPUT_H51_HD_1080I_5994 21 //DECODE only, HD stream (uses 60_CB as ref input since 5994 colorbars not supported by SDI chip, or colorbars gen by FPGA Ver 23+)
#define INPUT_H51_HD_720P_5994  22 //DECODE only, HD stream (uses 60_CB as ref input since 5994 colorbars not supported by SDI chip, or colorbars gen by FPGA Ver 23+)
#define INPUT_SDI_1080I_50_CB   23 //encode(just for testing, don't add to stream server)
#define INPUT_COMP1_480I        24 //composite input 1(NTSC)
#define INPUT_COMP1_576I        25 //composite input 1(PAL)
#define INPUT_SVIDEO1_480I      26 //svideo input 1(NTSC)
#define INPUT_SVIDEO1_576I      27 //svideo input 1(PAL)
#define INPUT_H51_HD_720P_50    28 //DECODE only, HD stream (colorbars supported by FPGA Ver 23+)

// Colorbars
//#define INPUT_SDI_480I_CB      5 // defined above
//#define INPUT_SDI_576I_CB      7 // defined above
//#define INPUT_SDI_1080I_60_CB 11 // defined above
#define INPUT_SDI_1080I_5994_CB 29 // (colorbars only supported by FPGA Ver 23+)
//#define INPUT_SDI_1080I_50_CB 23 // defined above
//#define INPUT_SDI_720P_60_CB  15 // defined above
#define INPUT_SDI_720P_5994_CB  30 // (colorbars only supported by FPGA Ver 23+)
#define INPUT_SDI_720P_50_CB    31 // (colorbars only supported by FPGA Ver 23+)

#define INPUT_SDI_1080P_30_CB   32 // (overlay only. not supported for record/playback)
#define INPUT_SDI_1080P_2997_CB 33 // (overlay only. not supported for record/playback)
#define INPUT_SDI_1080P_24_CB   34
#define INPUT_SDI_1080P_2398_CB 35
#define INPUT_SDI_720P_24_CB    36
#define INPUT_SDI_720P_2398_CB  37
#define INPUT_SDI_1080P_30      38 // (overlay only. not supported for record/playback)
#define INPUT_SDI_1080P_2997    39 // (overlay only. not supported for record/playback)
#define INPUT_SDI_1080P_24      40
#define INPUT_SDI_1080P_2398    41
#define INPUT_SDI_720P_24       42
#define INPUT_SDI_720P_2398     43

#define INPUT_H51_HD_720P_24    44 //DECODE only
#define INPUT_H51_HD_720P_2398  45 //DECODE only
#define INPUT_H51_HD_1080P_24   46 //DECODE only
#define INPUT_H51_HD_1080P_2398 47 //DECODE only



#define IS_DECODE_INPUT(_x_) ((_x_ == INPUT_H51_HD_720P_60)  || (_x_ == INPUT_H51_HD_720P_5994)  || (_x_ == INPUT_H51_HD_720P_50 )\
	|| (_x_ == INPUT_H51_HD_1080I_60) || (_x_ == INPUT_H51_HD_1080I_5994) || (_x_ == INPUT_H51_HD_1080I_50)	\
				|| (_x_ == INPUT_H51_SD_480I) || (_x_ == INPUT_H51_SD_576I))


// progressive input
#define IS_PROG_INPUT(_x_) ( (_x_ == INPUT_H51_HD_720P_60)  || (_x_ == INPUT_H51_HD_720P_5994)  || (_x_ == INPUT_H51_HD_720P_50 )\
                        || (_x_ == INPUT_SDI_720P_60)     || (_x_ == INPUT_SDI_720P_5994)     || (_x_ == INPUT_SDI_720P_50    )\
				|| (_x_ == INPUT_SDI_720P_60_CB)  || (_x_ == INPUT_SDI_720P_5994_CB)  || (_x_ == INPUT_SDI_720P_50_CB ))




#define IS_HD_INPUT(_x_) ( (_x_ == INPUT_H51_HD_720P_60)  || (_x_ == INPUT_H51_HD_720P_5994)  || (_x_ == INPUT_H51_HD_720P_50 )\
                        || (_x_ == INPUT_H51_HD_1080I_60) || (_x_ == INPUT_H51_HD_1080I_5994) || (_x_ == INPUT_H51_HD_1080I_50)\
                        || (_x_ == INPUT_H51_HD_1080P_24) || (_x_ == INPUT_H51_HD_1080P_2398) || (_x_ == INPUT_H51_HD_720P_24)\
                        || (_x_ == INPUT_H51_HD_1080P_2398) \
                        || (_x_ == INPUT_SDI_1080I_60)    || (_x_ == INPUT_SDI_1080I_5994)    || (_x_ == INPUT_SDI_1080I_50   )\
                        || (_x_ == INPUT_SDI_720P_60)     || (_x_ == INPUT_SDI_720P_5994)     || (_x_ == INPUT_SDI_720P_50    )\
                        || (_x_ == INPUT_SDI_720P_60_CB)  || (_x_ == INPUT_SDI_720P_5994_CB)  || (_x_ == INPUT_SDI_720P_50_CB )\
                        || (_x_ == INPUT_SDI_1080I_60_CB) || (_x_ == INPUT_SDI_1080I_5994_CB) || (_x_ == INPUT_SDI_1080I_50_CB)\
                        || (_x_ == INPUT_SDI_1080P_30_CB   ) || (_x_ == INPUT_SDI_1080P_2997_CB )\
                        || (_x_ == INPUT_SDI_1080P_24_CB   ) || (_x_ == INPUT_SDI_1080P_2398_CB )\
                        || (_x_ == INPUT_SDI_720P_24_CB    ) || (_x_ == INPUT_SDI_720P_2398_CB  )\
                        || (_x_ == INPUT_SDI_1080P_30      ) || (_x_ == INPUT_SDI_1080P_2997    )\
                        || (_x_ == INPUT_SDI_1080P_24      ) || (_x_ == INPUT_SDI_1080P_2398    )\
                        || (_x_ == INPUT_SDI_720P_24       ) || (_x_ == INPUT_SDI_720P_2398     ))

inline int is_hd_input(int input)
{
  if (IS_HD_INPUT(input))
    return 1;
  return 0;
}


#define IS_PAL_INPUT(_x_) ((_x_ == INPUT_H51_HD_720P_50)  || (_x_ == INPUT_H51_HD_1080I_50)  || (_x_ == INPUT_SDI_1080I_50 )\
                        || (_x_ == INPUT_SDI_720P_50) || (_x_ == INPUT_SDI_1080I_50_CB) || (_x_ == INPUT_SDI_720P_50_CB) \
                        || (_x_ == INPUT_COMP0_576I)    || (_x_ == INPUT_SVIDEO0_576I)    || (_x_ == INPUT_SVIDEO1_576I   )\
                        || (_x_ == INPUT_COMP1_576I)     || (_x_ == INPUT_SDI_576I)     || (_x_ == INPUT_SDI_576I_CB    ) \
                        || (_x_ == INPUT_H51_SD_576I))  

// For H51 only
// VFMT values match those of the V_FORMAT field in IPRV
// Note: differences between 59.94 and 60, and between 23.98 and 24 are controlled by the pixel clock setting
#define VFMT_1080_60i       0       // 1920x1080 (59.94i/60i) SMPTE274M supported (works)
#define VFMT_1080_50i       1       // 1920x1080 (50i) SMPTE274M supported (TBD)
#define VFMT_720_60p        2       // 1280x720 (59.94p/60p) SMPTE296M-2001 supported (works)
#define VFMT_720_50p        3       // 1280x720 (50p) SMPTE296M-2001 supported  (TBD)
#define VFMT_480_60i        4       // 720x480 (59.94i) ITU-R BT.656-4 supported (works)
#define VFMT_576_50i        5       // 720x576 (50i) ITU-R BT.656-4 supported (TBD)
#define VFMT_1080_24p       34      // 1920x1080 (23.98p/24p) SMPTE274M supported (TBD)
#define VFMT_720_24p        42      // 1280x720 (23.98p/24p) SMPTE296M-2001 supported

// AMODE values match those of the A_MODE field in APRA
#define AMODE_MP1L2         0       // MPEG-1 layer 2, default
#define AMODE_AC3           1       // AC3
#define AMODE_LPCM          2       // LPCM
#define AMODE_AAC           3       // AAC

// GOP
// according to .xls file, 1080 is always IBP, all other resolutions are always IBBP
// #define GOP_IBBP            0       //IBBP, supported for all resolutions
// #define GOP_IBP             3       //IBP, supported only for 1920x1088

// Device identifiers.
typedef enum DEVICE_ID {		// Firmware-based devices:
	DEVID_ARM,					//   CPU.
	DEVID_H51,					//   Compression codec.
	DEVID_FPGA,					//   Gate array.
	DEVID_SDI_IN,				//   CLC031.
	DEVID_SDI_OUT,				//   CLC030.
    DEVID_AUDIO,                // audio codec through I2C TLV320AIC33
    DEVID_SDI_SPLIT,            // SDI splitter chip LMH307
    DEVID_VIDDEC,              // composite video decoder saa7115
    DEVID_VIDENC,              // composite video encoder saa7120
	DEVID_INVALID				// always last in list
} DEVICE_ID;

typedef u8		DEVID;			// Device identifier.

// USB host commands to 2226
enum {
	HOSTCMD_NOP = 0,			// No operation.
	HOSTCMD_PERSONA_RUN,		// Switch to specified new personality. FX2 endpoints may now be used.
	HOSTCMD_PERSONA_HALT,		// Prepare to switch to new personality. Issued when an alternate USB endpoint configuration is activated. Don't use fifos until HOSTCMD_PERSONA_RUN.
	HOSTCMD_FLASH_ERASE,		// Erase flash sector.
	HOSTCMD_FLASH_WRITE,		// Write contiguous data starting at specified flash address. Flash must have been previously erased.
	HOSTCMD_FLASH_READ,			// Read contiguous data starting at specified flash address.
	HOSTCMD_FLASH_READINFO,		// Return flash info per JEDEC standard for "Manufacturer and Device ID Read Methodology for SPI Compatible Serial Interface Memory Devices"
	HOSTCMD_BOOT,				// Copy firmware from flash to specified device, activate firmware, or both, as specified by flags.
	HOSTCMD_ATTR_WRITE,			// Set/get device dependent attr's: bitrate, brightness, studio/field unit, encode/decode per stream, etc.
	HOSTCMD_ATTR_READ,
	HOSTCMD_SET_MODE,			// Switch to specified device-independent mode: START/STOP/
	HOSTCMD_REGWRITE,			// Write to a contiguous address range on a specified device (i.e, memory mapped, i2c device, etc.)
	HOSTCMD_REGREAD,			// Read from a contiguous chunk of registers.
	HOSTCMD_IOCTL,				// Device specific operation.
	HOSTCMD_SET_SUBMODE,        // Extra mode control when in the decode state for example
	HOSTCMD_REGREAD_OOB,		// Read from a contiguous chunk of registers on alternate endpoint    
	HOSTCMD_REGWRITE_DMA,   	// Write to a contiguous address range on a specified device (i.e, memory mapped, i2c device, etc.)
                                // sets up the data transfer.  Send the data immediately after this command
};

#ifndef WIN32

// USB host command structures -------------------

// All command buffers are "packed"  make sure buffer is aligned 4
#pragma pack(1)
typedef struct CMD_PERSONA_RUN {	// Activate new personality:
	u8		personality;			//  Enumerated personality. Zero is default personality at boot (diagnostic/firmware_upload persona).
	u8		optionFlags;			//  Option flags, if needed.
} CMD_PERSONA_RUN;

typedef struct CMD_SET_MODE {
	u8		streamId;				// Device-dependent stream id. This may reference any independently controllable stream on the board.
	u8		streamMode;				// STREAM_IDLE | STREAM_ENCODE_READY | STREAM_ENCODE_ACTIVE | STREAM_DECODE_READY | STREAM_DECODE_ACTIVE.
} CMD_SET_MODE;

typedef struct CMD_FLASH_ERASE {
	u32		addr;					// Base address of flash sector to be erased.
    u32     size;                   // if 0, erases only flash sector at base address
                                    // otherwise, erases a range of addresses
} CMD_FLASH_ERASE;

typedef struct CMD_FLASH_WRITE {
	u32		addr;					// Starting flash address to write to.
	u8		data[];					// Data to be written to flash. Must have an even byte count.
} CMD_FLASH_WRITE;

typedef struct CMD_FLASH_READ {
	u32		addr;					// Starting flash address to read from.
	u8		nBytes;					// Number of bytes to read from flash. Must have an even byte count.
} CMD_FLASH_READ;

typedef struct CMD_BOOT {
	u32		addr;					// Base address of firmware image in flash.
	u32		imageSize;				// Size of firmware image in bytes.
	DEVID	deviceId;				// Identifier for device that is to programmed with the firmware image.
	u8		padding;				// Force this command to have an even byte count.
} CMD_BOOT;


typedef struct CMD_SET_SUBMODE {
	u8 subMode; // sub command.
	u32 param1; // vdisp time if slow decode requested
	u32 reserved; // reserved
} CMD_SET_SUBMODE;


// example: (HOSTCMD_BOOT = 7)
// to boot h51,  addr = 00 09 00 00
//               imageSize = 00 0a f0 f4
//               deviceId = DEVID_H51 = 1
//               padding = 0
// 
// to boot FPGA, addr = 00 01 00 00
//               imageSize = 00 07 cb 88
//               deviceId = DEVID_FPGA = 2
//               padding = 0

#define REG_ADDRBUMP_ENABLE		0x80	// BumpEnab bitmask in mode member of CMD_REGREAD and CMD_REGWRITE.
#define REG_UNPACK_ENABLE		0x40	// data sent to ARM is packed. unpack from 8 bits to 16 bits
                                        // Used for faster overlays to the FPGA (not available with REG_ADDRBUMP_ENABLE)

#define REG_NORESP_ENABLE       0x20    // do not want a response.  for burst commands where a response for every packet is not wanted
#define REG_ADDRDATA_MODE       0x10    // data is in addr,data format.


typedef struct CMD_REGWRITE {
	DEVID	deviceId;				// Identifier for device that contains registers that are to be written to.
	u8		mode;					// (BumpEnab,0,0,0,0,ValSize/3) Register address bump enable, Size of data values (must be 1, 2 or 4).
	u16		nValues;				// Number of values to be written to registers.
	u32		regAddr;				// Base address of first register in contiguous register address range (assumes first device addr == 0).
	u8		values[];				// Data values to be written to registers. Must have even byte count and integral value count.
} CMD_REGWRITE;


typedef struct CMD_REGWRITE_DMA {
	DEVID	deviceId;				// Identifier for device that contains registers that are to be written to.
	u8		mode;					// (BumpEnab,0,0,0,0,ValSize/3) Register address bump enable, Size of data values (must be 1, 2 or 4).
    u16     reserved;
	u32		nValues;				// Number of values to be written to registers.
	u32		regAddr;				// Base address of first register in contiguous register address range (assumes first device addr == 0).
} CMD_REGWRITE_DMA;


typedef struct CMD_REGREAD {
	DEVID	deviceId;				// Identifier for device that contains registers that are to be read from.
	u8		mode;					// (BumpEnab,0,0,0,0,ValSize/3) Register address bump enable, Size of data values (must be 1, 2 or 4).
        // NOTE: This looks confusing, but it forces the user to
        // account for endianness.  Also, there are be alignment/packing issues with IAR and
        // the pragmas do not seem to be working.
	u16		nValues;				// Number of values to be read from registers.
	u32		regAddr;				// Base address of first register in contiguous register address range (assumes first device addr == 0).
} CMD_REGREAD;

typedef struct CMD_ATTRWRITE {
	u32	    attr;	                // attribute
	u32		val;					// value
} CMD_ATTRWRITE;


typedef struct CMD_ATTRREAD {
	u32	    attr;	                // attribute
} CMD_ATTRREAD;


typedef struct CMD_PERSONA_HALT {	// halts stream by stream index(different than an endpoint change halt)
    u8      index;                  // index = 0 == streamtask(h264),  index=1 == raw preview task
    u8		optionFlags;			//  Option flags, if needed.
} CMD_PERSONA_HALT;


// extended register read.  out of band (response is on raw preview endpoint).
typedef struct CMD_REGREAD_OOB {
	DEVID	deviceId;				// Identifier for device that contains registers that are to be read from.
	u8		mode;					// (BumpEnab,0,0,0,0,ValSize/3) Register address bump enable, Size of data values (must be 1, 2 or 4).
        // NOTE: This looks confusing, but it forces the user to
        // account for endianness.  Also, there are be alignment/packing issues with IAR and
        // the pragmas do not seem to be working.
	u32		nValues;				// Number of values to be read from registers.
	u32		regAddr;				// Base address of first register in contiguous register address range (assumes first device addr == 0).
} CMD_REGREAD_OOB;


// turn off the packing command
#pragma pack()


/*

// Command header struct
typedef struct {
    __int32         token;          // IN_DATA_TOKEN
    __int32         code;           // command code
    __int32         param;          // command parameter
} CMD_HDR;

#define	IN_DATA_TOKEN	0x2226c0de	// command token (host to 2256ED1)
#define ACK_TOKEN       0x2226acac	// acknowledge token (2256ED1 to host)

// Parameter bitmask
#define PRM_CHG_MODE        1       // register reload required
#define PRM_REG_SETTINGS    2       // copy register settings from packet
*/


// USB packet size
#define USB_PACKET_SIZE     512
//#define CMD_PACKET_SIZE     USB_PACKET_SIZE     // command packet size; must be <= USB_PACKET_SIZE


#define MAX_H51_FRM_SIZE    (1024 * 1024)   //1024 KB


#endif // WIN32

#endif	// #ifndef __ASSEMBLY__


#endif //DH2256ED1_H
