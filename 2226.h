#ifndef S2226_H
#define S2226_H

#define MODE_PENDING	1 //mode change pending 
#define CMD_USB_EN	1 //enable data transfer to host
#define CMD_USB_DIS	0 //disable -"-
#define	IN_DATA_TOKEN	0x2226c0de //command token (host to 2226)

#define CMD_2226      0xc2226000 //command base
#define CMD_SET_MODE (CMD_2226 | 0x10) //set mode
#define CMD_START    (CMD_2226 | 0x20) //start capture
#define CMD_STOP     (CMD_2226 | 0x30) //stop capture
#define CMD_STATUS   (CMD_2226 | 0x40) //return status

#define	OUT_DATA_TOKEN 0x2226da4a //data token (2255 to host)

#define ACK_DATA_TOKEN	  0x2226acac //ack token (2255 to host)
#define ACK_SET_MODE_DONE 0x1 //setmode complete
#define ACK_TSK_RUNNING   0x10 //task running
#define ACK_STATUS        0x20 //status data follows

#define CHANMASK_AUDIODEC0    (1 << 0)
#define CHANMASK_AUDIODEC1    (1 << 1)
#define CHANMASK_AUDIOENC0    (1 << 2)
#define CHANMASK_AUDIOENC1    (1 << 3)
#define CHANMASK_VIDENC       (1 << 4)
#define CHANMASK_VIDDEC       (1 << 5)
#define CHANMASK_MAX          (1 << 6)


// Copied from frm-2226-arm/Include/dh2226.h

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
    HOSTCMD_SET_MODE,			// Switch to specified device-independent mode: START/STOP
    HOSTCMD_REGWRITE,			// Write to a contiguous address range on a specified device (i.e, memory mapped, i2c device, etc.)
    HOSTCMD_REGREAD,			// Read from a contiguous chunk of registers.
    HOSTCMD_IOCTL				// Device specific operation.
};

#endif
