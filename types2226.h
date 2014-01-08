/////////////////////////////////////////////////////////////////////////
// File      : types2226.h
// Function  : fundamental types used throughout 2226 firmware source
/////////////////////////////////////////////////////////////////////////

#ifndef TYPES_2226_H
#define TYPES_2226_H

#ifdef u8
 #undef u8
#endif

#ifndef DRIVER_BUILD
typedef char            s8;
typedef unsigned char   u8;
typedef signed short    s16;
typedef unsigned short  u16;
typedef signed int      s32;
typedef unsigned int    u32;
typedef unsigned int    uint;
#ifndef WIN32
typedef char            BOOL;
#endif
#endif


#define FALSE           0
#define TRUE            1

#ifndef NULL
#define NULL            ((void *)0)
#endif

// Event notification codes.
#define EVENT_PERSONA_HALT		1		// Board personality is no longer valid; stop using FX2 fifos. No optional arg.
#define EVENT_PERSONA_RUN		2		// Board personality is valid (i.e., USB endpoint reconfiguration complete); ok to use FX2 fifos. Optional arg = personality code.
#define EVENT_FPGASTATE			4		// FPGA is about to go offline, or has just gone online. Optional arg: 0 (offline) | 1 (online).
#define EVENT_RESPONSE          8       // response (from command or asynchronous) wants to be sent out (higher priority than raw preview stream)
#define EVENT_FPGASCALAR		16      // FPGA scalar parameters have changed
#define EVENT_PERSONA_USBRESET  32		// USB was reset
// Event notification message.
typedef struct EVENT_MSG {
	u8	event;		// EVENT_PERSONA | EVENT_FPGASTATE.
	u8	arg;		// Personality code | 0,1 = fpga going offline/gone online
} EVENT_MSG;

#define MAX_EVENT_MSG_EXT 8
typedef struct EVENT_MSG_EXT {
	u8	event;		// EVENT_PERSONA | EVENT_FPGASTATE | EVENT_COMMAND_RESP
	u8	arg;		// Personality code | 0,1 = fpga going offline/gone online
    u8  resp[MAX_EVENT_MSG_EXT];    // size reduced response to reduce memory usage.
    u16 len;       
} EVENT_MSG_EXT;


// Configuration constants ----------------------------------------------------


#define LED_TOGGLE_MSEC					200
#define CMDQ_MAXITEMCOUNT				10					// Up to ten messages may pend in the queue at the same time.
#define FX2_BAUDRATE					115200				// Our uart baud rate, which must match that of FX2's uart.
#define AUDIO_FORMAT					H51_CMD_PRM_MP1		// Audio format for both encode and decode: MPEG-1 layer2. This is a constant in this version.

#define FW_UPLOAD_BUFSIZE				512					// Transfer firmware chunks of this size from dataflash to ram buf. This is a shared, dedicated buffer that consumes no stack space.

#define MAX_HOSTCMD_REPLY_SIZE			512					// Maximum size of intmsg that contains a reply to a host command, in bytes.

#define MAX_INTMSG_SIZE					512                 // must be 512 bytes.  int EP may be 64-512 bytes (see S2226_ATTR, G_attr.int_ep_size)


// Manifest constants ---------------------------------------------------------


// Priorities for the application tasks.
#define PRIORITY_P3					( tskIDLE_PRIORITY + 1 ) // lowest priority
#define PRIORITY_P2					( tskIDLE_PRIORITY + 2 )
#define PRIORITY_P1					( tskIDLE_PRIORITY + 3 )

#define USB_MAX_WORDS					256



// Enumerated states for the H51 codec device.
#define H51STATE_OFFLINE				0					// Powered up, held in reset.
#define H51STATE_OUTOFRESET             1                   // after reset deasserted, but before bootloader executing
#define H51STATE_UPLOAD_READY			2					// Bootloader executing, ready for firmware upload.
#define H51STATE_BOOTLOADER_FAIL		3					// Bootloader failed to execute within allotted time.
#define H51STATE_MAINBOOT_FAIL			4					// Failed to signal startup of uploaded firmware.
#define H51STATE_CHECK_FAIL				5					// Failed to respond to Check command following firmware upload.
#define H51STATE_ONLINE					6					// Firmware has been uploaded

// Enumerated states for the FX2 USB interface device.
#define FX2STATE_OFFLINE				0
#define FX2STATE_ONLINE					1					// Ready to communicate.

// Enumerated states for the FPGA device.
#define FPGASTATE_OFFLINE				0
#define FPGASTATE_ONLINE				1					// Ready to communicate.

#define FPGASCALAR_CHANGED             1					// scalar parameters changed

// Enumerated states for SAA7115 video decoder.
#define SAA7115_OFFLINE					0					// Not yet initialized via I2C.
#define SAA7115_UNPOPULATED				1					// Not detected via I2C.
#define SAA7115_ONLINE					2					// Detected and initialized with defaults.

// Enumerated states for SAA7115 video decoder.
#define SAA7121_OFFLINE					0					// Not yet initialized via I2C.
#define SAA7121_UNPOPULATED				1					// Not detected via I2C.
#define SAA7121_ONLINE					2					// Detected and initialized with defaults.

// Enumerated states for AIC33 audio chip
#define AIC33_OFFLINE					0					// Not yet initialized via I2C.
#define AIC33_UNPOPULATED				1					// Not detected via I2C.
#define AIC33_ONLINE					2					// Detected and initialized with defaults.

// Enumerated states for LMH307 SDI splitter output driver
#define LMH307_OFFLINE					0					// Not yet initialized via I2C.
#define LMH307_UNPOPULATED				1					// Not detected via I2C.
#define LMH307_ONLINE					2					// Detected and initialized with defaults.
//segment states
#define LEDSEG_OFFLINE					0					// Not yet initialized via I2C.
#define LEDSEG_UNPOPULATED				1					// Not detected via I2C.
#define LEDSEG_ONLINE					2					// Detected and initialized with defaults.


// Command queue message types, each represented by a u32 value:
#define CMDQ_MSG_NOP					0					// No operation.
#define CMDQ_MSG_HOST_CMD				1					// Host command is pending in host command buffer.
#define CMDQ_MSG_HOST_BADCHECKSUM		2					// Received checksum doesn't match computed checksum.
#define CMDQ_MSG_HOST_ERR				3					// Invalid command received from host.
#define CMDQ_MSG_H51_CMDACK				4					// H51 command ack is pending.
#define CMDQ_MSG_H51_NOTIFY				5					// H51 asynchronous notification message is pending.
#define CMDQ_MSG_H51_VSYNC				6					// H51 vertical sync message is pending. 
#define CMDQ_MSG_H51_ERROR				7					// H51 error message is pending.
//#define CMDQ_MSG_DEBUG_START_ENCODE     8	    		// Host command is pending in host command buffer.
//#define CMDQ_MSG_DEBUG_START_DECODE     9	    		// Host command is pending in host command buffer.

typedef u32		CMDQ_MESSAGE;								// Command queue message type.

// USB interrupt message types. All intmsg's have one of these types.
#define INTTYPE_CMDREPLY				1					// Response to usb host command.
#define INTTYPE_ASYNCEVENT				2					// fhronous event notification.

// Types of async events found in interrupt messages.
#define ASYNCEVENT_VSYNC				1					// VSYNC detected.
#define ASYNCEVENT_H51_MSG				2					// H51 issued an unsolicited message.
#define ASYNCEVENT_SYSERROR				3					// H51 internal error.
#define ASYNCEVENT_RAWDATA_START        4                   // raw streaming preview data
#define ASYNCEVENT_RAWDATA_END          5                   // raw streaming preview data
#define ASYNCEVENT_SNAPSHOT_START       6                   // snapshot data start


// USB interrupt message error codes.
typedef enum INTMSG_ERRCODE {
	// Messages common to all products:
	INTRESULT_OK = 0,				// No errors.
	INTRESULT_CHECKSUM,				// Received host command has checksum error.
	INTRESULT_MALFORMED,
	INTRESULT_BADPARAM,				// Illegal argument value in host command.
	INTRESULT_BADOPCODE,			// Unsupported host command opcode.
	INTRESULT_BADSIZE,				// Illegal host command size.
	// Messages specific to 2226:
	INTRESULT_H51_NOTREADY,			// H51 is offline.
	INTRESULT_H51_TIMEOUT,			// Timed out waiting for H51 cmdack or vsync.
	INTRESULT_H51_ABNORMAL,			// H51 cmdack has Abnormal flag asserted.
	INTRESULT_H51_ERR_UNKNOWN,		// Unhandled H51 communication error.
	INTRESULT_FPGA_NOTREADY,		// FPGA is offline.  [10]
	INTRESULT_FPGA_TIMEOUT,			// Timed out during FPGA configuration.
	INTRESULT_FPGA_CONFIGERR,		// At end of FPGA configuration, STATUS or CONF_DONE or both are low. 0x0c
	INTRESULT_FPGA_ONLINEERR,		// After FPGA config we can't successfully read register signature.
	INTRESULT_FLASHERR,				// Flash read or write fault.
	INTRESULT_NOFLASHIMAGE,			// Firmware image unavailable for specified device.
	INTRESULT_STREAMNOTIFY_FAIL,	// Stream task did not ack notification.
    INTRESULT_NO_MCLK,
    INTRESULT_I2CDEV_NOTREADY,
    INTRESULT_NOT_IDLE,             // host forgot to stop stream before set input, etc...
    INTRESULT_PREVIEWNOTIFY_FAIL,	// Preview task did not ack notification.
	INTRESULT_MAXERRCODE			// Dummy error code, always last.
} INTMSG_ERRCODE;

// System personality, based on USB enpoint configuration.
typedef enum PERSONALITY {
	PERSONA_DEFAULT = 0,		// Default is diagnostic/firmware_upload mode.
	PERSONA_ENCODE,				// Encode mode.
	PERSONA_DECODE,				// Decode mode
    PERSONA_PREVIEW,            // Same as Encode USB configuration, but controls previewtask
	MAX_PERSONA					//  always last
} PERSONALITY;

// H51 operating modes.
#define STREAM_MODE_IDLE				0x00
#define STREAM_MODE_ENCODE				0x10
#define STREAM_MODE_DECODE				0x20

// H51 decode sub commands
#define STREAM_SUBMODE_NORMAL		0x00
#define STREAM_SUBMODE_SLOW			0x01
#define STREAM_SUBMODE_PAUSE		0x02
#define STREAM_SUBMODE_ADVANCE		0x03


// internal attributes
#define MAX_SAVED_MSG_SIZE 32
#define DEFAULT_INPUT_DELAY (500) // default input startup delay. 
                                //may need
                                // to increase this(ATTR_INPUT_DELAY)
                                // for HD sources due to FPGA's
                                // processing

#endif  // #ifdef TYPES_2226_H
