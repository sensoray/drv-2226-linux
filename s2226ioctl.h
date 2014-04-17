/*** s2226ioctl.h
 * Copyright(C) Sensoray Company Inc. 2009-2010
 * @author: D.A., P.E.
 * 2226 IOCTL commands.  Linux
 */
#ifndef _S2226IOCTL_H
#define _S2226IOCTL_H

#include <linux/videodev2.h>

#define S2226_CID_BASE (V4L2_CTRL_CLASS_USER | 0x1070)

/* audio routing */
#define S2226_CID_AUDIOROUTE (S2226_CID_BASE + 1)
/* select source feeding the mpeg in */
#define S2226_CID_AUDMUX_MPEGIN (S2226_CID_BASE + 2)
/* select source feeding the audio line out */
#define S2226_CID_AUDMUX_LINEOUT (S2226_CID_BASE + 3)
/* select source feeding the sdi out */
#define S2226_CID_AUDMUX_SDIOUT (S2226_CID_BASE + 4)


/* audio input settings (individual left/right channel control */
/* AGC on: whether to turn AGC on or off */

#define S2226_CTRL_CLASS_AUDIN 0x00bc0000
#define S2226_CID_AUDIN_BASE (S2226_CTRL_CLASS_AUDIN | 0x900)
#define S2226_CID_AUDIN_CLASS (S2226_CTRL_CLASS_AUDIN | 1)

#define S2226_CID_AUDIN_AGC_ON_R (S2226_CID_AUDIN_BASE + 1) /*right*/
#define S2226_CID_AUDIN_AGC_ON_L (S2226_CID_AUDIN_BASE + 2) /*left*/
/* AGC gain (this is different from non-AGC gain) */
#define S2226_CID_AUDIN_AGC_GAIN_R (S2226_CID_AUDIN_BASE + 3) /*right*/
#define S2226_CID_AUDIN_AGC_GAIN_L (S2226_CID_AUDIN_BASE + 4) /*left*/
/* audio balanced(differential) settings */
/* 0 is default.  1 for balanced audio inputs */

#define S2226_CID_AUDIN_BAL_L (S2226_CID_AUDIN_BASE + 5) /*left*/
#define S2226_CID_AUDIN_BAL_R (S2226_CID_AUDIN_BASE + 6) /*right*/

/* standard audio gain (different from AGC gain) */
#define S2226_CID_AUDIN_GAIN_L (S2226_CID_AUDIN_BASE + 7) /*left*/
#define S2226_CID_AUDIN_GAIN_R (S2226_CID_AUDIN_BASE + 8) /*right*/



/* audio output settings */
#define S2226_CTRL_CLASS_AUDOUT 0x00bd0000
#define S2226_CID_AUDOUT_BASE (S2226_CTRL_CLASS_AUDOUT | 0x900)
#define S2226_CID_AUDOUT_CLASS (S2226_CTRL_CLASS_AUDOUT | 1)

/* audio output settings DAC (digital to audio for playback) volume */
#define S2226_CID_AUDOUT_DACVOL_L (S2226_CID_AUDOUT_BASE + 1)/*DAC volume left*/
#define S2226_CID_AUDOUT_DACVOL_R (S2226_CID_AUDOUT_BASE + 2)/*DAC volume right*/
/* mute the DAC output audio */
#define S2226_CID_AUDOUT_DACMUTE_L (S2226_CID_AUDOUT_BASE + 3)/*DAC mute left*/
#define S2226_CID_AUDOUT_DACMUTE_R (S2226_CID_AUDOUT_BASE + 4)/*DAC mute right*/


#define S2226_CID_AUDOUT_MONO_GAIN (S2226_CID_AUDOUT_BASE + 5)
#define S2226_CID_AUDOUT_MONO_MUTE (S2226_CID_AUDOUT_BASE + 6)

#define S2226_CID_AUDOUT_HP_GAIN_L (S2226_CID_AUDOUT_BASE + 7)
#define S2226_CID_AUDOUT_HP_GAIN_R (S2226_CID_AUDOUT_BASE + 8)

#define S2226_CID_AUDOUT_HP_MUTE_L (S2226_CID_AUDOUT_BASE + 9)
#define S2226_CID_AUDOUT_HP_MUTE_R (S2226_CID_AUDOUT_BASE + 10)


#define S2226_CID_AUDOUT_STEREO_GAIN_L (S2226_CID_AUDOUT_BASE + 11)
#define S2226_CID_AUDOUT_STEREO_GAIN_R (S2226_CID_AUDOUT_BASE + 12)

#define S2226_CID_AUDOUT_STEREO_MUTE_L (S2226_CID_AUDOUT_BASE + 13)
#define S2226_CID_AUDOUT_STEREO_MUTE_R (S2226_CID_AUDOUT_BASE + 14)



/* audio meter */
#define S2226_CTRL_CLASS_AUDMTR 0x00be0000
#define S2226_CID_AUDMTR_BASE (S2226_CTRL_CLASS_AUDMTR | 0x900)
#define S2226_CID_AUDMTR_CLASS (S2226_CTRL_CLASS_AUDMTR | 1)

#define S2226_CID_AUDMTR_CHANNEL (S2226_CID_AUDMTR_BASE + 1)
/** Get Audio Meter Peak and Decayed Level 
 * 23-bits, unsigned binary 0=min volume
 */

#define S2226_CID_AUDMTR_LEVEL_L (S2226_CID_AUDMTR_BASE + 2) /*left*/
#define S2226_CID_AUDMTR_LEVEL_R (S2226_CID_AUDMTR_BASE + 3) /*right*/

/** Get Audio Meter Peak and Decayed Level in dB
 * 11-bits, unsigned binary 0=max volume 
 * 2048 steps, -0.1 db each
 */
#define S2226_CID_AUDMTR_LEVELDB_L (S2226_CID_AUDMTR_BASE + 4)
#define S2226_CID_AUDMTR_LEVELDB_R (S2226_CID_AUDMTR_BASE + 5)




/* audio meter hold release 
 * 1 = force release of 'held' output
 * 0 = allow holding of highest db value per the set hold time
 */
#define S2226_CID_AUDMTR_HOLDREL (S2226_CID_AUDMTR_BASE + 6)
/* audio meter hold time */
#define S2226_CID_AUDMTR_HOLDTIME (S2226_CID_AUDMTR_BASE + 7)


/* hold value left and right side */
/*
 * hld  : 11-bits, unsigned binary 0=max volume. 2048 steps, -0.1 db each
 * clip : 1 indicates (0x7FFFFF or 0x800000 detected)
 */
#define S2226_CID_AUDMTR_HOLD_L (S2226_CID_AUDMTR_BASE + 8)
#define S2226_CID_AUDMTR_HOLD_R (S2226_CID_AUDMTR_BASE + 9)
/* clip detect */
#define S2226_CID_AUDMTR_CLIP_L (S2226_CID_AUDMTR_BASE + 10)
#define S2226_CID_AUDMTR_CLIP_R (S2226_CID_AUDMTR_BASE + 11)


/* audio meter test settings 
 * 0- disabled/normal mode (no TEST)
 * 1- force to zero
 * 2- force to clip
 * 3- force to -6dB
 */
#define S2226_CID_AUDMTR_TEST (S2226_CID_AUDMTR_BASE + 12)


typedef struct {
        int idx;
} start_param_t; //S2226_IOC_START_ENCODE, S2226_IOC_START_DECODE

typedef struct {
        int idx; //S2226_IOC_STOP_ENCODE, S2226_IOC_STOP_DECODE
} stop_param_t;


// 2226 error codes (see INTRESULT_xyz error codes in types2226.h
#define S2226_ERR_BASE               (-132)


// ========End Audio Settings=============


typedef struct {
    int addr;
    int value;
} audio_reg_t;

struct io_reg {
    int addr;
    int val;
};

struct attr_ext {
    int attr;
    int val;
    int val2;
};

// Maximum write size (256-14)=242
// maximum read size 512-4 = 508
struct io_burst {
    int addr; // address
    unsigned char data[512]; // see above maximum size
    int len; //bytes (must be multiple of 2)
};

struct flash_param {
	unsigned int addr;
	unsigned int len;
	unsigned char data[500];
};

typedef struct {
	unsigned char addr;
	unsigned char len;
	unsigned char data[128];
} i2c_param_t;




// V4L 2226 specific ioctls
#define S2226_VIDIOC_LOCK_OVERLAY  (BASE_VIDIOC_PRIVATE + 0)
#define S2226_VIDIOC_UNLOCK_OVERLAY (BASE_VIDIOC_PRIVATE + 1)
#define S2226_VIDIOC_GET_MODE      (BASE_VIDIOC_PRIVATE + 5)
#define S2226_VIDIOC_STARTDECODE   (BASE_VIDIOC_PRIVATE + 6)
#define S2226_VIDIOC_STOPDECODE    (BASE_VIDIOC_PRIVATE + 7)
#define S2226_VIDIOC_GET_INPUT     (BASE_VIDIOC_PRIVATE + 9)
// returns 1 if decoding, 0 otherwise
#define S2226_VIDIOC_DECODE_STATE  (BASE_VIDIOC_PRIVATE + 12)
#define S2226_VIDIOC_FX2_VER       (BASE_VIDIOC_PRIVATE + 13)







//=============================================================
// internal ioctls (debug, etc...)
// No support provided.  For debug, board bringup,
// and development only.
//=============================================================
#define S2226_VIDIOC_GET_STATUS    (BASE_VIDIOC_PRIVATE + 14)
#define S2226_VIDIOC_SDII_WR       (BASE_VIDIOC_PRIVATE + 15)
#define S2226_VIDIOC_SDII_RD       (BASE_VIDIOC_PRIVATE + 16)
#define S2226_VIDIOC_SDIO_WR       (BASE_VIDIOC_PRIVATE + 17)
#define S2226_VIDIOC_SDIO_RD       (BASE_VIDIOC_PRIVATE + 18)
#define S2226_VIDIOC_FPGA_WR       (BASE_VIDIOC_PRIVATE + 19)
#define S2226_VIDIOC_FPGA_WR_BURST (BASE_VIDIOC_PRIVATE + 20)
#define S2226_VIDIOC_FPGA_WR_BURST_FAST (BASE_VIDIOC_PRIVATE + 21)
#define S2226_VIDIOC_FPGA_WR_ADDRDATA (BASE_VIDIOC_PRIVATE + 22)

#define S2226_VIDIOC_FPGA_RD       (BASE_VIDIOC_PRIVATE + 23)
#define S2226_VIDIOC_FPGA_RD_BURST (BASE_VIDIOC_PRIVATE + 24)
#define S2226_VIDIOC_BOOT_H51      (BASE_VIDIOC_PRIVATE + 25)
#define S2226_VIDIOC_BOOT_FPGA     (BASE_VIDIOC_PRIVATE + 26)
#define S2226_VIDIOC_AUDIO_WR      (BASE_VIDIOC_PRIVATE + 27)
#define S2226_VIDIOC_AUDIO_RD      (BASE_VIDIOC_PRIVATE + 28)
#define S2226_VIDIOC_H51_WR        (BASE_VIDIOC_PRIVATE + 29)
#define S2226_VIDIOC_H51_RD        (BASE_VIDIOC_PRIVATE + 30)
#define S2226_VIDIOC_SET_ATTR      (BASE_VIDIOC_PRIVATE + 31)
#define S2226_VIDIOC_GET_ATTR      (BASE_VIDIOC_PRIVATE + 32)
#define S2226_VIDIOC_RESET_BOARD   (BASE_VIDIOC_PRIVATE + 33) //resets the ARM, which in turn will
                                       //reset the FPGA and H51 and 
                                       // other devices(except USB FX2 chip)
#define S2226_VIDIOC_FLASH_WR      (BASE_VIDIOC_PRIVATE + 34) //internal use only, voids warranty
                                       //if misused
#define S2226_VIDIOC_FLASH_ERASE   (BASE_VIDIOC_PRIVATE + 35) //internal use only, voids warranty
#define S2226_VIDIOC_I2C_TX        (BASE_VIDIOC_PRIVATE + 36) //internal use only
#define S2226_VIDIOC_FLASH_RD      (BASE_VIDIOC_PRIVATE + 37) //internal use only
#define S2226_VIDIOC_ARM_VER       (BASE_VIDIOC_PRIVATE + 38) //internal use only

// call before S2226_VIDIOC_RESET_CPU to load initial failsafe firmware
// This is only used if the firmware update was corrupted
// or cancelled early
#define S2226_VIDIOC_FX2SAM_LO     (BASE_VIDIOC_PRIVATE + 39) //internal use only
// same as above, but refactored.  loads base firmware after reset
#define S2226_VIDIOC_SET_BASEFW    (BASE_VIDIOC_PRIVATE + 40) //internal use only

#define S2226_VIDIOC_NOP           (BASE_VIDIOC_PRIVATE + 41) //internal use only

// default state.  If FX2SAM hi, ARM will look for secondary(newest) firmware
// Normally, this command does not need to be used.  Only for firmware
// updating.
#define S2226_VIDIOC_FX2SAM_HI     (BASE_VIDIOC_PRIVATE + 42) //internal use only
// same as above, but refactored.  loads latest firmware after reset
#define S2226_VIDIOC_SET_NEWFW     (BASE_VIDIOC_PRIVATE + 43)

#define S2226_VIDIOC_RESET_USB     (BASE_VIDIOC_PRIVATE + 44) //internal use only
#define S2226_VIDIOC_PRIME_FX2     (BASE_VIDIOC_PRIVATE + 45) //internal use only
#define S2226_VIDIOC_DEFAULT_PARAMS (BASE_VIDIOC_PRIVATE + 46) //internal use only
#define S2226_VIDIOC_DPB_SIZE	(BASE_VIDIOC_PRIVATE + 47) //internal use only


#define S2226_VIDIOC_GOP_STRUCT    (BASE_VIDIOC_PRIVATE + 48) //internal use only
// Warning: Using non-standard values is not supported.
// Recommended GOP struct is used by default without using this IOCTL
// GOP_STRUCT -1 default value. use recommended settings
// GOP_STRUCT 0 (override default, use IBBP) 
// GOP_STRUCT 1 (override default, use IP)
// GOP_STRUCT 3 (override default, use IBP) 
#define S2226_VIDIOC_AINOFFSET      (BASE_VIDIOC_PRIVATE + 49)  // internal use only
#define S2226_VIDIOC_AV_RESYNC      (BASE_VIDIOC_PRIVATE + 50) // (threshold) internal use only
#define S2226_VIDIOC_VIDDEC_RD      (BASE_VIDIOC_PRIVATE + 51)
#define S2226_VIDIOC_SDISPLIT_RD    (BASE_VIDIOC_PRIVATE + 52)// videncoder is write only




#endif //_S2226IOCTL_H

