/*** s2226ioctl.h
 * Copyright(C) Sensoray Company Inc. 2009-2010
 * @author: D.A., P.E.
 * 2226 IOCTL commands.  Linux
 */
#ifndef _S2226IOCTL_H
#define _S2226IOCTL_H


// for mode_param_t, S2226_IOC_SET_MODE 
#define S2226_IDX_AMODE     1
#define S2226_IDX_ABITRATE  2
#define S2226_IDX_VBITRATE  3
#define S2226_IDX_STEREO    4
// audio route (default is LINE1L)
#define S2226_IDX_AUDROUTE 5  // left
// values to use for AUDROUTE setting
#define S2226_AUDIOROUTE_LINE1L 0
#define S2226_AUDIOROUTE_LINE2L 1
#define S2226_AUDIOROUTE_LINE1L_BYPASS 2 // passthrough, Line1L in -> Line out
#define S2226_AUDIOROUTE_LINE2L_BYPASS 3
// audio input settings (individual left/right channel control
//                       by customer request/NRE)
// AGC on: whether to turn AGC on or off
#define S2226_IDX_AUDIN_AGC_ON_R 6 // right
#define S2226_IDX_AUDIN_AGC_ON_L 7 // left
#define S2226_IDX_AUDIN_AGC_ON  8 // right and left
// AGC gain (different from standard gain)
#define S2226_IDX_AUDIN_AGC_GAIN_R 9 // right
#define S2226_IDX_AUDIN_AGC_GAIN_L 10 // left
#define S2226_IDX_AUDIN_AGC_GAIN  11 // right and left
// audio balanced(differential) settings 
// 0 is default.  1 for balanced audio inputs
#define S2226_IDX_AUDIN_BAL_R 12   //right
#define S2226_IDX_AUDIN_BAL_L 13  // left
#define S2226_IDX_AUDIN_BAL   14  // right and left
// standard audio gain (different from AGC gain)
#define S2226_IDX_AUDIN_GAIN_R 15   //right
#define S2226_IDX_AUDIN_GAIN_L 16  // left
#define S2226_IDX_AUDIN_GAIN   17  // right and left

// future TBD
// audio output settings DAC (digital to audio for playback) volume
//#define S2226_IDX_AUDOUT_DACVOL_R 18 // DAC volume right
//#define S2226_IDX_AUDOUT_DACVOL_L 19 // DAC volume left
//#define S2226_IDX_AUDOUT_DACVOL   20 // DAC volume right/left


typedef struct {
	int idx;
	int val;
} mode_param_t;  //S2226_IOC_SET_MODE, S2226_IOC_GET_MODE

typedef struct {
        int idx;
} start_param_t; //S2226_IOC_START_ENCODE, S2226_IOC_START_DECODE

typedef struct {
        int idx; //S2226_IOC_STOP_ENCODE, S2226_IOC_STOP_DECODE
} stop_param_t;

// for S2226_IOC_SET_LEVEL, S2226_IOC_GET_LEVEL
#define S2226_LEVEL_BRIGHTNESS  1
#define S2226_LEVEL_CONTRAST    2
#define S2226_LEVEL_HUE         4
#define S2226_LEVEL_SATURATION  8
struct level_param {
	int idx;
	int val;
};//S2226_IOC_SET_LEVEL, S2226_IOC_GET_LEVEL



// ========End Audio Settings=============
// 2226 error codes (see INTRESULT_xyz error codes in types2226.h
#define S2226_ERR_BASE               (-132)


// ========End Audio Settings=============

/*



#define S2226_IOC_LAST_IOCTL    0x105c // update when updating anything below
                                       // not sequential

// user ioctls for 2226
#define S2226_IOC_LOCK_OVERLAY  0x1054
#define S2226_IOC_UNLOCK_OVERLAY 0x1056
#define S2226_IOC_SET_MODE      0x1000
#define S2226_IOC_STARTENCODE   0x1002
#define S2226_IOC_STOPENCODE    0x1004
#define S2226_IOC_GET_MODE      0x1008
#define S2226_IOC_STARTDECODE   0x100a
#define S2226_IOC_STOPDECODE    0x100c
#define S2226_IOC_SET_INPUT     0x1020
#define S2226_IOC_GET_INPUT     0x1052
#define S2226_IOC_SET_LEVEL     0x1042
#define S2226_IOC_GET_LEVEL     0x1044
// returns 1 if decoding, 0 otherwise
#define S2226_IOC_DECODE_STATE  0x1046
#define S2226_IOC_FX2_VER       0x1048



//=============================================================
// internal ioctls (debug, etc...)
// No support provided.  For debug, board bringup,
// and development only.
//=============================================================
#define S2226_IOC_SDII_WR       0x1010 // low level(for debug, bringup)
#define S2226_IOC_SDII_RD       0x1012 // low level(for debug, bringup)
#define S2226_IOC_SDIO_WR       0x1014 // low level(for debug, bringup)
#define S2226_IOC_SDIO_RD       0x1016 // low level(for debug, bringup)
#define S2226_IOC_FPGA_WR       0x1018 // low level(for debug, bringup)
#define S2226_IOC_FPGA_WR_BURST 0x1019 // low level(for debug, bringup)
#define S2226_IOC_FPGA_WR_BURST_FAST 0x104a // low level(for debug, bringup)
#define S2226_IOC_FPGA_WR_ADDRDATA 0x1050 // low level(for debug, bringup)

#define S2226_IOC_FPGA_RD       0x101a // low level(for debug, bringup)
#define S2226_IOC_FPGA_RD_BURST 0x101b // low level(for debug, bringup)
#define S2226_IOC_BOOT_H51      0x101c // low level(for debug, bringup)
#define S2226_IOC_BOOT_FPGA     0x101e // low level(for debug, bringup)
#define S2226_IOC_AUDIO_WR      0x1022
#define S2226_IOC_AUDIO_RD      0x1024
#define S2226_IOC_H51_WR        0x1026
#define S2226_IOC_H51_RD        0x1028
#define S2226_IOC_SET_ATTR      0x102a
#define S2226_IOC_GET_ATTR      0x102c

#define S2226_IOC_RESET_H51     0x102e // refactored, but left for 
                                       // backward compatibility

#define S2226_IOC_RESET_BOARD   0x102e //resets the ARM, which in turn will
                                       //reset the FPGA and H51 and 
                                       // other devices(except USB FX2 chip)

#define S2226_IOC_FLASH_WR      0x1030 //internal use only, voids warranty
                                       //if misused
#define S2226_IOC_FLASH_ERASE   0x1032 //internal use only, voids warranty
#define S2226_IOC_I2C_TX        0x1034 //internal use only
#define S2226_IOC_FLASH_RD      0x1036 //internal use only
#define S2226_IOC_ARM_VER       0x1038 //internal use only

// call before S2226_IOC_RESET_CPU to load initial failsafe firmware
// This is only used if the firmware update was corrupted
// or cancelled early
#define S2226_IOC_FX2SAM_LO     0x103a //internal use only
// same as above, but refactored.  loads base firmware after reset
#define S2226_IOC_SET_BASEFW    0x103a //internal use only

#define S2226_IOC_NOP           0x103b //internal use only

// default state.  If FX2SAM hi, ARM will look for secondary(newest) firmware
// Normally, this command does not need to be used.  Only for firmware
// updating.
#define S2226_IOC_FX2SAM_HI     0x103c //internal use only
// same as above, but refactored.  loads latest firmware after reset
#define S2226_IOC_SET_NEWFW     0x103c

#define S2226_IOC_RESET_USB     0x103d //internal use only
#define S2226_IOC_PRIME_FX2     0x103e //internal use only
#define S2226_IOC_DEFAULT_PARAMS 0x103f //internal use only
#define S2226_IOC_DPB_SIZE	0x1040 //internal use only


#define S2226_IOC_GOP_STRUCT    0x104c //internal use only
// Warning: Using non-standard values is not supported.
// Recommended GOP struct is used by default without using this IOCTL
// GOP_STRUCT -1 default value. use recommended settings
// GOP_STRUCT 0 (override default, use IBBP) 
// GOP_STRUCT 1 (override default, use IP)
// GOP_STRUCT 3 (override default, use IBP) 
#define S2226_IOC_AINOFFSET      0x140e  // internal use only
#define S2226_IOC_AV_RESYNC      0x140f // (threshold) internal use only
#define S2226_IOC_VIDDEC_RD      0x105a
#define S2226_IOC_SDISPLIT_RD      0x105c // videncoder is write only

*/

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




// new V4L ioctl
#define S2226_VIDIOC_LOCK_OVERLAY  (BASE_VIDIOC_PRIVATE + 0)
#define S2226_VIDIOC_UNLOCK_OVERLAY (BASE_VIDIOC_PRIVATE + 1)
#define S2226_VIDIOC_SET_MODE      (BASE_VIDIOC_PRIVATE + 2)
#define S2226_VIDIOC_STARTENCODE   (BASE_VIDIOC_PRIVATE + 3)
#define S2226_VIDIOC_STOPENCODE    (BASE_VIDIOC_PRIVATE + 4)
#define S2226_VIDIOC_GET_MODE      (BASE_VIDIOC_PRIVATE + 5)
#define S2226_VIDIOC_STARTDECODE   (BASE_VIDIOC_PRIVATE + 6)
#define S2226_VIDIOC_STOPDECODE    (BASE_VIDIOC_PRIVATE + 7)
#define S2226_VIDIOC_SET_INPUT     (BASE_VIDIOC_PRIVATE + 8)
#define S2226_VIDIOC_GET_INPUT     (BASE_VIDIOC_PRIVATE + 9)
#define S2226_VIDIOC_SET_LEVEL     (BASE_VIDIOC_PRIVATE + 10)
#define S2226_VIDIOC_GET_LEVEL     (BASE_VIDIOC_PRIVATE + 11)
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

