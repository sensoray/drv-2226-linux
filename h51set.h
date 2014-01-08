/////////////////////////////////////////////////
//Fujitsu MB86H51 encoder: register settings
//Host side software
/////////////////////////////////////////////////
//Copyright (C) Sensoray, 2008
/////////////////////////////////////////////////

//Register settings are transfered to the 2256ED1 in a single USB command packet
//The format of the packet is:
/// PACKET_HEADER               12 bytes
/// H51_MODE     mode;          ?? bytes
/// H51_RB_SIZE  h51RD;         16 bytes
/// h51APRM[]
/// h51IPRM[]
/// h51IPRV[]
/// h51IPRA[]                   ..total of ??? bytes

#ifndef H51SET_H
#define H51SET_H


unsigned int h51APRM[34][2] = {
    { 0x000800ea, 0x0a0c}, //0
    { 0x000800ec, 0x001e},    /*1     ec , ES_INFO_LENGTH, must be 0x28 for LPCM*/
    { 0x000800ee, 0x0000},    /*2     ee */
    { 0x000800f0, 0x0504},    /*3     f0 */
    { 0x000800f2, 0x4844},    /*4     f2 */
    { 0x000800f4, 0x4d56},    /*5     f4 */
    { 0x000800f6, 0x8804},    /*6     f6 */
    { 0x000800f8, 0x0fff},    /*7     f8 */
    { 0x000800fa, 0xfcfc},    /*8     fa */
    { 0x00080100, 0x6308},    /*9,   0x80100 */
    { 0x00080102, 0x0000},         /*10      2, TS_INFO_02, MUX_RATE dependent */
    { 0x00080104, 0x0000},         /*11      4, TS_INFO_04, MUX_RATE dependent */
    { 0x00080106, 0xffff},         /*12      6 */
    { 0x00080108, 0xffff},         /*13      8 */
    { 0x00080110, 0x1bf0},         /*14   0x80110 */
    { 0x00080112, 0x11f0},         /*15     12 */
    { 0x00080114, 0x1405},  /*16     14 */
    { 0x00080116, 0x0848},  /*17     16 */
    { 0x00080118, 0x444d},  /*18     18 */
    { 0x0008011a, 0x56ff},  /* 19    1a */
    { 0x0008011c, 0x1b47},  /*20     1c, 1080i*59.94, ES_INFO_0C, format dependent */
    { 0x0008011e, 0x3f28},  /*21     1e, 1080i*59.94, ES_INFO_0E, format dependent */
    { 0x00080120, 0x0464},  /*22     20 */
    { 0x00080122, 0x0028},  /*23     22 */
    { 0x00080124, 0x3f2a},  /*24     24 */
    { 0x00080126, 0x027e},  /*25     26 */
    { 0x00080128, 0xff03},  /*26     28, MP1L2, ES_INFO_18, audio format dependent */
    { 0x0008012a, 0xf100},  /*27     2a, = (0x7<<13)|(A_PID&0x1fff), where A_PID=0x1100 (h51IPRM[], addr0x01018)*/
    { 0x0008012c, 0xf000},  /*28     2c, MP1L2, ES_INFO_1C, audio format dependent */
    { 0x0008012e, 0x0508},  /*29     2e, ES_INFO_1E, only relevant for LPCM */
    { 0x00080130, 0x4844},  /*30     30, only relevant for LPCM */
    { 0x00080132, 0x4d56},  /*31     32, only relevant for LPCM */
    { 0x00080134, 0xff80},  /*32     34, only relevant for LPCM */
    { 0x00080136, 0x317f}    /*33     36, only relevant for LPCM, 2 channels, 16 bits/sample */
};

//indexes of elements that require mode-dependent adjustment
#define IND_ES_INFO_LENGTH 1     //aMode *
#define IND_TS_INFO_02    10    //MUX_RATE *
#define IND_TS_INFO_04    11    //MUX_RATE *
#define IND_ES_INFO_0C    20    //vFormat *
#define IND_ES_INFO_0E    21    //vFormat *
#define IND_ES_INFO_18    26    //aMode *
#define IND_ES_INFO_1C    28    //aMode *
#define IND_ES_INFO_1E    29    //aMode *

//Initialization-dedicated parameter register M settings
unsigned int h51IPRM[23][2] = {
    { 0x00001000, 0x0000},  /*0  0x01000 */
    { 0x00001002, 0x0002},  /*1    02, 2226 specific: out port - parallel, STCLK edge ^, STCLK output, 27Mhz */
    { 0x00001004, 0x0002},  /*2    04 */
    { 0x00001006, 0x0000},  /*3    06, MUX_RATE */
    { 0x00001008, 0x0000},  /*4    08 */
    { 0x0000100a, 0x0001},  /*5    0a */
    { 0x0000100c, 0x0000},  /*6    0c */
    { 0x0000100e, 0x0000},  /*7    0e */
    { 0x00001010, 0x0000},  /*8    10 */
    { 0x00001012, 0x0000},  /*9    12 */
    { 0x00001014, 0x0000},  /*10   14 */
    { 0x00001016, 0x1011},  /*11   16 */
    { 0x00001018, 0x1100},  /*12   18 */
    { 0x0000101a, 0x0100},  /*13   1a */
    { 0x0000101c, 0x001f},  /*14   1c */
    { 0x0000101e, 0x1001},  /*15   1e */
    { 0x00001020, 0x00e0},  /*16   20 */
    { 0x00001022, 0x00c0},  /*17   22, A_SID, audio format dependent */
    { 0x00001110, 0x0000},  /*18    0x01110 */
    { 0x00001112, 0x0000},  /*19    2 */
    { 0x00001114, 0x0000},  /*20    4 */
    { 0x00001116, 0x0000},  /*21    6 */
    { 0x00001120, 0x0001}   /*22    0x01120 GOP_INFO masked*/
};  

//indexes of elements that require mode-dependent adjustment
#define IND_MUX_RATE 3     //MUX_RATE *
#define IND_A_SID  17    //aMode *

//Initialization-dedicated parameter register V settings
unsigned int h51IPRV[28][2] = {
    { 0x00001400, 0x0000},  /*0   0x01400  */
    { 0x00001402, 0x0000},  /*1     1   02 */
    { 0x00001404, 0x0000},  /*2     04, V_RATE_MODE, mode dependent */
    { 0x00001406, 0x0000},  /*3     06, V_BITRATE */
    { 0x00001408, 0x0000},  /*4     08, AVE_V_BITRATE */
    { 0x0000140a, 0x0000},  /*5     0a, V_FORMAT, video format dependent */
    { 0x0000140c, 0x0000},  /*6     0c */
    { 0x0000140e, 0x0000},  /*7     0e, GOP_CLK, video format dependent */
    { 0x00001410, 0x0000},  /*8     10 */
    { 0x00001412, 0x0000},  /*9     12 */
    { 0x00001414, 0x0000},  /*10     14 */
    { 0x00001416, 0x0000},  /*11     16 */
    { 0x00001418, 0x0001},  /*12     18, 2256ed1 specific*/
    { 0x0000141a, 0x0000},  /*13     1a */
    { 0x0000141c, 0x0000},  /*14     1c */
    { 0x0000141e, 0x0000},  /*15     1e */
    { 0x00001420, 0x0000},  /*16     20 */
    { 0x00001422, 0x0000},  /*17     22, MAX_V_BITRATE, mode dependent */
    { 0x00001424, 0x0000},  /*18     24 */
    { 0x00001430, 0x0000},  /*19     0x01430, GOP_STRM, mode dependent */
    { 0x00001470, 0x0026},  /*20     0x01470, V_V420_FILTER_00, video format dependent*/
    { 0x00001472, 0x007d},  /*21      2, video format dependent */
    { 0x00001474, 0x0056},  /*22      4, video format dependent */
    { 0x00001476, 0x0007},   /*23      6, video format dependent */
    { 0x00001478, 0x0000},   /*24      8, video format dependent */
    { 0x0000147a, 0x0000},   /*25      a, video format dependent */
    { 0x0000147c, 0x0000},   /*26      c, video format dependent */
    { 0x0000147e, 0x0000},   /*27      e, video format dependent */
};  

//indexes of elements that require mode-dependent adjustment
#define IND_V_RATE_MODE   2     //vbr *
#define IND_V_BITRATE     3     //vBitrate *
#define IND_AVE_V_BITRATE 4     //vBitrate *
#define IND_V_FORMAT      5     //vFormat *
#define IND_GOP_CLK       7    //vFormat *
#define IND_MAX_V_BITRATE 17    //vFormat *
#define IND_GOP_STRM      19    //vFormat, gop *
#define IND_V_V420_FILTER_00    20    //vFormat; start of 4 consecutive registers *



//Initialization-dedicated parameter register A settings
//
unsigned int h51IPRA[14][2] = {
    { 0x00001800, 0x0000},  /*0x01800 */
    { 0x00001802, 0x0000},  /*02 AV_RESYNC_THRESHOLD, A_SAMPLE */
    { 0x00001804, 0x0000},  /*04, A_BITRATE */
    { 0x00001806, 0x00c1},  /*06, 2256ed1 specific I2S settings */
    // Note: ARM firmware 0x4c will automatically correct the
    //       master/slave setting based on ATTR_AUDH51_MASTER
    //       This is best done in the ARM because the FPGA direction
    //       needs set and to prevent bus contention.
    //       In short, do not worry about the lowest bit setting.
    //       Use ATTR_AUDH51_MASTER and NEVER set the AIC33 registers 8 or 9
    { 0x00001808, 0x00c1},  /*08 */
    { 0x0000180a, 0x0000},  /*0a */
    { 0x0000180c, 0x0000},  /*0c */
    { 0x0000180e, 0x0000},  /*0e */
    { 0x00001810, 0x0000},  /*10 */
    { 0x00001812, 0x0000},  /*12 AIN_OFFSET */ 
    { 0x00001820, 0x0018},  /*0x01820, (AC3 only) */
    { 0x00001830, 0x0000},  /*0x01830, (MP1L2 only) */
    { 0x00001850, 0x0032},  /*0x01850, (AAC only) */
    { 0x00001860, 0x0003}   /*0x01860, (LPCM only) */
};

//indexes of elements that require mode-dependent adjustment
#define IND_A_BITRATE     2     //aBitrate *




//VFMT values match those of the V_FORMAT field in IPRV
//Note: the differences between 59.94 and 60, and between 23.98 and 24
//are controlled by the setting of the pixel clock
#define VFMT_1080_60i       0       //1920x1080 (59.94i/60i) SMPTE274M supported
#define VFMT_1080_50i       1       //1920x1080 (50i) SMPTE274M supported
#define VFMT_720_60p        2       //1280x720 (59.94p/60p) SMPTE296M-2001 supported
#define VFMT_720_50p        3       //1280x720 (50p) SMPTE296M-2001 supported
#define VFMT_480_60i        4       //720x480 (59.94i) ITU-R BT.656-4 supported
#define VFMT_576_50i        5       //720x576 (50i) ITU-R BT.656-4 supported
#define VFMT_1080_24p       34      //1920x1080 (23.98p/24p) SMPTE274M supported
//#define VFMT_720_24p        42      //1280x720 (23.98p/24p) SMPTE296M-2001 supported

//AMODE values match those of the A_MODE field in APRA
#define AMODE_MP1L2         0       //MPEG-1 layer 2, default
#define AMODE_AC3           1       //AC3
#define AMODE_LPCM          2       //LPCM
#define AMODE_AAC           3       //AAC

//GOP
//according to .xls file, 1080 is always IBP, all other resolutions are always IBBP
//#define GOP_IBBP            0       //IBBP, supported for all resolutions
//#define GOP_IBP             3       //IBP, supported only for 1920x1088


//H51 operation mode
typedef struct {
    int         vFormat;        //video format (resolution) (see consts below)
    int         frRed;          //!=0 if frame rate is reduced: e.g. 59.94i vs. 60i
    int         aMode;          //audio compression mode (see consts below)
    int         vbr;            //!=0 for VBR; ==0 for CBR
    int         vBitrate;       //video bitrate in kbit/s
    int         aStereo;        //audio stereo/mono selection: !=0 if stereo
    int         aBitrate;       //audio bitrate in kbit/s (CURRENTLY UNSUPPORTED)
    int         gop;            //GOP structure (CURRENTLY UNSUPPORTED)
} H51_MODE;


#endif //H51SET_H
