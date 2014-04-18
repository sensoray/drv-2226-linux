// 2226 MODE struct
// (C) Sensoray, 2007
#ifndef __s2226mode_H
#define __s2226mode_H


//Structure defines data size in bytes for each of
//the four H51's register banks that require loading
typedef struct {
	int         sizeAPRM;
	int         sizeIPRM;
	int         sizeIPRV;
	int         sizeIPRA;
} H51_RB_SIZE;

//H51 operation mode
struct MODE2226 {
	int         vFormat;        //video format (resolution) (see consts below)
	int         frRed;          //!=0 if frame rate is reduced: e.g. 59.94i vs. 60i
	int         aMode;          //audio compression mode (see consts below)
	int         vbr;            //!=0 for VBR; ==0 for CBR
	int         vBitrate;       //video bitrate in kbit/s
	int         aStereo;        //audio stereo/mono selection: !=0 if stereo
	int         aBitrate;       //audio bitrate in kbit/s (CURRENTLY UNSUPPORTED)
	int         gop;            //GOP structure (CURRENTLY UNSUPPORTED)
	int v_pid;
	int a_pid;
	int pcr_pid;
	int pmt_pid;
	int sit_pid;
	int v_sid;
	int a_sid;
	int program_num;
};

int setH51regs (struct MODE2226 *mode);


// the number of registers in each register group
#define h51APRMregs 34
#define h51IPRMregs 23
#define h51IPRVregs 28
#define h51IPRAregs 14
extern unsigned int h51APRM[h51APRMregs][2];
extern unsigned int h51IPRM[h51IPRMregs][2];
extern unsigned int h51IPRV[h51IPRVregs][2];
extern unsigned int h51IPRA[h51IPRAregs][2];



#endif		//__s2226mode_H									

