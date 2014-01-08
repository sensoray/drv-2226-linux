/////////////////////////////////////////////////
//Fujitsu MB86H51 encoder: register settings
//Host side software
/////////////////////////////////////////////////
//Copyright (C) Sensoray, 2009
/////////////////////////////////////////////////


typedef int __int32;
typedef short __int16;

//Structure defines data size in bytes for each of
//the four H51's register banks that require loading
typedef struct {
    __int32         sizeAPRM;
    __int32         sizeIPRM;
    __int32         sizeIPRV;
    __int32         sizeIPRA;
} H51_RB_SIZE;

#include "h51set.h"

		 
// fill in the sizes of the register banks
// and load the register data immediately after it in memory
void loadH51rb(H51_RB_SIZE *rb_size)
{
	__int16 *rb;
	int n;
	
// TODO: don't use this until we fix the register layouts
	rb_size->sizeAPRM = sizeof(h51APRM);
	rb_size->sizeIPRM = sizeof(h51IPRM);
	rb_size->sizeIPRV = sizeof(h51IPRV);
	rb_size->sizeIPRA = sizeof(h51IPRA);

	rb = (__int16 *) (rb_size + 1);
	n = 0;
	while (n*2 < sizeof(h51APRM))
		*rb++ = h51APRM[n++][1];
	n = 0;
	while (n*2 < sizeof(h51IPRM))
		*rb++ = h51IPRM[n++][1];
	n = 0;
	while (n*2 < sizeof(h51IPRV))
		*rb++ = h51IPRV[n++][1];
	n = 0;
	while (n*2 < sizeof(h51IPRA))
		*rb++ = h51IPRA[n++][1];
}


//calculates frame rate based on vFormat setting
void frameRate (__int16 vFormat, int *num, int *denom)
{
	*denom = 1;
	
	switch (vFormat) {
	default:
	case VFMT_1080_60i:
	case VFMT_480_60i:
		*num = 30000;
		*denom = 1001;
		break;
	case VFMT_1080_50i:
	case VFMT_576_50i:
		*num = 25;
		break;
	case VFMT_720_60p:
		*num = 60000;
		*denom = 1001;
		break;
	case VFMT_720_50p:
		*num = 50;
		*denom = 1;
		break;
	case VFMT_1080_24p:
		*num = 24;
		break;
    }
}

//calculates MUX_RATE used in H51 settings
//based on MB86H51_setting_parameter_r4.xls.
//Audio sampling frequency is assumed to be 48kHz
//(the only one supported by the H51).
unsigned int muxRate (H51_MODE *mode)
{

    unsigned int         v0, v1, v2, v3;
    unsigned int         a0, a1, a2, a3;
    unsigned int         s0, s1, s2, s3;
    int         samples[4] = {1152, 1536, 240, 1024};   //from .xls
    int         hdrSize[4] = {14, 17, 18, 14};          //from .xls
    int         aSampFreq = 48000;      //Hz
    //default H51 values, ms (see CCS, p.38-39)
    int         PATcycle = 90;
    int         PMTcycle = 90;
    int         SITcycle = 900;
    int         PCRcycle = 30;
    unsigned int         mr;
    int		frNum, frDenom;
    
    // get the frame rate fraction
    frameRate(mode->vFormat, &frNum, &frDenom);
    
    v0 = (19 * 8 * frNum / frDenom) + 1;
    v1 = mode->vBitrate * 1000 + v0;
    v2 = (183 * 8 * frNum / frDenom) + 1;
    v3 = (((v1 + v2) * 188) / 184) + 1;

    a0 = (mode->aBitrate * 1000 * samples[mode->aMode] / aSampFreq / 8) + 1;
    a1 = a0 + hdrSize[mode->aMode];
    a2 = (a1 / 184) + 1;
    a3 = (a2 * 188 * 8 * aSampFreq / samples[mode->aMode]) + 1;
    s0 = (188 * 8 * 1000 / PATcycle) + 1;
    s1 = (188 * 8 * 1000 / PMTcycle) + 1;
    s2 = (188 * 8 * 1000 / SITcycle) + 1;
    s3 = (188 * 8 * 1000 / PCRcycle) + 1;

    mr = (((v3 + a3 + s0 + s1 + s2 + s3) * 21) / 20) + 1;  // 21/20 is 1.05
    return mr;
}

//Modifies the values of H51's register banks according to mode settings.
//Assumes the arrays are global.
//Returns non-zero if error.
int setH51regs (H51_MODE *mode)
{
    int         mr;
    __int16     tmp;

    mr = muxRate (mode) / 1000; // muxRate returns bit/s

    h51APRM[IND_ES_INFO_LENGTH][1] = (mode->aMode == AMODE_LPCM) ? 0x28 : 0x1e;

    tmp = mr * 1000 / 400;
    h51APRM[IND_TS_INFO_02][1] = 0xc000 | (tmp >> 8);
    h51APRM[IND_TS_INFO_04][1] = ((tmp << 8) & 0xff00) | 0x00ff;

    switch (mode->vFormat) {
        case VFMT_1080_60i:
            h51APRM[IND_ES_INFO_0C][1] = 0x1b47;
            h51APRM[IND_ES_INFO_0E][1] = 0x3f28;
            h51IPRV[IND_V_FORMAT][1] = 0x1000;
            if (mode->frRed) {
		h51IPRV[IND_GOP_CLK][1] = 0xc000;
	    } else {
		h51IPRV[IND_GOP_CLK][1] = 0xc001;
	    }
            h51IPRV[IND_GOP_STRM][1] = 0x0302;
            h51IPRV[IND_V_V420_FILTER_00][1] = 0x0026;
            h51IPRV[IND_V_V420_FILTER_00 + 1][1] = 0x007d;
            h51IPRV[IND_V_V420_FILTER_00 + 2][1] = 0x0056;
            h51IPRV[IND_V_V420_FILTER_00 + 3][1] = 0x0007;
            break;

        case VFMT_1080_50i:
            h51APRM[IND_ES_INFO_0C][1] = 0x1b46;
            h51APRM[IND_ES_INFO_0E][1] = 0x3f28;
            h51IPRV[IND_V_FORMAT][1] = 0x1001;
            h51IPRV[IND_GOP_CLK][1] = 0xc001;
            h51IPRV[IND_GOP_STRM][1] = 0x0302;
            h51IPRV[IND_V_V420_FILTER_00][1] = 0x0026;
            h51IPRV[IND_V_V420_FILTER_00 + 1][1] = 0x007d;
            h51IPRV[IND_V_V420_FILTER_00 + 2][1] = 0x0056;
            h51IPRV[IND_V_V420_FILTER_00 + 3][1] = 0x0007;
            break;

        case VFMT_1080_24p:
            if (mode->frRed) {      //if frame rate is 23.98
                h51APRM[IND_ES_INFO_0C][1] = 0x1b61;
		h51IPRV[IND_GOP_CLK][1] = 0xc000;
            } else {                //if frame rate is 24
                h51APRM[IND_ES_INFO_0C][1] = 0x1b62;
		h51IPRV[IND_GOP_CLK][1] = 0xc001;
            }
            h51APRM[IND_ES_INFO_0E][1] = 0x3f28;
            h51IPRV[IND_V_FORMAT][1] = 0x1422;
            h51IPRV[IND_GOP_STRM][1] = 0x0302;
            h51IPRV[IND_V_V420_FILTER_00][1] = 0x0010;
            h51IPRV[IND_V_V420_FILTER_00 + 1][1] = 0x0070;
            h51IPRV[IND_V_V420_FILTER_00 + 2][1] = 0x0070;
            h51IPRV[IND_V_V420_FILTER_00 + 3][1] = 0x0010;
            break;

        case VFMT_720_60p:
            h51APRM[IND_ES_INFO_0C][1] = 0x1b57;
            h51APRM[IND_ES_INFO_0E][1] = 0x3f28;
            h51IPRV[IND_V_FORMAT][1] = 0x1502;
	    if (mode->frRed) {
		h51IPRV[IND_GOP_CLK][1] = 0xc000;
	    } else {
		h51IPRV[IND_GOP_CLK][1] = 0xc001;
	    }
	    h51IPRV[IND_GOP_STRM][1] = 0x0000;
            h51IPRV[IND_V_V420_FILTER_00][1] = 0x0010;
            h51IPRV[IND_V_V420_FILTER_00 + 1][1] = 0x0070;
            h51IPRV[IND_V_V420_FILTER_00 + 2][1] = 0x0070;
            h51IPRV[IND_V_V420_FILTER_00 + 3][1] = 0x0010;
            break;

        case VFMT_720_50p:
            h51APRM[IND_ES_INFO_0C][1] = 0x1b56;
            h51APRM[IND_ES_INFO_0E][1] = 0x3f28;
            h51IPRV[IND_V_FORMAT][1] = 0x1503;
            h51IPRV[IND_GOP_CLK][1] = 0xc001;
            h51IPRV[IND_GOP_STRM][1] = 0x0000;
            h51IPRV[IND_V_V420_FILTER_00][1] = 0x0010;
            h51IPRV[IND_V_V420_FILTER_00 + 1][1] = 0x0070;
            h51IPRV[IND_V_V420_FILTER_00 + 2][1] = 0x0070;
            h51IPRV[IND_V_V420_FILTER_00 + 3][1] = 0x0010;
            break;

        case VFMT_480_60i:
            h51APRM[IND_ES_INFO_0C][1] = 0x1b17;
            h51APRM[IND_ES_INFO_0E][1] = 0x2f28;
            h51IPRV[IND_V_FORMAT][1] = 0x1004;
	    if (mode->frRed) {
		h51IPRV[IND_GOP_CLK][1] = 0xc000;
	    } else {
		h51IPRV[IND_GOP_CLK][1] = 0xc001;
	    }
	    h51IPRV[IND_GOP_STRM][1] = 0x0000;
            h51IPRV[IND_V_V420_FILTER_00][1] = 0x0026;
            h51IPRV[IND_V_V420_FILTER_00 + 1][1] = 0x007d;
            h51IPRV[IND_V_V420_FILTER_00 + 2][1] = 0x0056;
            h51IPRV[IND_V_V420_FILTER_00 + 3][1] = 0x0007;
            break;


        case VFMT_576_50i:
            h51APRM[IND_ES_INFO_0C][1] = 0x1b26;
            h51APRM[IND_ES_INFO_0E][1] = 0x2f28;
            h51IPRV[IND_V_FORMAT][1] = 0x1005;
            h51IPRV[IND_GOP_CLK][1] = 0xc001;
            h51IPRV[IND_GOP_STRM][1] = 0x0000;
            h51IPRV[IND_V_V420_FILTER_00][1] = 0x0026;
            h51IPRV[IND_V_V420_FILTER_00 + 1][1] = 0x007d;
            h51IPRV[IND_V_V420_FILTER_00 + 2][1] = 0x0056;
            h51IPRV[IND_V_V420_FILTER_00 + 3][1] = 0x0007;
            break;

        default:
            return 1;
    }

    switch (mode->aMode) {
        case AMODE_MP1L2:
            h51APRM[IND_ES_INFO_18][1] = 0xff03;
            h51APRM[IND_ES_INFO_1C][1] = 0xf000;
            h51IPRM[IND_A_SID][1] = 0x00c0;
            break;

        case AMODE_AC3:
            h51APRM[IND_ES_INFO_18][1] = 0xff06;
            h51APRM[IND_ES_INFO_1C][1] = 0xf000;
            h51IPRM[IND_A_SID][1] = 0x00bd;
            break;

        case AMODE_LPCM:
            h51APRM[IND_ES_INFO_18][1] = 0xff80;
            h51APRM[IND_ES_INFO_1C][1] = 0xf00a;
            h51APRM[IND_ES_INFO_1E][1] = 0x0508;
            h51APRM[IND_ES_INFO_1E + 1][1] = 0x4844;
            h51APRM[IND_ES_INFO_1E + 2][1] = 0x4d56;
            h51APRM[IND_ES_INFO_1E + 3][1] = 0xff80;
            h51IPRM[IND_A_SID][1] = 0x00bd;
            break;

        case AMODE_AAC:
            h51APRM[IND_ES_INFO_18][1] = 0xff0f;
            h51APRM[IND_ES_INFO_1C][1] = 0xf000;
            h51IPRM[IND_A_SID][1] = 0x00c0;
            break;

        default:
            return 2;
    }

    h51IPRA[IND_A_BITRATE][1] = mode->aBitrate;        //for now set to 192 kbps

    h51IPRM[IND_MUX_RATE][1] = mr;

    h51IPRV[IND_V_RATE_MODE][1] = 0x0030 | (mode->vbr ? 1 : 0);

    h51IPRV[IND_V_BITRATE][1] = mode->vBitrate;
    h51IPRV[IND_AVE_V_BITRATE][1] = mode->vbr ? mode->vBitrate : 0;
    // App note 0.1.  V_MAX_BITRATE should be same as bitrate for CBR.
    h51IPRV[IND_MAX_V_BITRATE][1] = mode->vbr ? (7 * mode->vBitrate) / 4 : mode->vBitrate;
    return 0;
}

