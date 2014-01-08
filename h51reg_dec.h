
unsigned int G_h51reg_dec[][2] = {
 {0x000800c2, 0x4100},  // (TBD END_PTS_VALID_FLAG (bit 10)
 {0x000800c4, 0x0000},  // START_PTS_H
 {0x000800c6, 0x1194},  // START_PTS_L
 {0x000800c8, 0x7fff},  // END_PTS_H 
 {0x000800ca, 0xffff},  // END_PTS_L
 {0x00001000, 0x0000},  // should be 0
 {0x00001002, 0x0080},  // STCLK Use bit 7 : 1-input (other settings for output)
 {0x00001008, 0x0000},  // [1:0] TS_PACKET_SIZE (0-188, 1-192)
 // 100A, encode only (PROGRAM_NUM)
 // 100C, encode only (INITIAL_STC_H)
 // 100E, encode only (INITIAL_STC_L)
 // 1010, encode only (more STC values)
 // 1012, encode only (INITIAL_TIME_STAMP_H)
 // 1014, encode only (INITIAL_TIME_STAMP_L)
 {0x00001016, 0x1011}, // V_PID
 {0x00001018, 0x1100}, // A_PID
 {0x0000101a, 0x0100}, // PMT_PID
 {0x0000101c, 0x001f}, // SIT_PID
 {0x0000101e, 0x1001}, // PCR_PID
 // 1020 V_SID encode only
 // 1022 A_SID encode only
 {0x00001024, 0x0001}, //vout_sync_mode

 // 1026 - 1120 (reserved and encode only)
 {0x00001122, 0x0000}, // DEC_SMES_MASK
 // 1124-112e reserved (do not write anything)
 {0x00001130, 0x0000},  //PES_ALIGN[0]
 // 1132- 11fe reserved (do not write anything)
 {0x0000140a, 0x1000}, // {0x0000140a, 0x1000 HD, 0x1004 SD},  // format dependent
 {0x0000140e, 0xc000}, // format dependent (same for HD and SD.  different for 24fps or 1440)
 {0x00001410, 0x0101}, //errorRatePpic[15:8], errorRateIpic[7:0]
 {0x00001412, 0x0101}, //errorRateRefpic[15:8], errorRateBpic[7:0]
 {0x00001416, 0x0000}, // [0] V_IP_DETECT
 {0x00001418, 0x0000}, // [8]VOUT_SWAP, [0]VIN_SWAP
 {0x0000141c, 0x0000}, // V_ERROR_LEVEL_TH_H
 {0x0000141e, 0x0000}, // V_ERROR_LEVEL_TH_L

 {0x00001430, 0x0002}, // V_GOP_STRUCT[15:8]enc, V_STRM_FORMAT[7:0] format dependent (0 SD) (2 HD)

 {0x00001434, 0x0000}, //DPB_SIZE[7:0] = 0

 {0x00001460, 0x0007}, // 720p specific filter stuff
 {0x00001462, 0x0056},
 {0x00001464, 0x007d},
 {0x00001466, 0x0026},
 {0x00001468, 0x0000},
 {0x0000146a, 0x0000},
 {0x0000146c, 0x0000},
 {0x0000146e, 0x0000},
 // interlaced settings (V_V420_FILTER_*)V_
 {0x00001470, 0x00fc},

 {0x00001472, 0x0024},
 {0x00001474, 0x00f8},
 {0x00001476, 0x00e8},
 {0x00001478, 0x00da},
 {0x0000147a, 0x00c6},
 {0x0000147c, 0x0076},
 {0x0000147e, 0x00ea},

 {0x00001802, 0x0000}, // AV_RESYNC_THRESHOLD[13:2] A_SAMPLE[1:0]
// I2S (0x4000 bitmask is AOUT_BCLK_HL should be on falling edge)
//      0x0200 bitmask is AOUT_LRCLK_SEL needs delay of 2 through FPGA

 // Note: ARM firmware 0x4c will automatically correct the
 //       master/slave setting (bit 0 in 1808 below)
 //       based on ATTR_AUDH51_MASTER.
 //       This is best done in the ARM because the FPGA direction
 //       needs set and to prevent bus contention.
 //       In short, do not worry about the lowest bit setting.
 //       Use ATTR_AUDH51_MASTER and NEVER set the AIC33 registers 8 or 9
 {0x00001808, 0x42c1}, // see mnual
 {0x0000180a, 0x0000}, //AOUT_OFFSET
 {0x0000180c, 0x0000}, //ASOUT_OFFSET
 {0x0000180e, 0x0000}, //MUTE_TIME[9:8]
 {0x000018c0, 0x0000},  //AERR_AUTO_MUTE[15], AERR_MSK_LEN[3:0]
};
