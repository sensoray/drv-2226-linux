//  Used for overlays.
// Copyright Sensoray 2009-2014
#ifndef _FPGA2226_H
#define _FPGA2226_H

//////////////////////////////////////////////////////////////////////////
//  FPGA REGISTERS
//////////////////////////////////////////////////////////////////////////

// Direct access registers -------------------------------

  // Video Control and Status 
#define  S2226_REG_SDICLKSEL    (0x000<<1)
#define  S2226_REG_SDIIN        (0x001<<1)
#define  S2226_REG_SDIOUT       (0x002<<1)
#define  S2226_REG_VIDCFG       (0x003<<1)
#define  S2226_REG_VIDSTAT      (0x004<<1)
#define  S2226_REG_VINSEL       (0x005<<1)
#define  S2226_REG_AUDCFG       (0x006<<1)
#define  S2226_REG_RESETCNT     (0x007<<1)

// Reset and Interrupt status/control
#define  S2226_REG_RESETCTRL    (0x010<<1)
#define  S2226_REG_INTENAB      (0x011<<1)
#define  S2226_REG_INTSTAT      (0x012<<1)

// Miscellaneous.
#define S2226_REG_LEDDATA       (0x020<<1)    // r/w  Led control signals
#define S2226_REG_BOARDID       (0x021<<1)    // r   [15..11] Indicates board rev, [10..0] Indicates FPGA rev
#define S2226_REG_DEBUG         (0x022<<1)    // w    Misc
#define S2226_REG_GPIO          (0x023<<1)    // r/w  General-purpose I/O
#define S2226_REG_GPREG         (0x024<<1)    // r/w  Test register for datapath integrety test
    
// Stream FIFO (Between Fujitsu H51 codec and ARM cpu). 
// FIFO capacity is 1KB, accessed as 512 16-bit words.
#define S2226_REG_FIFO_MODE     (0x030<<1)
#define S2226_REG_FIFO_LOWATR   (0x031<<1)
#define S2226_REG_FIFO_HIWATR   (0x032<<1)
#define S2226_REG_FIFO_STATUS   (0x033<<1)
#define S2226_REG_FIFO_PKTSZ    (0x034<<1)
#define S2226_REG_FIFO_TMSTMP0  (0x035<<1)
#define S2226_REG_FIFO_TMSTMP1  (0x036<<1)
#define S2226_REG_FIFO_RDDATA   (0x800<<1) // thru BFF
#define S2226_REG_FIFO_WRDATA   (0xC00<<1) // thru FFF

// Video offset and gain.
#define S2226_REG_CONTRAST      (0x044<<1)    // w    contrast
#define S2226_REG_BRIGHTNESS    (0x045<<1)    // w    brightness
#define S2226_REG_CRSAT         (0x046<<1)    // w    red saturation
#define S2226_REG_CBSAT         (0x047<<1)    // w    blue saturation

// Overlay window configuration (indirect access via index register).
#define S2226_REG_WCDATA        (0x050<<1)    // w    Write Data to WindowConfig[WCINDEX]
#define S2226_REG_WCINDEX       (0x051<<1)    // w    Index/Address into Window Configuration Registers

// Overlay memory.
#define S2226_REG_OVDATA        (0x052<<1)    // w    overlay data, auto-inc mar
#define S2226_REG_OVACLR        (0x053<<1)    // w    reset mar to zero

#define S2226_REG_OVALSB        (0x054<<1)    // w    mar LSB
#define S2226_REG_OVAMSB        (0x055<<1)    // w    mar MSB
#define S2226_REG_OVAUPD        (0x056<<1)    // w    Set LSB to update the overlay
#define S2226_REG_OVAMSB2       (0x057<<1)    // w    V.1.3.5+ Top 6 bits of write address
#define S2226_REG_OVASTAT       (0x058<<1)    // r    Read bank bits. Overlay is being read from (B1) and written to (B0)

// Snapshot control
#define S2226_REG_FREEZE        (0x060<<1)    // w    Lock output image until cleared
#define S2226_REG_SNAPREQ       (0x061<<1)    // w    Transfer image to host
#define S2226_REG_SNAPACK       (0x062<<1)    // r    Image data is valid (In response to req)
#define S2226_REG_SNAPDAT       (0x063<<1)    // r    Image data. Consecutive access read LSW and MSB of captured pixes. First access reads CrCb
                                              //      SNAPSTAT(0) Toggles each time SNAPDAT read. 
                                              //        When SNAPSTAT(0)=0 - MSB=Cr(7..0), LSB=Cb(7..0) will be read on next SNAPDAT access.
                                              //        When SNAPSTAT(0)=1 - Y(7..0) will be read on LSB of next SNAPDAT access.
#define S2226_REG_SNAPSTAT      (0x064<<1)    // r    Reset image data sequencer, Returns SNAPDAT read type

// video to D1 scalar(MPEG) reset must be active when registers written
#define S2226_REG_MPEGHORZ0     (0x80<<1)    // w    source width LSB
#define S2226_REG_MPEGHORZ1     (0x81<<1)    // w    source width MSB
#define S2226_REG_MPEGHORZ2     (0x82<<1)    // w    scaled width LSB
#define S2226_REG_MPEGHORZ3     (0x83<<1)    // w    source width MSB
#define S2226_REG_MPEGHORZ4     (0x84<<1)    // w    131072 / ScaledWidth
#define S2226_REG_MPEGHORZ5     (0x85<<1)    // w    131072 / ScaledWidth
#define S2226_REG_MPEGHORZ6     (0x86<<1)    // w    (int) (source width / scaled width)
#define S2226_REG_MPEGHORZ7     (0x87<<1)    // w    (source width % scaled width)
#define S2226_REG_MPEGHORZ8     (0x88<<1)    // w    (source width % scaled width)
#define S2226_REG_MPEGHORZ9     (0x89<<1)    // w    start active scaled width
#define S2226_REG_MPEGHORZ10    (0x8a<<1)    // w    start active scaled width
#define S2226_REG_MPEGHORZ11    (0x8b<<1)    // w    end active scaled width
#define S2226_REG_MPEGHORZ12    (0x8c<<1)    // w    end active scaled width

#define S2226_REG_MPEGVERT0     (0x90<<1)    // w    source height
#define S2226_REG_MPEGVERT1     (0x91<<1)    // w    source height
#define S2226_REG_MPEGVERT2     (0x92<<1)    // w    scaled height
#define S2226_REG_MPEGVERT3     (0x93<<1)    // w    scaled height
#define S2226_REG_MPEGVERT4     (0x94<<1)    // w    65536 / scaledheight
#define S2226_REG_MPEGVERT5     (0x95<<1)    // w    65536 / scaledheight
#define S2226_REG_MPEGVERT6     (0x96<<1)    // w    (int) (sourceheight / scaled height)
#define S2226_REG_MPEGVERT7     (0x97<<1)    // w    sourceheight % scaled height
#define S2226_REG_MPEGVERT8     (0x98<<1)    // w    sourceheight % scaled height

#define S2226_REG_PLLFLAGS      (0xE6<<1)    // r    [15..6]=errors(9 downto 0), 0, MEMCHECK_DONE, PLLSTATUS[3..0]

// Bits in S2226_REG_PLLFLAGS
#define S2226_BIT_STREAM_SAV_N  0x8000
#define S2226_BIT_STREAM_ERROR  0x4000
#define S2226_BIT_STREAM_DAV_N  0x2000
#define S2226_BIT_SNAP_ERROR1   0x1000
#define S2226_BIT_SNAP_ERROR2   0x0800
#define S2226_BIT_SNAP_DAV_N    0x0400
#define S2226_BIT_SOURCESAV_N   0x0080
#define S2226_BIT_MEMCHECK_DONE 0x0010

// Bits in S2226_REG_DEBUG
#define S2226_BIT_VID_SWAP      0x8000 // Switch MPEG out and Encoder out for debug purposes.
#define S2226_BIT_BYTE_SWAP     0x4000 // Switch MPEG out MSB/LSB for debug purposes.
#define S2226_BIT_RESET_ERR     0x0001 // Clear upper 10-bits (error bits) in C_ADDR_PLLSTAT register

//////////////////////////////////////////////////////////////////////////
// Indirect access registers
//////////////////////////////////////////////////////////////////////////

// These are write-only registers, accessed indirectly via S2226_REG_WCDATA and S2226_REG_WCINDEX.

// Window boundary register base addresses (Indexed address space)
// Add (8*WINDOW) to base address for a specific window, where WINDOW is in range 0:7.
#define S2226_REG_LBLSB         0x00    // Window beginning line LSB
#define S2226_REG_LBMSB         0x01    // Window beginning line MSB
#define S2226_REG_LELSB         0x02    // Window ending line LSB
#define S2226_REG_LEMSB         0x03    // Window ending line MSB
#define S2226_REG_CBLSB         0x04    // Window beginning column LSB
#define S2226_REG_CBMSB         0x05    // Window beginning column MSB
#define S2226_REG_CELSB         0x06    // Window ending column LSB
#define S2226_REG_CEMSB         0x07    // Window ending column MSB

// Black window boundary register base addresses (Indexed address space)
// Add (8*WINDOW) to base address for a specific window, where WINDOW is in range 0:1.
#define S2226_REG_BWLBLSB       0x80    // Black window beginning line LSB
#define S2226_REG_BWLBMSB       0x81    // Black window beginning line MSB
#define S2226_REG_BWLELSB       0x82    // Black window ending line LSB
#define S2226_REG_BWLEMSB       0x83    // Black window ending line MSB
#define S2226_REG_BWCBLSB       0x84    // Black window beginning column LSB
#define S2226_REG_BWCBMSB       0x85    // Black window beginning column MSB
#define S2226_REG_BWCELSB       0x86    // Black window ending column LSB
#define S2226_REG_BWCEMSB       0x87    // Black window ending column MSB

// Black window color (Indexed address space)
#define S2226_REG_BWCOLOR0      0xC0    // Black window color LSB
#define S2226_REG_BWCOLOR1      0xC1    // Black window color MSB

// Registers in SDI IC
#define ADDRESS_SDIIN_FORMAT1   0x0C
#define ADDRESS_SDIIN_TEST      0x0D
#define ADDRESS_SDIIN_VIDCTL0   0x55

#endif // _FPGA2226_H
