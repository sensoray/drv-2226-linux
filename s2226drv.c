/*
 * 2226 USB Linux driver
 *
 * Copyright (C) 2009-2014 Sensoray Company Inc.
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License as
 *      published by the Free Software Foundation, version 2.
 *
 * @author: D.A., P.E.
 * @date: 2009-2014
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/mman.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kref.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/videodev2.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-common.h>
#include <media/videobuf-vmalloc.h>
#include <media/v4l2-ioctl.h>


// update with each new version for easy detection of driver
// version on embedded products (2420 for instance)
#define S2226_DRIVER_VERSION_STRING "V1.0.8: 03/18/2011"

#include "s2226mod.h"
#include "s2226ioctl.h"
#include "h51reg.h"
#include "h51reg_dec.h"
#include "dh2226.h"

#define S2226_SETINPUT_TO 20000
#define S2226_FLASHERASE_TO 60000
#define S2226_FLASHWRITE_TO 9000
#define S2226_FLASHREAD_TO S2226_FLASHWRITE_TO
#define S2226_SETMODE_TO 9000
#define S2226_BULKMSG_TO  5000
#define S2226_PRIMEFX2_TO 500
#define S2226_CTRLMSG_TO  500
#define S2226_DEF_VBITRATE 2000
#define S2226_MIN_VBITRATE 1000
#define S2226_MAX_VBITRATE 19000
#define S2226_DEF_ABITRATE 256
#define S2226_CONTEXT_USBDEV 0  // for /dev/s2226vX
#define S2226_CONTEXT_V4L    1  // for /dev/videoX

#define DRIVERNAME "s2226"

#define USB_S2226_VENDOR_ID   0x1943
#define USB_S2226_PRODUCT_ID  0x2226

#define MAX_ENDPOINTS 4
#define INDEX_EP_H264 0    // video in, video out.  endpoint index
#define INDEX_EP_CONTROL 1 // control messages
#define INDEX_EP_RESP    2 // responses
#define INDEX_EP_RAW     3 // raw pipe (USB FW 0x20+ only)



#define s2226_mutex_init     mutex_init
#define s2226_mutex_lock     mutex_lock
#define s2226_mutex_lock_interruptible mutex_lock_interruptible
#define s2226_mutex_unlock   mutex_unlock


/* table of devices that work with this driver */
static struct usb_device_id s2226_table [] = {
	{ USB_DEVICE(USB_S2226_VENDOR_ID, USB_S2226_PRODUCT_ID) },
	{ }                                     /* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, s2226_table);

/* Get a minor range for your devices from the usb maintainer */
#define USB_S2226_MINOR_BASE    0

static int debug = 0;
static int filter_mode = 0; // 0 = autodetect, 1=fixed
int *s2226_debug=&debug;
int *s2226_filter_mode=&filter_mode;


static int majorv = 126;
static LIST_HEAD(s2226_devlist);
static unsigned int vid_limit = 16;	/* Video memory limit, in Mb */
static int video_nr = -1;	/* /dev/videoN, -1 for autodetect */


module_param(debug,int,0);
MODULE_PARM_DESC(debug, "Debug level(0-4)");
module_param(filter_mode,int,0);
MODULE_PARM_DESC(filter_mode, "filter mode(diagnostic only)");

/*
 * set driver parameter audiomode to 0 if
 * you are configuring the audio settings
 * through userspace and don't want automatic
 * configuration after each set input.
 */
#define S2226_AUDIOMODE_MANUAL      0
#define S2226_AUDIOMODE_AUTO_2420   1
#define S2226_AUDIOMODE_AUTO_2226S  2
static int audiomode = S2226_AUDIOMODE_AUTO_2226S;
module_param(audiomode,int,0);
MODULE_PARM_DESC(audiomode, "Audiomode (0-manual, 1-auto)");

/* internal use only.  Will void warranty if mis-used */
static int mfgmode = 1;
module_param(mfgmode,int,0);
MODULE_PARM_DESC(mfgmode, "Mfg mode (0-off default) internal use.  do not change");


module_param(majorv,int,0);
MODULE_PARM_DESC(majorv, "major for video devname");
module_param(vid_limit, int, 0644);
MODULE_PARM_DESC(vid_limit, "video memory limit(Mb)");
module_param(video_nr, int, 0644);
MODULE_PARM_DESC(video_nr, "start video minor(-1 default autodetect)");


#define dprintk(level,fmt, arg...)					\
	do {								\
		if (*s2226_debug >= (level)) {				\
			printk(KERN_DEBUG "s2226: " fmt, ##arg);	\
		}							\
	} while (0)


#define to_s2226_dev(d) container_of(d, struct s2226_dev, kref)


static struct usb_driver s2226_driver;
static struct mutex  s2226_devices_lock;


#define MAX_CHANNELS 5

/*
  ring buffer: 
*/
#define S2226_RB_PKT_SIZE 4096
#define WR_URBS		  8
#define RD_URBS		  8

struct s2226_urb {
	struct s2226_dev *dev;
	struct urb *urb;
	int ready;
	int context;
	void *buffer;
};

struct s2226_dev;

struct s2226_dmaqueue {
	struct list_head active;
	struct s2226_dev *dev;
};

#define AUDIOROUTE_LINE1L S2226_AUDIOROUTE_LINE1L
#define AUDIOROUTE_LINE2L S2226_AUDIOROUTE_LINE2L 
#define AUDIOROUTE_LINE1L_BYPASS S2226_AUDIOROUTE_LINE1L_BYPASS 
#define AUDIOROUTE_LINE2L_BYPASS S2226_AUDIOROUTE_LINE2L_BYPASS 
static int SetAudioIn(struct s2226_dev *dev);
static int SetAudioRoute(struct s2226_dev *dev, int route);
static int SetAudioOut(struct s2226_dev *dev);
static int SetAudioLeftGain(struct s2226_dev *dev, int gain);
static int SetAudioRightGain(struct s2226_dev *dev, int gain);
static int SetAudioRightAGC(struct s2226_dev *dev, int bOn, int gain);
static int SetAudioLeftAGC(struct s2226_dev *dev, int bOn, int gain);
static int SetAudioBalR(struct s2226_dev *dev, int bBal);
static int SetAudioBalL(struct s2226_dev *dev, int bBal);
struct s2226_audio {
	int bAGC_R; /* if AGC on or off*/
	int bAGC_L; /* if AGC on or off*/
	int in_balR; /* if balanced or not */
	int in_balL; /* if balanced or not */
	int iAGCRightGain;
	int iAGCLeftGain;
	int iRightGain;
	int iLeftGain;
	// audio output gains and balance settings
	int iVolGainR;
	int iVolGainL;
	int bVolMuteR;
	int bVolMuteL;
	int iMonoGain;
	int iHpGainR;
	int iHpGainL;
	int iStereoGainR;
	int iStereoGainL;
	int bStereoBal;
	int bHpBal;
	int bMonoBal;
	int bStereoMuteR;
	int bStereoMuteL;
	int bHpMuteR;
	int bHpMuteL;
	int bMonoMute;
	int iRoute;
};

/* 2226 device */
struct s2226_dev {
	int			users;
	int			debug;
	struct mutex ioctl_lock;
	struct mutex            audlock;
	struct usb_device	*udev;		// our USB device
	struct usb_interface	*interface;	// usb interface
	struct kref		kref;
	int			h51_state;
	int			persona;
	void			*interrupt_buffer;
	void			*control_buffer;
	struct urb		*interrupt_urb;
	struct urb		*control_urb;
	struct s2226_urb	write_vid[WR_URBS];
	struct s2226_urb	read_vid[RD_URBS];
	int			read_vid_head;
	int			write_vid_ready;
	int			interrupt_ready;
	int			control_ready;
	wait_queue_head_t       read_vid_wq;
	wait_queue_head_t       write_vid_wq;
	struct MODE2226		h51_mode;
	int			cur_input;
	int                     cur_audiompeg; // current audio input (for mpeg)
	int                     is_decode; // set to decode input
	int                     input_set;
	unsigned                id; // message id
	struct mutex            cmdlock;
	int                     closed_gop;
	int                     dec_pes_pkts;
	int                     fpga_ver;
	int                     board_id;
	int                     arm_ver;
	int                     minor;
	int			dpb_size;
	int			gop_struct;
	int			ainoffset;
	int			avresync;
	int                     hue;
	int                     brightness;
	int                     saturation;
	int                     contrast;
	int			resources;
	struct mutex            reslock; // resource lock(V4L)
	int                     v4l_is_pal; // is PAL standard
	int                     v4l_input;
	spinlock_t		slock;
	struct video_device     *vdev;
	struct v4l2_device      v4l2_dev;
	struct s2226_dmaqueue   vidq;
	struct list_head        s2226_devlist;
	v4l2_std_id		current_norm;
	struct s2226_audio      aud;
	int h51_reload_required;
	int usb_ver;
	int ep[MAX_ENDPOINTS]; // endpoint address
	int audio_page;
};


#define    S2226_INPUT_COMPOSITE_0     0
#define    S2226_INPUT_SVIDEO_0        1
#define    S2226_INPUT_COMPOSITE_1     2
#define    S2226_INPUT_SVIDEO_1        3
#define    S2226_INPUT_SD_COLORBARS    4
#define    S2226_INPUT_720P_COLORBARS  5
#define    S2226_INPUT_1080I_COLORBARS 6
#define    S2226_INPUT_SDI_SD          7
#define    S2226_INPUT_SDI_720P_5994   8 
#define    S2226_INPUT_SDI_720P_60     9
#define    S2226_INPUT_SDI_1080I_50    10  
#define    S2226_INPUT_SDI_1080I_5994  11
#define    S2226_INPUT_SDI_1080I_60    12
#define    S2226_INPUT_SDI_720P_50     13
#define    S2226_INPUT_SDI_720P_24     14
#define    S2226_INPUT_SDI_720P_2398   15
#define    S2226_INPUT_SDI_1080P_24    16
#define    S2226_INPUT_SDI_1080P_2398  17
#define    S2226_INPUT_MAX             18

// MPEG audio source
#define    S2226_AUDIOINPUT_LINE       0
#define    S2226_AUDIOINPUT_TONE       1 // test tone
#define    S2226_AUDIOINPUT_SDI        2 // SDI-IN embedded audio
#define    S2226_AUDIOINPUT_MAX        3

/* maximum size of URB buffers */
#define MAX_USB_SIZE (16*1024)
#define MAX_USB_INT_SIZE 512	// interrupt


#define S2226_BUFFER_SIZE (S2226_RB_PKT_SIZE)
#define S2226_DEF_BUFS    10

/* buffer for one video frame */
struct s2226_buffer {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;
//	const struct s2226_fmt *fmt;
};


struct s2226_fh {
	struct s2226_dev	*dev;
	unsigned int resources;	/* resource management for device open */
	int type;
	struct videobuf_queue	vb_vidq;
};

/* function prototypes (forwards)*/
#define RES_STREAM  1
#define RES_OVERLAY 2
#define RES_ALL  (RES_STREAM | RES_OVERLAY)
static int res_get(struct s2226_dev *dev, struct s2226_fh *fh, int res);
static void res_free(struct s2226_dev *dev, struct s2226_fh *fh, int res);
static int res_check(struct s2226_fh *fh);
static int res_locked(struct s2226_dev *dev, struct s2226_fh *fh);
void s2226_sleep( int ms);
static int write_control_endpoint(struct s2226_dev *dev, int timeout);
static int read_interrupt_endpoint(struct s2226_dev *dev, int timeout);
static int s2226_get_attr(struct s2226_dev *dev, int attr, int *value);
extern int setH51regs (struct MODE2226 *mode);
extern void loadH51rb(H51_RB_SIZE *rb_size);

static void s2226_read_vid_callback(struct urb *u);


int s2226_vendor_request(void *pdev, unsigned char req,
			 unsigned short idx, unsigned short val,
			 void *pbuf, unsigned int len,
			 int bOut);
static int send_sdi_write(struct s2226_dev *dev, unsigned char addr, unsigned char val, int bIn);
static int send_persona_run(struct s2226_dev *dev, unsigned char persona);
static int send_persona_halt(struct s2226_dev *dev);
static int s2226_flush_in_ep(struct s2226_dev *dev);
static int s2226_set_interface(struct s2226_dev *dev, int cfg, int alt);

// for controlling which firmware is loaded.  see comments for
// S2226_IOC_SET_BASEFW and S2226_IOC_SET_NEWFW in ioctl file.
static int s2226_fx2sam(struct s2226_dev *dev, int bNewFw);

// resets the ARM CPU to bring up all board devices(h51, FPGA, ARM)
// except the FX2 usb chip in a fresh state
static int s2226_reset_board(struct s2226_dev *dev);

static int s2226_default_params(struct s2226_dev *dev);
static int s2226_probe_v4l(struct s2226_dev *dev);
static void s2226_exit_v4l(struct s2226_dev *dev);
static int s2226_got_data(struct s2226_dev *dev, int urb_idx);
static struct videobuf_queue_ops s2226_video_qops;

static unsigned char compute_checksum(unsigned char *pdata, int len)
{
	int i;
	unsigned char chksum = 0;
	for (i = 0; i < len; i++) {
		chksum += pdata[i];
	}
	return chksum;
}

static void s2226_display_err(unsigned char err)
{
	switch (err) {
	case INTRESULT_CHECKSUM:
		dprintk(0, "checksum err\n");
		break;
	case INTRESULT_MALFORMED:
		dprintk(0, "malformed command err\n");
		break;
	case INTRESULT_BADPARAM:
		dprintk(0, "bad parameter err\n");
		break;
	case INTRESULT_BADOPCODE:
		dprintk(0, "bad opcode\n");
		break;
	case INTRESULT_BADSIZE:
		dprintk(0, "bad size\n");
		break;
	case INTRESULT_H51_NOTREADY:
		dprintk(0, "h51 not ready\n");
		break;
	case INTRESULT_H51_TIMEOUT:
		dprintk(0, "h51 timeout\n");
		break;
	case INTRESULT_H51_ABNORMAL:
		dprintk(0, "h51 abnormal\n");
		break;
	case INTRESULT_H51_ERR_UNKNOWN:
		dprintk(0, "h51 err unknown\n");
		break;
	case INTRESULT_FPGA_NOTREADY:
		dprintk(0, "fpga not ready\n");
		break;
	case INTRESULT_FPGA_TIMEOUT:
		dprintk(0, "fpga timeout\n");
		break;
	case INTRESULT_FPGA_CONFIGERR:
		dprintk(0, "fpga config err\n");
		break;
	case INTRESULT_FPGA_ONLINEERR:
		dprintk(0, "fpga online err\n");
		break;
	case INTRESULT_FLASHERR:
		dprintk(0, "flash err\n");
		break;
	case INTRESULT_NOFLASHIMAGE:
		dprintk(0, "no flash image\n");
		break;
	case INTRESULT_STREAMNOTIFY_FAIL:
		dprintk(0, "stream notify fail\n");
		break;
	case INTRESULT_NO_MCLK:
		dprintk(0, "no input or no mclk\n");
		break;
	default:
		dprintk(0, "unknown err\n");
		break;
	}
	return;
}

// this function must be called with cmdlock locked
static int s2226_send_msg_noresp(struct s2226_dev *dev)
{
	int rc;
	rc = write_control_endpoint(dev, S2226_BULKMSG_TO);
	return rc;
}

// this function must be called with cmdlock locked
// timeout is time to wait for response in ms
static int s2226_send_msg(struct s2226_dev *dev, int timeout)
{
	int rc;
	unsigned char *buf;
	unsigned char cmd;
	unsigned char id;
	buf = (unsigned char *) dev->control_buffer;
	cmd = buf[1];
	id = buf[2];
	rc = write_control_endpoint(dev, S2226_BULKMSG_TO);
	if (rc < 0)
		return rc;
	rc = read_interrupt_endpoint(dev, timeout);
	if (rc < 0) {
		return rc;
	}
	// normal reply structure
	// buf[0]: INTYPE_CMDREPLY==1
	// buf[1]: opcode
	// buf[2]: echo ID
	// buf[3..]: response
	buf = (unsigned char *) dev->interrupt_buffer;
	if (buf[0] != INTTYPE_CMDREPLY) {
		rc = read_interrupt_endpoint(dev, timeout);
		if (rc < 0) {
			return rc;
		}
	}
	if (buf[0] != INTTYPE_CMDREPLY) {
		return -EAGAIN;
	}
	// some error replies
	// buf[0]: INTYPE_CMDREPLY==1
	// buf[1]: 0  (same as NOP opcode, so check for NOP below)
	// buf[2]: reason
	if (buf[1] == 0 && buf[0] == 1 && cmd != 0) {
		s2226_display_err(buf[2]);
		return (S2226_ERR_BASE - buf[2]);
	}
	if ((buf[1] != cmd) || (buf[2] != id)) {
		printk(KERN_INFO "command mismatch, re-reading: %x:%x, %x:%x\n",
		       buf[1], cmd, buf[2], id);
		printk(KERN_INFO "%x %x %x %x %x %x\n",
		       buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		// wait for full timeout.  if mismatch, endpoint was
		// already populated
		rc = read_interrupt_endpoint(dev, timeout);
		if (rc < 0) {
			printk(KERN_INFO "%s failed retry: %d\n", __func__, rc);
			return rc;
		}
	}
	if ((buf[1] != cmd) || (buf[2] != id)) {
		printk(KERN_INFO "command mismatch\n");
		return -EAGAIN;
	}
	if (buf[3] != 0) {
		s2226_display_err(buf[3]);
		return (S2226_ERR_BASE - buf[3]);
	}
	// rc must be 0 to get here
	return 0;
}

// assumes i counter initialized to 0
#define fill_cmd_buf(buf, i, cmd, id)		\
	buf[i++] = 0;				\
	buf[i++] = cmd;				\
	buf[i++] = id;				\
	buf[i++] = 0;				
	


#define fill_cmd_buf_val32(buf, i, _val_)	\
	buf[i++] = (_val_ >> 24) & 0xff;	\
	buf[i++] = (_val_ >> 16) & 0xff;	\
	buf[i++] = (_val_ >> 8) & 0xff;		\
	buf[i++] = (_val_ & 0xff);			


#define fill_cmd_buf_val16(buf, i, _val_)	\
	buf[i++] = (_val_ >> 8) & 0xff;		\
	buf[i++] = (_val_ & 0xff);			



// form sdi write command
// addr is sdi address to write to
// val is value
// bIn is 1 if writing to the SDI input chip, 0 if writing to the output chip
static int send_sdi_write(struct s2226_dev *dev, unsigned char addr, unsigned char val, int bIn)
{
	int i=0;
	unsigned char *buf= dev->control_buffer;
	unsigned char id;
	int rc;
	s2226_mutex_lock(&dev->cmdlock);    
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = bIn ? DEVID_SDI_IN : DEVID_SDI_OUT;
	buf[i++] = 1; //mode (no address increment, 8 bit values)
	buf[i++] = 0; // values (MSB)
	buf[i++] = 1; // 1 8 bit data field present
	buf[i++] = 0; // MSB of addr(32 bits in command)
	buf[i++] = 0;
	buf[i++] = 0;
	buf[i++] = addr;
	buf[i++] = val;
	buf[0] = (unsigned char) i;
	dprintk(4, "%s: %x:%x\n", __func__, addr, val);
	buf[i++] = compute_checksum(buf,buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}


// form sdi read command
// addr is sdi address to read from
// bIn is 1 if writing to the SDI input chip, 0 if writing to the output chip
static int form_sdi_read(unsigned char *buf, unsigned char addr, int bIn, unsigned char id)
{
	int i = 0;
	fill_cmd_buf(buf, i, HOSTCMD_REGREAD, id);
	buf[i++] = bIn ? DEVID_SDI_IN : DEVID_SDI_OUT;
	buf[i++] = 1; //mode (no address increment, 8 bit values)
	buf[i++] = 0; // values (MSB)
	buf[i++] = 1; // no data present
	buf[i++] = 0; // MSB of addr(32 bits in command)
	buf[i++] = 0;
	buf[i++] = 0;
	buf[i++] = addr;
	buf[0] = (unsigned char) i;
	dprintk(4, "%s: %x\n", __func__, addr);
	buf[i++] = compute_checksum(buf,buf[0]);
	return i;
}

// form FPGA write command
// addr is FPGA address to write to
// val is value
// bIn is 1 if writing to the SDI input chip, 0 if writing to the output chip
static int send_fpga_write(struct s2226_dev *dev, unsigned int addr, unsigned int val)
{
	int i=0;
	unsigned char *buf= dev->control_buffer;
	unsigned char id;
	int rc;
	s2226_mutex_lock(&dev->cmdlock);    
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = DEVID_FPGA;
	buf[i++] = 2; //mode (no address increment, 16 bit values)
	buf[i++] = 0; // values (MSB)
	buf[i++] = 1; // 1 16 bit data field present
	buf[i++] = 0; // MSB of addr(32 bits in command)
	buf[i++] = 0;
	fill_cmd_buf_val16(buf, i, addr);
	fill_cmd_buf_val16(buf, i, val);
	buf[0] = (unsigned char) i;
	dprintk(2, "%s: %x:%x\n", __func__, addr, val);
	buf[i++] = compute_checksum(buf,buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);    
	return rc;
}

#define FPGA_WRITE_NORMAL     0
#define FPGA_WRITE_FAST       1
#define FPGA_WRITE_ADDRDATA   2
// Note: length can not be greater than 256-14(14 for the header size plus 
// checksum) if firmware under 0x44.
// If firmware > 0x44, length can't be greater than 512-14
// type == 0 (normal write)
// type == 1 (fast write)
// type == 2 (addrdata)
static int send_fpga_write_burst(struct s2226_dev *dev, unsigned int addr, unsigned char *data, int len, int type)
{
	int i=0;
	int siz;
	int j;
	unsigned char *buf= dev->control_buffer;
	unsigned char id;
	int rc;
	s2226_mutex_lock(&dev->cmdlock); 
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = DEVID_FPGA;
	buf[i] = 2; //mode (no address increment, 16 bit values)
	if (type == FPGA_WRITE_FAST) 
		buf[i] |= (REG_UNPACK_ENABLE | REG_NORESP_ENABLE);
	else if (type == FPGA_WRITE_ADDRDATA)
		buf[i] |= (REG_ADDRDATA_MODE | REG_NORESP_ENABLE);

	i++;
	buf[i++] = 0; // values (MSB)
	buf[i++] = len / 2;
	buf[i++] = 0; // MSB of addr(32 bits in command)
	buf[i++] = 0;
	fill_cmd_buf_val16(buf, i, addr);
	// values
	for (j = 0; j < len; j++)
		buf[i++] = data[j];
	// Firmware 0x44 and above supports extended commands(larger than 256) use reserved field 
	// for MSB
	buf[0] = (unsigned char) i & 0xff;
	buf[3] = (unsigned char) ((i >> 8) & 0xff);

	siz = i + 1;
	dprintk(2, "%s: %x\n", __func__, addr);
	buf[i++] = compute_checksum(buf, siz - 1);
	
	if (type == FPGA_WRITE_NORMAL)
		rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	else
		rc = s2226_send_msg_noresp(dev);
	
	s2226_mutex_unlock(&dev->cmdlock);    
	return rc;
}



// Note: length can not be greater than 256-14(14 for the header size plus 
// checksum).
static int send_fpga_read_burst(struct s2226_dev *dev, unsigned int addr, unsigned char *data, int len)
{
	int i=0;
	unsigned char *buf= dev->control_buffer;
	unsigned char id;
	int rc;
	if (len > 508)
		return -EINVAL;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGREAD, id);
	buf[i++] = DEVID_FPGA;
	buf[i++] = 2; //mode (no address increment, 16 bit values)
	buf[i++] = 0; // values (MSB)
	buf[i++] = (len / 2);
	buf[i++] = 0; // MSB of addr(32 bits in command)
	buf[i++] = 0;
	fill_cmd_buf_val16(buf, i, addr);
	buf[0] = (unsigned char) i;
	dprintk(2, "%s: %x\n", __func__, addr);
	buf[i++] = compute_checksum(buf,buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);    
	if (rc != 0)
		return rc;
	buf = (unsigned char *) dev->interrupt_buffer;
//	printk("buf[0-3] %x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
//	printk("buf[4-7] %x %x %x %x\n", buf[0+4], buf[1+4], buf[2+4], buf[3+4]);
	memcpy(data, buf+4, len);
	return rc;
}


// form fpga read command
// addr is fpga address to read from
static int form_fpga_read(unsigned char *buf, unsigned int addr, unsigned char id)
{
	int i = 0;
	fill_cmd_buf(buf, i, HOSTCMD_REGREAD, id);
	buf[i++] = DEVID_FPGA;
	buf[i++] = 2; //mode (no address increment, 16 bit values)
	buf[i++] = 0; // values (MSB)
	buf[i++] = 1; // 1 value
	buf[i++] = 0; // MSB of addr(32 bits in command)
	buf[i++] = 0;
	fill_cmd_buf_val16(buf, i, addr);
	buf[0] = (unsigned char) i;
	dprintk(4, "%s: %x\n", __func__, addr);
	buf[i++] = compute_checksum(buf,buf[0]);
	return i;
}

static int send_h51_regwr(struct s2226_dev *dev, unsigned int addr, unsigned int val)
{
	int i=0;
	unsigned char *buf= dev->control_buffer;
	unsigned char id;
	int rc;
	s2226_mutex_lock(&dev->cmdlock);    
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = DEVID_H51; //device ID
	buf[i++] = 2; //mode (value size)
	buf[i++] = 0; // values (MSB)
	buf[i++] = 1; // values (LSB)
	fill_cmd_buf_val32(buf, i, addr);
	fill_cmd_buf_val16(buf, i, val);
	buf[0] = (unsigned char) i;
	dprintk(2, "H51: { 0x%08x,0x%04x},\n", addr, (unsigned int )val& 0xffff);
	buf[i++] = compute_checksum(buf,buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);    
	return rc;
}


static int form_h51_regrd(unsigned char *buf, unsigned int addr, unsigned char id)
{
	int i= 0;
	fill_cmd_buf(buf, i, HOSTCMD_REGREAD, id);
	buf[i++] = DEVID_H51; //device ID
	buf[i++] = 2; //mode (value size)
	buf[i++] = 0; // values (MSB)
	buf[i++] = 1; // values (LSB)
	fill_cmd_buf_val32(buf, i, addr);
	buf[0] = (unsigned char) i;
	//printf("{ 0x%08lx,0x%04lx},\n", addr, (unsigned int )val& 0xffff);
	buf[i++] = compute_checksum(buf,buf[0]);
	return i;
}

static int form_generic_regrd_u8(unsigned char *buf, unsigned int addr, unsigned char id, int devid)
{
	int i= 0;
	fill_cmd_buf(buf, i, HOSTCMD_REGREAD, id);
	buf[i++] = devid; //device ID
	buf[i++] = 1; //mode (value size)
	buf[i++] = 0; // values (MSB)
	buf[i++] = 1; // values (LSB)
	fill_cmd_buf_val32(buf, i, addr);
	buf[0] = (unsigned char) i;
	//printf("{ 0x%08lx,0x%04lx},\n", addr, (unsigned int )val& 0xffff);
	buf[i++] = compute_checksum(buf,buf[0]);
	return i;
}




static int get_reg(struct s2226_dev *dev, int devID,
		   unsigned int addr, unsigned int *val)
{
	unsigned char *buf;
	int rsize;
	s2226_mutex_lock(&dev->cmdlock);
	switch (devID) {
	case DEVID_AUDIO: //readable
	case DEVID_SDI_SPLIT: // may be write-only like the 7121
	case DEVID_VIDDEC: // readable
	case DEVID_VIDENC: //Note: saa7121 is not readable
		form_generic_regrd_u8(dev->control_buffer, addr, dev->id++, devID);
		rsize = 1;
		break;
	case DEVID_H51:
		form_h51_regrd(dev->control_buffer, addr, dev->id++);
		rsize = 2;// 2 bytes
		break;
	case DEVID_FPGA:
		form_fpga_read(dev->control_buffer, addr, dev->id++);
		rsize = 2;// 2 bytes
		break;
	case DEVID_SDI_IN:
		form_sdi_read(dev->control_buffer, addr, 1, dev->id++);
		rsize = 1;// 1 byte
		break;
	case DEVID_SDI_OUT:
		form_sdi_read(dev->control_buffer, addr, 0, dev->id++);
		rsize = 1;// 1 byte
		break;
	default:
		s2226_mutex_unlock(&dev->cmdlock);
		return -EINVAL;
	}
	(void) s2226_send_msg(dev, S2226_BULKMSG_TO);
	buf = (unsigned char *) dev->interrupt_buffer;

	dprintk(9, "buf %x %x %x %x %x %x %x %x\n",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
		buf[6], buf[7]);

	if (buf[3] != 0) {
		s2226_mutex_unlock(&dev->cmdlock);
		return buf[3];
	}

	if (rsize == 1)
		*val = buf[4];
	else if (rsize == 2)
		*val = (buf[4] << 8) + buf[5];
	dprintk(9, "addr %x value %x\n", addr, *val);
	s2226_mutex_unlock(&dev->cmdlock);
	return 0;
}



static int send_h51_boot(struct s2226_dev *dev)
{
	int i=0;
	unsigned char *buf = dev->control_buffer;
	int rc;
	unsigned char id;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	buf[i++] = 0;
	buf[i++] = HOSTCMD_BOOT;
	buf[i++] = id;
	buf[i++] = 0;
	buf[i++] = 0x00; //addr MSB
	buf[i++] = 0x09; //addr
	buf[i++] = 0x00; //addr
	buf[i++] = 0x00; //addr LSB
	buf[i++] = 0x00; //size MSB
	buf[i++] = 0x00;//0x0a; //size
	buf[i++] = 0x00;//0xf0; //size
	buf[i++] = 0x00;//0xf4; //size LSB
	buf[i++] = DEVID_H51;
	buf[i++] = 0;
	buf[0] = (unsigned char) i;
	dprintk(1, "%s\n", __func__);
	buf[i++] = compute_checksum(buf,buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}

static int send_fpga_boot(struct s2226_dev *dev)
{
	int i=0;
	unsigned char *buf = dev->control_buffer;
	int rc;
	unsigned char id;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_BOOT, id);
	buf[i++] = 0x00; //addr MSB
	buf[i++] = 0x01; //addr
	buf[i++] = 0x00; //addr
	buf[i++] = 0x00; //addr LSB
	buf[i++] = 0x00; //size MSB
	buf[i++] = 0x07; //size
	buf[i++] = 0xcb; //size
	buf[i++] = 0x88; //size LSB
	buf[i++] = DEVID_FPGA;
	buf[i++] = 0;
	buf[0] = (unsigned char) i;
	dprintk(4, "%s\n", __func__);
	buf[i++] = compute_checksum(buf,buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}

//static int send_aic33_rd(struct s2226_dev *dev, unsigned char addr, unsigned char *val);
static int s2226_aic33_check(u8 page, u8 reg, u8 val);
static int send_aic33_wr(struct s2226_dev *dev, unsigned char addr, unsigned char val)
{
	int i=0;
	unsigned char *buf = dev->control_buffer;
	int rc;
	unsigned char id;
//	unsigned char rval;
	if (addr == 0) {
		dev->audio_page = val & 1;
	}
	// currently for debug.
	rc = s2226_aic33_check(dev->audio_page, addr, val);
	if (rc != 0) {
		printk("%s: possible invalid AIC33 write\n", __func__);
	}

	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = DEVID_AUDIO;
	buf[i++] = 1; //mode (no address increment, 8 bit values)
	buf[i++] = 0; // values (MSB)
	buf[i++] = 1; // 1 8 bit data field present
	buf[i++] = 0; // MSB of addr(32 bits in command)
	buf[i++] = 0;
	buf[i++] = 0;
	buf[i++] = addr;
	buf[i++] = val;
	buf[0] = (unsigned char) i;
	dprintk(2, "%s: 0x%02x:0x%02x\n", __func__, addr, val);
	buf[i++] = compute_checksum(buf,buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	if (rc != 0)
		printk(KERN_INFO "failed to send message %d\n", rc);
	s2226_mutex_unlock(&dev->cmdlock);
#if 0
	// DEBUG
	// read back value
	(void) send_aic33_rd(dev, addr, &rval);
	if (rval != val) {
		// not an error because some bits are read-only for 
		// certain registers
		dprintk(2, "%s: mismatch.  Addr %d, value 0x%02x, rval 0x%02x\n",
			__func__, addr, val, rval);
	}
#endif
	return rc;
}



static int send_saa7115_wr(struct s2226_dev *dev, unsigned char addr, unsigned char val)
{
	int i=0;
	unsigned char *buf = dev->control_buffer;
	int rc;
	unsigned char id;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = DEVID_VIDDEC;
	buf[i++] = 1; //mode (no address increment, 8 bit values)
	buf[i++] = 0; // values (MSB)
	buf[i++] = 1; // 1 8 bit data field present
	buf[i++] = 0; // MSB of addr(32 bits in command)
	buf[i++] = 0;
	buf[i++] = 0;
	buf[i++] = addr;
	buf[i++] = val;
	buf[0] = (unsigned char) i;
	dprintk(2, "%s: %x:%x\n", __func__, addr, val);
	buf[i++] = compute_checksum(buf,buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}


static int s2226_set_brightness(struct s2226_dev *dev, unsigned char val)
{
	int rc;
	rc = send_saa7115_wr(dev, 0x0a, val);
	dev->brightness = val;
	return rc;
}

static int s2226_set_contrast(struct s2226_dev *dev, char val)
{
	int rc;
	rc = send_saa7115_wr(dev, 0x0b, (unsigned char) val);
	dev->contrast = val;
	return rc;
}

static int s2226_set_saturation(struct s2226_dev *dev, char val)
{
	int rc;
	rc = send_saa7115_wr(dev, 0x0c, (unsigned char) val);
	dev->saturation = val;
	return rc;
}

static int s2226_set_hue(struct s2226_dev *dev, char val)
{
	int rc;
	rc = send_saa7115_wr(dev, 0x0d, (unsigned char) val);
	dev->hue = val;
	return rc;
}



#define STREAM_MODE_IDLE				0x00
#define STREAM_MODE_ENCODE				0x10
#define STREAM_MODE_DECODE				0x20

static int send_h51_setmode(struct s2226_dev *dev, unsigned char streamID, unsigned char mode)
{
	int i = 0;
	unsigned char id;
	int rc;
	unsigned char *buf = dev->control_buffer;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_SET_MODE, id);
	buf[i++] = streamID;
	buf[i++] = mode;
	buf[0] = (unsigned char) i;
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_SETMODE_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}

static int send_flash_read(struct s2226_dev *dev, unsigned int raddr, int rlen, unsigned char *data, int *len)
{
	int i=0;
	unsigned char id;
	int rc;
	int reqlen = rlen;
	unsigned char *buf = dev->control_buffer;
#define S2226_MAX_FLASHREAD_BYTES 508
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_FLASH_READ, id);
	fill_cmd_buf_val32(buf, i, raddr);
	if (rlen > 255) {
		if (dev->arm_ver < 0x44) {
			s2226_mutex_unlock(&dev->cmdlock);
			return -EINVAL;
		}
		if (rlen > S2226_MAX_FLASHREAD_BYTES) {
			s2226_mutex_unlock(&dev->cmdlock);
			return -EINVAL;
		}
		// use length = 0 for 508 bytes
		rlen = 0;
	}
	buf[i++] = rlen;
	buf[0] = (unsigned char) i;
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_FLASHREAD_TO);
	*len = 0;
	if (rc != 0) {
		s2226_mutex_unlock(&dev->cmdlock);
		return rc;
	}
	buf = (unsigned char *) dev->interrupt_buffer;
	memcpy(data, buf+4, reqlen);
	*len = reqlen;
	s2226_mutex_unlock(&dev->cmdlock);
	return 0;
}

static int send_flash_write(struct s2226_dev *dev, unsigned int addr, unsigned char *data, int len)
{
	int i=0;
	int j;
	int siz;
	unsigned char id;
	int rc;
	unsigned char *buf = dev->control_buffer;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_FLASH_WRITE, id);
	fill_cmd_buf_val32(buf, i, addr);
	for (j = 0; (j < len) && (i < MAX_USB_SIZE); j++)
		buf[i++] = data[j];
	if (i < 255) {
		buf[0] = (unsigned char) i;
	} else {
		if (dev->arm_ver < 0x44) {
			s2226_mutex_unlock(&dev->cmdlock);
			return -EINVAL;
		}
		buf[0] = (unsigned char) i & 0xff;
		// use reserved field for upper 8 bits 
		// support on ARM FW 0x44 and above
		buf[3] = (unsigned char) (i >> 8) & 0xff;
	}
	siz = i;
	buf[i++] = compute_checksum(buf, siz);
	rc = s2226_send_msg(dev, S2226_FLASHWRITE_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}

static int send_flash_erase(struct s2226_dev *dev, unsigned int fw_addr, unsigned int fw_len)
{
	int i = 0;
	unsigned char id;
	int rc;
	unsigned char *buf = dev->control_buffer;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_FLASH_ERASE, id);
	fill_cmd_buf_val32(buf, i, fw_addr);
	fill_cmd_buf_val32(buf, i, fw_len);
	buf[0] = (unsigned char) i;
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_FLASHERASE_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;

}

static int send_persona_run(struct s2226_dev *dev, unsigned char persona_id)
{
	int i = 0;
	int rc;
	unsigned char id;
	unsigned char *buf = dev->control_buffer;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_PERSONA_RUN, id);
	buf[i++] = persona_id;
	buf[i++] = 0;
	buf[0] = (unsigned char) i;
	buf[i++] = compute_checksum(buf,buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}

static int send_persona_halt(struct s2226_dev *dev)
{
	int i = 0;
	int rc;
	unsigned char id;
	unsigned char *buf = dev->control_buffer;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_PERSONA_HALT, id);
	buf[0] = (unsigned char) i;
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}


static int form_h51_nop(unsigned char *buf, unsigned char id)
{
	int i = 0;
	fill_cmd_buf(buf, i, HOSTCMD_NOP, id);
	buf[0] = (unsigned char) i;
	buf[i++] = compute_checksum(buf, buf[0]);
	return i;
}

static int s2226_start_rb_urbs(struct s2226_dev *dev, int context)
{
	int i;
	int rc;
	dev->read_vid_head = 0;
	for (i = 0; i < RD_URBS; i++) {
		usb_fill_bulk_urb(dev->read_vid[i].urb, dev->udev,
				  usb_rcvbulkpipe(dev->udev, dev->ep[INDEX_EP_H264]),
				  (void *) dev->read_vid[i].buffer,
				  S2226_RB_PKT_SIZE,
				  s2226_read_vid_callback,
				  &dev->read_vid[i]);
		dev->read_vid[i].ready = 0;
		dev->read_vid[i].context = context;
		rc = usb_submit_urb(dev->read_vid[i].urb, GFP_KERNEL);
		if (rc != 0) {
			dprintk(0, "%s: submit urb failed %d\n", __func__, rc);
			return rc;
		}
	}
	return 0;
}
				   
static int s2226_start_encode(struct s2226_dev *dev, int idx, int context)
{
	int i, bank;
	int rc;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	if (!dev->udev) return -ENODEV;

	dprintk(1,"cur_input=%d\n", dev->cur_input);
	dprintk(1,"vFormat=%d\n", dev->h51_mode.vFormat);
	dprintk(1,"frRed=%d\n", dev->h51_mode.frRed);
	dprintk(1,"vBitrate=%d\n", dev->h51_mode.vBitrate);
	dprintk(1,"aBitrate=%d\n", dev->h51_mode.aBitrate);
	dprintk(1,"aMode=%d\n", dev->h51_mode.aMode);


	// 1) Change to alternate configuration
	iface_desc = dev->interface->cur_altsetting;
	// temporarily commented out
	//if (iface_desc->desc.bAlternateSetting != 1)
	s2226_set_interface(dev, 0, 1);
	if (dev->h51_reload_required) {
		send_h51_boot(dev);
		dev->h51_reload_required = 0;
	}
	iface_desc = dev->interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;
		dprintk(1,"endpoint %d is %s %s\n", endpoint->bEndpointAddress & 7,
			((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK) ? "bulk" : "other",
			((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) ? "in" : "out");
	}

	setH51regs(&dev->h51_mode);
	for (bank = 0; bank < 4; bank++) {
		int numregs = 0;
		switch (bank) {
		case 0: numregs = h51APRMregs; break;
		case 1: numregs = h51IPRMregs; break;
		case 2: numregs = h51IPRVregs; break;
		case 3: numregs = h51IPRAregs; break;
		}
		for (i = 0; i < numregs; i++) {
			unsigned int address = 0, value = 0;
			switch(bank) {
			case 0: address = h51APRM[i][0]; value = h51APRM[i][1]; break;
			case 1: address = h51IPRM[i][0]; value = h51IPRM[i][1]; break;
			case 2: address = h51IPRV[i][0]; value = h51IPRV[i][1]; break;
			case 3: address = h51IPRA[i][0]; value = h51IPRA[i][1]; break;
			}
			dprintk(3, "h51 register setting: %08x: %04x\n", address, value);
			if (address == H51REG_GOP_CLK) {
				if (dev->closed_gop) {
					printk("closed gop\n");
					value |= 0x0400;
				} else {
					value &= ~0x0400;
				}
			}
			if (address == H51REG_GOP_STRM) {
				if (dev->gop_struct != -1) {
					printk(KERN_INFO "GOP struct override %d\n", dev->gop_struct);
					value &= 0x00ff;
					value |= ((dev->gop_struct & 0xff) << 8);
					if (dev->gop_struct == 2 || (dev->gop_struct > 3))
						printk(KERN_INFO "using reserved value\n");
				}
			}
			if (address == H51REG_AIN_OFFSET) {
				if (dev->ainoffset != -1) {
					printk(KERN_INFO "AIN_OFFSET override 0x%x\n", dev->ainoffset);

					if (dev->ainoffset > AINOFFSET_MAX) {
						dev->ainoffset = AINOFFSET_MAX;
						printk(KERN_INFO "clamping ainoffset to max value");
					}
					value = dev->ainoffset;
				}
			}
			rc = send_h51_regwr(dev, address, value);
			if (rc < 0) { dprintk(1, "%s regwr %d\n", __func__, rc); return rc; }
		}
	}
	// 3) Send Halt to config persona:
	rc = send_persona_halt(dev);
	if (rc < 0) { dprintk(1, "%s persona_halt %d\n", __func__, rc); return rc; }
	dev->persona = PERSONA_DEFAULT;

	// 4) Set encode persona:
	printk("done persona encode\n");
	rc = send_persona_run(dev, 1);
	if (rc < 0) { dprintk(1, "%s persona_run %d\n", __func__, rc); return rc; }

	dev->persona = PERSONA_ENCODE;
	s2226_flush_in_ep(dev);

	// start the read pipe
	s2226_start_rb_urbs(dev, context);

	// 5)  Start encode
	rc = send_h51_setmode(dev, 0, STREAM_MODE_ENCODE);
	if (rc < 0) { dprintk(1, "%s persona_halt %d\n", __func__, rc); return rc; }

	dev->h51_state = STREAM_MODE_ENCODE;

	return 0;
}

int s2226_stop_encode(struct s2226_dev *dev, int idx)
{
	int rc;
	int streaid = 0;

	rc = send_h51_setmode(dev, streaid, STREAM_MODE_IDLE);
	if (rc < 0)
		return rc;

	rc = send_persona_halt(dev);
	if (rc < 0)
		return rc;

	dev->persona = PERSONA_DEFAULT;
	dev->h51_state = STREAM_MODE_IDLE;
	return 0;
}	

int s2226_start_decode(struct s2226_dev *dev, int idx)
{
	int i;
	int rc;
	int numregs;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	if (!dev->udev) return -ENODEV;

	dprintk(1,"vFormat=%d\n", dev->h51_mode.vFormat);
	dprintk(1,"vBitrate=%d\n", dev->h51_mode.vBitrate);
	dprintk(1,"aBitrate=%d\n", dev->h51_mode.aBitrate);
	dprintk(1,"aMode=%d\n", dev->h51_mode.aMode);

    
	// 1) Change to alternate configuration
	iface_desc = dev->interface->cur_altsetting;
	// temporarily commented out
	//if (iface_desc->desc.bAlternateSetting != 0)
	s2226_set_interface(dev, 0, 0);
	iface_desc = dev->interface->cur_altsetting;
	if (dev->h51_reload_required) {
		send_h51_boot(dev);
		dev->h51_reload_required = 0;
	}
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;
		dprintk(1,"endpoint %d is %s %s\n", endpoint->bEndpointAddress & 7,
			((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK) ? "bulk" : "other",
			((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) ? "in" : "out");
	}

	numregs = sizeof(G_h51reg_dec) / (2*sizeof(u32));
	// 2) send h51 registers
	// update decode registers based on format
	for (i = 0; i < numregs; i++) {
		int address, value;
		address = G_h51reg_dec[i][0];
		value = G_h51reg_dec[i][1];
		switch (address) {
		case H51REG_V_V420_FILTER_00:
			// default is interlaced
			value = (IS_PROG_INPUT(dev->cur_input)) ? 0x07 : value;
			break;
		case H51REG_V_V420_FILTER_02:
			value = (IS_PROG_INPUT(dev->cur_input)) ? 0x56 : value;
			break;
		case H51REG_V_V420_FILTER_04:
			value = (IS_PROG_INPUT(dev->cur_input)) ? 0x7d : value;
			break;
		case H51REG_V_V420_FILTER_06:
			value = (IS_PROG_INPUT(dev->cur_input)) ? 0x26 : value;
			break;
		case H51REG_V_V420_FILTER_08:
		case H51REG_V_V420_FILTER_0A:
		case H51REG_V_V420_FILTER_0C:
		case H51REG_V_V420_FILTER_0E:
			value = (IS_PROG_INPUT(dev->cur_input)) ? 0x00 : value;
			break;
		}
		if (address == H51REG_V_FORMAT) {
			switch (dev->cur_input) {
			case INPUT_H51_SD_480I:
				value = 0x1004;
				break;
			case INPUT_H51_SD_576I:
				value = 0x1005;
				break;
			case INPUT_H51_HD_720P_2398:
			case INPUT_H51_HD_720P_24:
				value = 0x102a;
				break;
			case INPUT_H51_HD_1080P_2398:
			case INPUT_H51_HD_1080P_24:
				value = 0x1022;
				break;
			case INPUT_H51_HD_1080I_60:
			case INPUT_H51_HD_1080I_5994:
				value = 0x1000;
				break;
			case INPUT_H51_HD_1080I_50:
				value = 0x1001;
				break;
			case INPUT_H51_HD_720P_60:
			case INPUT_H51_HD_720P_5994:
				value = 0x1002;
				break;
			case INPUT_H51_HD_720P_50:
				value = 0x1003;
				break;
			}
		}
		if (address == H51REG_GOP_STRM) {
			switch (dev->cur_input) {
			case INPUT_H51_SD_480I:
			case INPUT_H51_SD_576I:
			case INPUT_H51_HD_720P_50:
			case INPUT_H51_HD_720P_60:
			case INPUT_H51_HD_720P_5994:
			case INPUT_H51_HD_720P_2398:
			case INPUT_H51_HD_720P_24:
				value = 0x0000;
				break;
			case INPUT_H51_HD_1080I_60:
			case INPUT_H51_HD_1080I_50:
			case INPUT_H51_HD_1080I_5994:
			case INPUT_H51_HD_1080P_2398:
			case INPUT_H51_HD_1080P_24:
				value = 0x0002;
				break;
			}
		}
		
		if ((address == H51REG_V_IP_DETECT) && IS_PROG_INPUT(dev->cur_input)) {
			value = 0x0001;
			if (*s2226_filter_mode == 1) {
				printk("s2226: turning off V_IP_DETECT\n");
				value = 0x0000;
			}
		}

		if (address == H51REG_DPB_SIZE && dev->dpb_size > -1) {
			value = dev->dpb_size;
			printk("using dpb size %d\n", value);
		}

		// frRed is set after the set_input
		// 74.1758 not defined in the ARM, so we piggy back
		// on the non frRed inputs and set frRed value.
		// only need to change V_VINPELCLK H51 register
		if (address == H51REG_GOP_CLK) {
			if (dev->h51_mode.frRed == 0)
				value |= 0x0001; // 74.25 (no reduction)
			else
				value &= ~0x0001;
		}

		
		if ((address == H51REG_AV_RESYNC) && (dev->avresync != -1)) {
			if (dev->avresync && (dev->avresync < AVRESYNC_MIN))
				dev->avresync = AVRESYNC_MIN;
			if (dev->avresync > AVRESYNC_MAX)
				dev->avresync = AVRESYNC_MAX;
			value |= (dev->avresync << 2);
			printk(KERN_INFO "using AV_RESYNC_THRESHOLD 0x%x\n", dev->avresync);
		}
		dprintk(3, "h51 register setting: %08x: %04x\n", address, value);
		rc = send_h51_regwr(dev, address, value);
		if (rc < 0) { dprintk(1, "%s regwr %d\n", __func__, rc); return rc; }
	}
	// 3) Send Halt to config persona:
    
	rc = send_persona_halt(dev);
	if (rc < 0) { dprintk(0, "%s persona_halt %d\n", __func__, rc); return rc; }
	dev->persona = PERSONA_DEFAULT;
	// 4) Set decode persona:
	printk("done persona\n");
	rc = send_persona_run(dev, PERSONA_DECODE);
	if (rc < 0) { dprintk(1, "%s persona_run %d\n", __func__, rc); return rc; }

	dev->persona = PERSONA_DECODE;

	// 5)  Start decode
	rc = send_h51_setmode(dev, 0, STREAM_MODE_DECODE);
	if (rc < 0) { dprintk(1, "%s persona_halt %d\n", __func__, rc); return rc; }

	dev->h51_state = STREAM_MODE_DECODE;
	return 0;
}

int s2226_stop_decode(struct s2226_dev *dev, int idx)
{
	int rc;
	rc = send_h51_setmode(dev, 0, STREAM_MODE_IDLE);
	if (rc < 0)
		return rc;
	rc = send_persona_halt(dev);
	if (rc < 0)
		return rc;

	dev->persona = PERSONA_DEFAULT;
	dev->h51_state = STREAM_MODE_IDLE;
	return 0;
}

int s2226_set_attr(struct s2226_dev *dev, int attr, int value);

int s2226_is_decode_input(int value) 
{
	switch (value) {
	case INPUT_H51_SD_480I:
	case INPUT_H51_SD_576I:
	case INPUT_H51_HD_1080I_50:
	case INPUT_H51_HD_1080I_5994:
	case INPUT_H51_HD_1080I_60:
	case INPUT_H51_HD_720P_60:
	case INPUT_H51_HD_720P_5994:
	case INPUT_H51_HD_720P_50:
		return 1;
	default:
		return 0;
	}
}

int s2226_set_h51_master(struct s2226_dev *dev, int bMaster)
{
	int ret;
	ret = s2226_set_attr(dev, ATTR_AUDH51_MASTER, bMaster);
	printk(KERN_INFO "setting H51 audio INTF to %s, rc: %d\n", bMaster ? "MASTER" : "SLAVE", ret);
	return ret;
}

int s2226_set_attr(struct s2226_dev *dev, int attr, int value)
{
	unsigned char *buf = (unsigned char *) dev->control_buffer;
	int rc;
	unsigned char id;
	int i = 0;
	int timeout = S2226_BULKMSG_TO;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_ATTR_WRITE, id);
	fill_cmd_buf_val32(buf, i, attr);
	fill_cmd_buf_val32(buf, i, value);
	buf[0] = i;
	buf[i++] = compute_checksum(buf,buf[0]);

	if (attr == ATTR_INPUT) {
		timeout = S2226_SETINPUT_TO;
		dev->h51_reload_required  = 0;
	}

	rc = s2226_send_msg(dev, timeout);

	if (rc < 0) {
		printk("%s fail %d\n", __func__, rc);
		s2226_mutex_unlock(&dev->cmdlock);
		return rc;
	}
	buf = (unsigned char *) dev->interrupt_buffer;
	rc = buf[3];
	if (attr == ATTR_INPUT)
		rc = (buf[3] != 0) ? -EAGAIN : 0;
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}

static int s2226_get_attr_ext(struct s2226_dev *dev, int attr, int *value, int *v2);
static int s2226_get_attr(struct s2226_dev *dev, int attr, int *value)
{
	return s2226_get_attr_ext(dev, attr, value, NULL);
}

static int s2226_get_attr_ext(struct s2226_dev *dev, int attr, int *value, int *v2)
{
	unsigned char *buf = dev->control_buffer;
	int rc;
	unsigned char id;
	int i = 0;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_ATTR_READ, id);
	fill_cmd_buf_val32(buf, i, attr);
	buf[0] = i;
	buf[i++] = compute_checksum(buf,buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	if (rc < 0) {
		printk("%s fail %d\n", __func__, rc);
		s2226_mutex_unlock(&dev->cmdlock);
		return rc;
	}
	buf = (unsigned char *) dev->interrupt_buffer;
	if (buf[3] != 0) {
		printk("%s: invalid returned value %d\n", __func__, buf[3]);
		s2226_mutex_unlock(&dev->cmdlock);
		return buf[3];
	}
	if ((attr == ATTR_LAST_ERR) || (attr == ATTR_LAST_MESSAGE)) {
		dprintk(0, "%s dump: %x %x %x %x\n %x %x %x %x\n %x %x %x %x \n",
			(attr==ATTR_LAST_ERR) ? "last err" : "last msg",
			buf[3], buf[4],
			buf[5], buf[6],
			buf[7], buf[8],
			buf[9], buf[10],
			buf[11], buf[12],
			buf[13], buf[14]);
	}
	*value = (buf[4] << 24) + (buf[5] << 16) + (buf[6] << 8) + buf[7];
	if (v2)
		*v2 = (buf[8] << 24) + (buf[9] << 16) + (buf[10] << 8) + buf[11];
	s2226_mutex_unlock(&dev->cmdlock);
	return 0;
}

static int s2226_send_nop(struct s2226_dev *dev, int bResp)
{
	int rlen;
	int rc = 0;
	s2226_mutex_lock(&dev->cmdlock);
	form_h51_nop(dev->control_buffer, dev->id++);
	if (bResp)
		rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	else {
		(void) write_control_endpoint(dev, S2226_PRIMEFX2_TO);
		// read interrupt endpoint with short timeout
		// in case FX2 is just priming itself
		// but long enough in case of POR when there is
		// a delay before the firmware in the ARM loads
		// 3s should be plenty
		(void) usb_bulk_msg(dev->udev, usb_rcvbulkpipe(dev->udev, dev->ep[INDEX_EP_RESP]),
				    dev->interrupt_buffer,
				    512,
				    &rlen,
				    S2226_PRIMEFX2_TO);
	}
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}

static int s2226_prime_fx2(struct s2226_dev *dev)
{
	void *tbuf;
	int rc;
	int i;
	int epnum;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;

	if (dev->interface == NULL)
		return -1;
	tbuf = kmalloc(512, GFP_KERNEL);
	if (tbuf == NULL)
		return -ENOMEM;

	if (dev->usb_ver >= 8 && dev->usb_ver < 0x20) {
		rc = 0;
		(void) s2226_vendor_request(dev, 0x12, 0, 0, &rc, 2, 0);
		(void) s2226_vendor_request(dev, 0x14, 0, 0, &rc, 2, 0);
		(void) s2226_vendor_request(dev, 0x16, 0, 0, &rc, 2, 0);
	}

	// write some nops to prime the control out buffer
	for (i = 0; i < 2; i++) {
		(void) s2226_send_nop(dev, 0); // NOP without response
	}
	iface_desc = dev->interface->cur_altsetting;
	// flush all the in endpoints
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;
		epnum = endpoint->bEndpointAddress & 7;
		if (((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
		     == USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
		     == USB_ENDPOINT_XFER_BULK)) {
			// flush any data in FX2, just in case
			int j = 0;
			do {
				rc = usb_bulk_msg(dev->udev,
						  usb_rcvbulkpipe(dev->udev, epnum),
						  tbuf,
						  512,
						  NULL,
						  (dev->usb_ver < 0x20) ?
						  S2226_PRIMEFX2_TO :
						  10);
				j++;
			} while (rc >= 0 && (j < 10));
		}
	}
	kfree(tbuf);
	return 0;
}


static int s2226_set_interface(struct s2226_dev *dev, int cfg, int alt)
{
	int rc;
	// ARM should be halted before changing the interface.
	// Also, it should not be streaming or decoding
	if (dev->h51_state == STREAM_MODE_ENCODE) {
		s2226_stop_encode(dev, 0);
	}
	if (dev->h51_state == STREAM_MODE_DECODE) {
		s2226_stop_decode(dev, 0);
	}
	// always send halt
	(void) send_persona_halt(dev);
	// TODO: this might be fixed in newer firmwares
	// need to send again or first encode after power up
	// may not work
	(void) send_persona_halt(dev);
 	// set the interface
 	rc = usb_set_interface(dev->udev, cfg, alt);
 	s2226_prime_fx2(dev);
	return rc;
}

static int s2226_new_input(struct s2226_dev *dev, int input)
{
	int vFormat = 0; 
	int frRed = 0;
	int is_decode = 0;
	switch (input) {
	case INPUT_COMP0_480I:
	case INPUT_COMP1_480I:
	case INPUT_SVIDEO0_480I:
	case INPUT_SVIDEO1_480I:
	case INPUT_H51_SD_480I:
		vFormat = VFMT_480_60i;
		frRed = 1; // for SD, this field is a don't care
		break;
	case INPUT_COMP0_576I:
	case INPUT_COMP1_576I:
	case INPUT_SVIDEO0_576I:
	case INPUT_SVIDEO1_576I:
	case INPUT_H51_SD_576I:
		vFormat = VFMT_576_50i;
		frRed = 0; // for SD, this field is a don't care
		break;
	case INPUT_SDI_480I:
	case INPUT_SDI_480I_CB:
		vFormat = VFMT_480_60i;
		frRed = 1; // for SD, this field is a don't care
		break;
	case INPUT_SDI_576I:
	case INPUT_SDI_576I_CB:
		vFormat = VFMT_576_50i;
		frRed = 0; // for SD, this field is a don't care
		break;
	case INPUT_SDI_1080I_50:
	case INPUT_SDI_1080I_50_CB:
	case INPUT_H51_HD_1080I_50:
		vFormat = VFMT_1080_50i;
		frRed = 0;
		break;
	case INPUT_SDI_1080I_5994:
	case INPUT_SDI_1080I_5994_CB:
	case INPUT_H51_HD_1080I_5994:
		vFormat = VFMT_1080_60i;
		frRed = 1;
		break;
	case INPUT_SDI_1080I_60:
	case INPUT_SDI_1080I_60_CB:
	case INPUT_H51_HD_1080I_60:
		vFormat = VFMT_1080_60i;
		frRed = 0;
		break;
	case INPUT_SDI_720P_50:
	case INPUT_SDI_720P_50_CB:
	case INPUT_H51_HD_720P_50:
		if (dev->fpga_ver <= 0x18 && dev->fpga_ver != -1) {
			printk(KERN_INFO "upgrade your FPGA firmware, current %x\n",
			       dev->fpga_ver);
			return -EINVAL;
		}
		vFormat = VFMT_720_50p;
		frRed = 0;
		break;
	case INPUT_SDI_720P_5994:
		vFormat = VFMT_720_60p;
		frRed = 1;
		break;
	case INPUT_SDI_720P_60:
	case INPUT_SDI_720P_60_CB:
	case INPUT_SDI_720P_5994_CB:
	case INPUT_H51_HD_720P_60:
	case INPUT_H51_HD_720P_5994: 
		vFormat = VFMT_720_60p;
		if (dev->fpga_ver <= 0x18 && dev->fpga_ver != -1) {
			// 59.94 uses 60hz colorbar reference
			// for decode
			if (input == INPUT_SDI_720P_5994_CB) {
				printk(KERN_INFO "upgrade your FPGA firmware, current %x\n",
				       dev->fpga_ver);
				return -EINVAL;
			}
			frRed = 0;
		} else {
			switch (input) {
			case INPUT_H51_HD_720P_5994:
			case INPUT_SDI_720P_5994:
			case INPUT_SDI_720P_5994_CB:
				frRed = 1;
				break;
			default:
				frRed = 0;
			}
		}
		break;
	case INPUT_SDI_720P_24:
	case INPUT_SDI_720P_24_CB:
	case INPUT_H51_HD_720P_24:
		vFormat = VFMT_720_24p;
		frRed = 0;
		break;
	case INPUT_SDI_720P_2398:
	case INPUT_SDI_720P_2398_CB:
	case INPUT_H51_HD_720P_2398:
		vFormat = VFMT_720_24p;
		frRed = 1;
		break;
	case INPUT_SDI_1080P_24:
	case INPUT_SDI_1080P_24_CB:
	case INPUT_H51_HD_1080P_24:
		vFormat = VFMT_1080_24p;
		frRed = 0;
		break;
	case INPUT_SDI_1080P_2398:
	case INPUT_SDI_1080P_2398_CB:
	case INPUT_H51_HD_1080P_2398:
		vFormat = VFMT_1080_24p;
		frRed = 1;
		break;
	default:
		printk(KERN_ERR "s2226: unknown input %d!\n", input);
		return -EINVAL;
	}
	/* adjust input wait based on current input */
	switch (input) {
	case INPUT_COMP0_480I:
	case INPUT_SVIDEO0_480I:
	case INPUT_COMP0_576I:
	case INPUT_SVIDEO0_576I:
	case INPUT_SDI_480I:
	case INPUT_SDI_480I_CB:
	case INPUT_SDI_576I:
	case INPUT_SDI_576I_CB:
	case INPUT_SDI_1080I_50:
	case INPUT_SDI_1080I_50_CB:
	default:
		break;
	case INPUT_H51_SD_576I:
	case INPUT_H51_SD_480I:
		is_decode = 1;
		break;
	case INPUT_H51_HD_1080I_50:
	case INPUT_H51_HD_1080I_5994:
	case INPUT_H51_HD_1080I_60:
	case INPUT_H51_HD_720P_50:
	case INPUT_H51_HD_720P_5994:
	case INPUT_H51_HD_720P_60:
		is_decode = 1;
		break;
	}
	dev->cur_input = input;
	dev->is_decode = is_decode;
	dev->h51_mode.vFormat = vFormat;
	dev->h51_mode.frRed = frRed;
	dev->input_set = 1;
	/*
	 * new input will clear the audio settings. 
	 * reconfigure if necessary
	 */
	if (audiomode == S2226_AUDIOMODE_AUTO_2420 ||
	    audiomode == S2226_AUDIOMODE_AUTO_2226S) {
		SetAudioIn(dev);
		SetAudioRoute(dev, dev->aud.iRoute);
		SetAudioOut(dev);
	}
#if 0
	if (audiomode == S2226_AUDIOMODE_AUTO_2226S) {
		(void) send_aic33_wr(dev, 37, 0xe0); // HPLCOM single-ended
		(void) send_aic33_wr(dev, 38, 0x14); // HPRCOM single-ended, SC
		(void) send_aic33_wr(dev, 40, 0x40); // 1.5V output voltage
		(void) send_aic33_wr(dev, 41, 0x00);    // DAC_L1 path, (Unlike the 2420,
		//we still need stereo and mono outputs and can't use DAC_L2)
		(void) send_aic33_wr(dev, 47, 0x80); // DAC_L1 to HPLOUT
		(void) send_aic33_wr(dev, 64, 0x80); // DAC_R1 to HPROUT
	}
#endif

	return 0;
}

#define S2226_REG_AUD_INPK_CTL  (0x015<<1)

int s2226_set_audiomux_mpegin(struct s2226_dev *dev, int aud)
{
	int reg;
	int rc;
	rc = get_reg(dev, DEVID_FPGA, S2226_REG_AUD_INPK_CTL, &reg);
	if (rc != 0) {
		printk(KERN_WARNING "s2226 setaudio mux fail\n");
	}
	switch (aud) {
	case S2226_AUDIOINPUT_LINE:
		reg = reg & 0x010c;// Keep bit 8(GENCK),3, 2
		reg = reg | 0x0000;// line in to MPEG in
		break;
	case S2226_AUDIOINPUT_TONE:
		reg = reg & 0x010C; 
		reg = reg | 0x00C2; // Tone test data to MPEG in
		break;
	case S2226_AUDIOINPUT_SDI:
		reg = reg & 0x010C; 
		reg = reg | 0x0042; // SDIiAud data to MPEG in
		break;
	default:
		printk(KERN_INFO "s2226 setaudio mux, invalid input %d\n", aud);
		return -EINVAL;
		break;
	}
	rc = send_fpga_write(dev, S2226_REG_AUD_INPK_CTL, reg);
	return rc;
}


int s2226_ioctl(struct inode *inode, struct file *file,
                unsigned int cmd, unsigned long arg);

static long s2226_ioctl_compat( struct file *file,
				unsigned int cmd, unsigned long arg)
{
	long rc;
	dprintk(4, "compat ioctl cmd 0x%x, arg 0x%lx\n", cmd, arg);
	rc = s2226_ioctl(NULL, file, cmd, arg);
	return rc;
}

long s2226_ioctl_unlocked(struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	int rc;
	struct s2226_fh  *fh = (struct s2226_fh *) file->private_data;
	struct s2226_dev *dev = (struct s2226_dev *) fh->dev;
	dprintk(4, "ioctl unlocked\n");
	if (s2226_mutex_lock_interruptible(&dev->ioctl_lock)) {
		printk("could not acquire lock s2226_open\n");
		return -EAGAIN;
	}
	rc = s2226_ioctl(NULL, file, cmd, arg);
	s2226_mutex_unlock(&dev->ioctl_lock);
	return rc;
}
int s2226_get_fpga_ver(struct s2226_dev *dev);

int s2226_ioctl(struct inode *inode, struct file *file,
                unsigned int cmd, unsigned long arg)
{
	struct s2226_fh  *fh = (struct s2226_fh *) file->private_data;
	struct s2226_dev *dev = (struct s2226_dev *) fh->dev;
	void __user *argp = (void __user *)arg;
	int ret=0;
	dprintk(4, "fh %p ioctl %x %lx\n", fh, cmd, arg);
	if (dev == NULL) {
		printk("invalid device\n");
		return -EINVAL;
	}

	switch (cmd) {
	case S2226_IOC_DECODE_STATE:
	{
		int j;
		for (j = 0; j < WR_URBS; j++)
			if (!dev->write_vid[j].ready) {
				return 1; // still decoding
			}
                // driver idle,
		// but H51 decoder could still be running
		return 0; 
	}
	case S2226_IOC_I2C_TX:
	{
		i2c_param_t p;
		unsigned char addr;
		if (copy_from_user(&p, (void __user *) arg, sizeof(i2c_param_t))) {
			return -EINVAL;
		}
		addr = (p.addr);// >> 1);
		dprintk(1, "I2C_TX addr:[%x], len:%d, data: %x\n", addr, p.len, p.data[2]);
		ret = s2226_vendor_request(dev,
					   0x23,
					   addr, addr,	// val, idx
					   p.data, //pbuf
					   p.len,// len
					   1);	// bIn
		dprintk(1, "I2C_TX %d\n", ret);
	}
	case S2226_IOC_SET_MODE:
	{
		mode_param_t mode;
		if (res_locked(dev, fh) & RES_STREAM) {
			printk(KERN_INFO "s2226: set mode, device busy(streaming)\n");
			return -EBUSY;
		}
		dprintk(3,"set mode\n");
		if (copy_from_user(&mode, argp, sizeof(mode_param_t)))
			return -EINVAL;
		switch (mode.idx) {
		case S2226_IDX_AMODE:
			if (mode.val == AMODE_MP1L2 || mode.val == AMODE_AC3 ||
			    mode.val == AMODE_LPCM || mode.val == AMODE_AAC)
				dev->h51_mode.aMode = mode.val;
			else
				ret = -EINVAL;
			break;
		case S2226_IDX_ABITRATE:
			if (mode.val > 0 && mode.val <= 256)
				dev->h51_mode.aBitrate = mode.val;
			else
				ret = -EINVAL;
			break;
		case S2226_IDX_VBITRATE:
			if (mode.val > 0 && mode.val <= 20000)
				dev->h51_mode.vBitrate = mode.val;
			else
				ret = -EINVAL;
			break;
		case S2226_IDX_STEREO:
			if (mode.val >= 0 && mode.val <= 1)
				dev->h51_mode.aStereo = mode.val;
			else
				ret = -EINVAL;
			break;
		case S2226_IDX_AUDROUTE:
			if (mode.val >= 0 && mode.val <= AUDIOROUTE_LINE2L_BYPASS)
				SetAudioRoute(dev, mode.val);
			else
				ret = -EINVAL;
			break;
		case S2226_IDX_AUDIN_GAIN_L:
		case S2226_IDX_AUDIN_GAIN_R:
		case S2226_IDX_AUDIN_GAIN:
			if (!(mode.val >= 0 && mode.val <= 118))
				return -EINVAL;
			if (mode.idx == S2226_IDX_AUDIN_GAIN_L)
				SetAudioLeftGain(dev, mode.val);
			else if (mode.idx == S2226_IDX_AUDIN_GAIN_R)
				SetAudioRightGain(dev, mode.val);
			else {
				SetAudioLeftGain(dev, mode.val);
				SetAudioRightGain(dev, mode.val);
			}
			break;
		case S2226_IDX_AUDIN_AGC_GAIN_L:
		case S2226_IDX_AUDIN_AGC_GAIN_R:
		case S2226_IDX_AUDIN_AGC_GAIN:
			if (!(mode.val >= 0 && mode.val <= 118))
				return -EINVAL;
			if (mode.idx == S2226_IDX_AUDIN_AGC_GAIN_R)
				SetAudioRightAGC(dev, dev->aud.bAGC_R, mode.val);
			else if (mode.idx == S2226_IDX_AUDIN_AGC_GAIN_L)
				SetAudioLeftAGC(dev, dev->aud.bAGC_L, mode.val);
			else {
				SetAudioRightAGC(dev, dev->aud.bAGC_R, mode.val);
				SetAudioLeftAGC(dev, dev->aud.bAGC_L, mode.val);
			}
			break;
		case S2226_IDX_AUDIN_AGC_ON:
		case S2226_IDX_AUDIN_AGC_ON_R:
		case S2226_IDX_AUDIN_AGC_ON_L:
			if (!(mode.val >= 0 && mode.val <= 1))
				return -EINVAL;
			if (mode.idx == S2226_IDX_AUDIN_AGC_ON_R)
				SetAudioRightAGC(dev, mode.val, dev->aud.iAGCRightGain);
			else if (mode.idx == S2226_IDX_AUDIN_AGC_ON_L)
				SetAudioLeftAGC(dev, mode.val, dev->aud.iAGCLeftGain);
			else {
				SetAudioRightAGC(dev, mode.val, dev->aud.iAGCRightGain);
				SetAudioLeftAGC(dev, mode.val, dev->aud.iAGCLeftGain);
			}
			break;
		case S2226_IDX_AUDIN_BAL:
		case S2226_IDX_AUDIN_BAL_L:
		case S2226_IDX_AUDIN_BAL_R:
			if (!(mode.val >= 0 && mode.val <= 1))
				return -EINVAL;
			if (mode.idx == S2226_IDX_AUDIN_BAL_L)
				SetAudioBalL(dev, mode.val);
			else if (mode.idx == S2226_IDX_AUDIN_BAL_R)
				SetAudioBalR(dev, mode.val);
			else {
				SetAudioBalL(dev, mode.val);
				SetAudioBalR(dev, mode.val);
			}
			break;
		}
		dprintk(3,"set mode: ret: %d\n",ret);
		break;
	}
	case S2226_IOC_GET_MODE:
	{
		mode_param_t mode;
		dprintk(3, "get mode\n");
		if (copy_from_user(&mode, argp, sizeof(mode_param_t)))
			return -EINVAL;
		dprintk(3,"get mode: idx %d\n", mode.idx);
		switch (mode.idx) {
		case S2226_IDX_AMODE:
			mode.val = dev->h51_mode.aMode;
			break;
		case S2226_IDX_ABITRATE:
			mode.val = dev->h51_mode.aBitrate;
			break;
		case S2226_IDX_VBITRATE:
			mode.val = dev->h51_mode.vBitrate;
			break;
		case S2226_IDX_STEREO:
			mode.val = dev->h51_mode.aStereo;
			break;
		}
		ret = copy_to_user(argp, &mode, sizeof(mode_param_t));
		break;
	}
	case S2226_IOC_STARTENCODE:
	{
		start_param_t cmd;
		dprintk(3,"start encode\n");
		if (copy_from_user( &cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(3,"start encode channel %d\n", cmd.idx);
		//printk("start encode %d\n",cmd.idx);
		if (!res_get(dev, fh, RES_STREAM)) {
			printk(KERN_INFO "s2226: stream busy\n");
			return -EBUSY;
		}
		(void) s2226_set_attr(dev, ATTR_INPUT, dev->cur_input);
		(void) s2226_set_audiomux_mpegin(dev, dev->cur_audiompeg);
		ret = s2226_start_encode(dev, cmd.idx, S2226_CONTEXT_USBDEV);
        if (ret != 0)
            res_free(dev, fh, RES_STREAM);
		return ret;
	}
	case S2226_IOC_STARTDECODE:
	{
		start_param_t cmd;
		dprintk(3,"start decode\n");
		if (copy_from_user( &cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(3,"start decode channel %d\n", cmd.idx);
		//printk("start encode %d\n",cmd.idx);
		if (!res_get(dev, fh, RES_STREAM)) {
			printk(KERN_INFO "s2226: stream busy\n");
			return -EBUSY;
		}
		ret = s2226_start_decode(dev, cmd.idx);
        if (ret != 0)
            res_free(dev, fh, RES_STREAM);
		return ret;
	}
	case S2226_IOC_STOPENCODE:
	{
		stop_param_t cmd;
		int i;
		if ((res_locked(dev, fh) & RES_STREAM) && !(res_check(fh) & RES_STREAM)) {
			// other handle owns the streaming resource
			// (If no handle owns resource, allow re-sending
			//  a stop encode or decode command even if stream
			//  not running)
			return 0;
		}
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "s2226: stop encode, kill urbs\n");
		for (i = 0; i < RD_URBS; i++) {
			usb_kill_urb(dev->read_vid[i].urb);
			dev->read_vid[i].ready = 1;
		}
		//dprintk(3,"stop encode channel %d\n", cmd.idx);
		ret = s2226_stop_encode(dev, cmd.idx);
		res_free(dev, fh, RES_STREAM);
		dev->h51_reload_required  = 1;
		return ret;
	}
	case S2226_IOC_STOPDECODE:
	{
		stop_param_t cmd;
		int i;
		if ((res_locked(dev, fh) & RES_STREAM) && !(res_check(fh) & RES_STREAM)) {
			// other handle owns the streaming resource
			// (If no handle owns resource, allow re-sending
			//  a stop encode or decode command even if stream
			//  not running)
			return 0;
		}
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		// kill the urbs
		dprintk(2, "s2226, stop decode, kill urbs\n");
		for (i = 0; i < WR_URBS; i++) {
			usb_kill_urb(dev->write_vid[i].urb);
			dev->write_vid[i].ready = 1;
		}
		//dprintk(3,"stop encode channel %d\n", cmd.idx);
		ret = s2226_stop_decode(dev, cmd.idx);
		res_free(dev, fh, RES_STREAM);
		dev->h51_reload_required  = 1;
		return ret;
	}
	case S2226_IOC_GET_STATUS:
	{
		// TODO
		return -EINVAL;
	}
	case S2226_IOC_SDII_WR:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "sdii write %x:%x\n", cmd.addr, cmd.val);
		ret = send_sdi_write(dev, cmd.addr, cmd.val, 1);
		break;
	}
	case S2226_IOC_SDII_RD:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		ret = get_reg(dev, DEVID_SDI_IN, cmd.addr, &cmd.val);
		if (ret != 0)
			return ret;
		dprintk(2, "sdii rd %x:%x\n", cmd.addr, cmd.val);
		if (ret != 0)
			return ret;
		ret = copy_to_user(argp, &cmd, sizeof(cmd));
		break;
	}
	case S2226_IOC_SDIO_WR:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "sdio write %x:%x\n", cmd.addr, cmd.val);
		ret = send_sdi_write(dev, cmd.addr, cmd.val, 0);
		break;
	}
	case S2226_IOC_SDIO_RD:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		ret = get_reg(dev, DEVID_SDI_OUT, cmd.addr, &cmd.val);
		if (ret != 0)
			return ret;
		dprintk(2, "sdio rd %x:%x\n", cmd.addr, cmd.val);
		if (ret != 0)
			return ret;
		ret = copy_to_user(argp, &cmd, sizeof(cmd));
		break;
	}
	case S2226_IOC_FPGA_WR:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "Fpga write %x:%x\n", cmd.addr, cmd.val);
		ret = send_fpga_write(dev, cmd.addr, cmd.val);
		if (ret < 0) return ret;
		break;
	}
	case S2226_IOC_FPGA_WR_BURST:
	{
		struct io_burst cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "fpga write burst %x\n", cmd.addr);
		ret = send_fpga_write_burst(dev, cmd.addr, cmd.data, cmd.len, FPGA_WRITE_NORMAL);
		if (ret < 0) return ret;
		break;
	}
	case S2226_IOC_FPGA_WR_BURST_FAST:
	{
		struct io_burst cmd;
		if (dev->arm_ver < 0x50) {
			printk(KERN_INFO "s2226: ioctl not available for this firmware[0x%x]\n", dev->arm_ver);
			return -EINVAL;
		}
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "S2226_IOC_FPGA_WR_BURST_FAST %x\n", cmd.addr);
		ret = send_fpga_write_burst(dev, cmd.addr, cmd.data, cmd.len, FPGA_WRITE_FAST);
		if (ret < 0) return ret;
		break;
	}
	case S2226_IOC_FPGA_WR_ADDRDATA:
	{
		struct io_burst cmd;
		if (dev->arm_ver < 0x58) {
			printk(KERN_INFO "s2226: ioctl not available for this firmware[0x%x]\n", dev->arm_ver);
			return -EINVAL;
		}
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "S2226_IOC_FPGA_WR_ADDRDATA %x\n", cmd.addr);
		ret = send_fpga_write_burst(dev, cmd.addr, cmd.data, cmd.len, FPGA_WRITE_ADDRDATA);
		if (ret < 0) return ret;
		break;
	}
	case S2226_IOC_FPGA_RD:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		ret = get_reg(dev, DEVID_FPGA, cmd.addr, &cmd.val);
		if (ret != 0)
			return ret;
		ret = copy_to_user(argp, &cmd, sizeof(cmd));
		break;
	}
	case S2226_IOC_FPGA_RD_BURST:
	{
		struct io_burst cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		ret = send_fpga_read_burst(dev, cmd.addr, cmd.data, cmd.len);
		if (ret != 0)
			return ret;
		ret = copy_to_user(argp, &cmd, sizeof(cmd));
		break;
	}
	case S2226_IOC_H51_WR:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "h51 write %x:%x\n", cmd.addr, cmd.val);
		ret = send_h51_regwr(dev, cmd.addr, cmd.val);
		if (ret < 0) return ret;
		break;
	}
	case S2226_IOC_H51_RD:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		ret = get_reg(dev, DEVID_H51, cmd.addr, &cmd.val);
		if (ret != 0)
			return ret;
		ret = copy_to_user(argp, &cmd, sizeof(cmd));
		break;
	}
	case S2226_IOC_SET_ATTR:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
#define S2226_ATTR_INPUT       13
		if ((res_locked(dev, fh) & RES_STREAM) && (cmd.addr == S2226_ATTR_INPUT)) {
			printk(KERN_INFO "s2226: set attr, device busy(streaming)\n");
			return -EBUSY;
		}
		dprintk(2, "set attr %x:%x\n", cmd.addr, cmd.val);
		ret = s2226_set_attr(dev, cmd.addr, cmd.val);
		break;
	}
	case S2226_IOC_GET_ATTR:
	{
		struct io_reg cmd;
		struct attr_ext cmd2;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		cmd2.attr = cmd.addr;
		ret = s2226_get_attr_ext(dev, cmd2.attr, &cmd2.val, &cmd2.val2);
		if (ret != 0)
			return ret;
		if (cmd2.attr == ATTR_ARM_FPGA_TS)
			ret = copy_to_user(argp, &cmd2, sizeof(cmd2));
		else
			ret = copy_to_user(argp, &cmd2, sizeof(cmd));
		break;
	}
	case S2226_IOC_FLASH_WR:
	{
		struct flash_param p;
		if (dev->users > 1) {
			printk(KERN_INFO "s2226: other process using device\n");
			return -EBUSY;
		}
		if (copy_from_user(&p, argp, sizeof(p)))
			return -EINVAL;
		// protect our memory space
		if (!mfgmode && (p.addr < 0x10000))
			return -EINVAL;
		dprintk(2, "flash write addr:%x, len:%x\n", p.addr, p.len);
		ret = send_flash_write(dev, p.addr, p.data, p.len);
		if (ret < 0) return ret;
		break;
	}
	case S2226_IOC_FLASH_RD:
	{
		struct flash_param p;
		if (copy_from_user(&p, argp, sizeof(p)))
			return -EINVAL;
		dprintk(2, "flash read addr:%x, len:%x\n", p.addr, p.len);
		ret = send_flash_read(dev, p.addr, p.len, p.data, &p.len);
		if (ret < 0) return ret;
		ret = copy_to_user(argp, &p, sizeof(p));
		break;
	}
	case S2226_IOC_FLASH_ERASE:
	{
		struct flash_param p;
		if (dev->users > 1) {
			printk(KERN_INFO "s2226: other process using device\n");
			return -EBUSY;
		}
		if (copy_from_user(&p, argp, sizeof(p)))
			return -EINVAL;
		// protect our memory space for 
		if (!mfgmode && (p.addr < 0x10000))
			return -EINVAL;
		dprintk(2, "flash erase addr:%x, len:%x\n", p.addr, p.len);
		ret = send_flash_erase(dev, p.addr, p.len);
		printk(KERN_INFO "s2226: flash erase %d\n", ret);
		if (ret < 0) return ret;
		break;
	}
	case S2226_IOC_BOOT_H51:
		printk("s226: ioc boot h51\n");
		if (res_locked(dev, fh) & RES_STREAM) {
			printk(KERN_INFO "s2226: boot, device busy(streaming)\n");
			return -EBUSY;
		}
		ret = send_h51_boot(dev);
		if (ret < 0) return ret;
		break;
	case S2226_IOC_BOOT_FPGA:
		if (res_locked(dev, fh) & RES_STREAM) {
			printk(KERN_INFO "s2226: boot, device busy(streaming)\n");
			return -EBUSY;
		}
		ret = send_fpga_boot(dev);
		if (ret < 0) return ret;
		break;
	case S2226_IOC_SET_INPUT:
		dprintk(3, "set input %d\n", (int)arg);
		if (res_locked(dev, fh) & RES_STREAM) {
			printk(KERN_INFO "s2226: set input, device busy(streaming)\n");
			return -EBUSY;
		}
		ret = s2226_set_attr(dev, ATTR_INPUT, arg);
		if (dev->fpga_ver <= 0) {
			s2226_get_fpga_ver(dev);
		}
		if (1) { //ret == 0) {
			int rc;
			rc = s2226_new_input(dev, arg);
			if (rc != 0)
				return rc;
		} else {
			dprintk(1, "input %d is not connected\n", (int)arg);
		}
		(void) s2226_set_audiomux_mpegin(dev, dev->cur_audiompeg);
		break;
	case S2226_IOC_GET_INPUT:
		dprintk(3, "get input %d\n", (int)dev->cur_input);
		ret = copy_to_user(argp, &dev->cur_input, sizeof(cmd));
		break;
	case S2226_IOC_AUDIO_WR:
	{
		audio_reg_t cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(3, "ioc_audio_wr  %d %x\n", cmd.addr, cmd.value);
		ret = send_aic33_wr(dev, cmd.addr, cmd.value);
		if (ret < 0) return ret;
		break;
	}
	case S2226_IOC_AUDIO_RD:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		ret = get_reg(dev, DEVID_AUDIO, cmd.addr, &cmd.val);
		if (ret != 0)
			return ret;
		ret = copy_to_user(argp, &cmd, sizeof(cmd));
		break;
	}
	case S2226_IOC_VIDDEC_RD:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		ret = get_reg(dev, DEVID_VIDDEC, cmd.addr, &cmd.val);
		if (ret != 0)
			return ret;
		ret = copy_to_user(argp, &cmd, sizeof(cmd));
		break;
	}
	case S2226_IOC_SDISPLIT_RD:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		ret = get_reg(dev, DEVID_SDI_SPLIT, cmd.addr, &cmd.val);
		if (ret != 0)
			return ret;
		ret = copy_to_user(argp, &cmd, sizeof(cmd));
		break;
	}
	case S2226_IOC_RESET_BOARD:
		// If resetting from user space, should do the following
		// S2226_IOC_RESET_BOARD
		// sleep(2)  // let ARM load
		// S2226_IOC_PRIME_FX2
		// S2226_IOC_DEFAULT_PARAMS
		ret = s2226_reset_board(dev);
		break;
	case S2226_IOC_ARM_VER:
		ret = copy_to_user(argp, &dev->arm_ver, sizeof(cmd));
		break;
	case S2226_IOC_SET_BASEFW:
		ret = s2226_fx2sam(dev, 0);
		dprintk(2, "%s FX2SAM_LO %d\n", __func__, ret);
		break;
	case S2226_IOC_SET_NEWFW:
		ret = s2226_fx2sam(dev, 1);
		dprintk(2, "%s FX2SAM_HI %d\n", __func__, ret);
		break;
	case S2226_IOC_NOP:
		// sends NOP to the ARM. For use in firmware 
		// if ARM does not 
		ret = s2226_send_nop(dev, 1);
		dprintk(2, "%s NOP %d\n", __func__, ret);
		break;
	case S2226_IOC_RESET_USB:
		// if firmware corrupt, we may have to reset the USB also
		if (dev->users > 1) {
			printk(KERN_INFO "s2226: RESET_USB device busy\n");
			return -EBUSY;
		}
		ret = usb_reset_device(dev->udev);
		dprintk(2, "%s USB_RESET %d\n", __func__, ret);
		break;
	case S2226_IOC_PRIME_FX2:
		if (dev->users > 1) {
			printk(KERN_INFO "s2226: PRIME_FX2 device busy\n");
			return -EBUSY;
		}
		ret = s2226_prime_fx2(dev);
		dprintk(2, "%s PRIME_FX2 %d\n", __func__, ret);
		break;
	case S2226_IOC_DEFAULT_PARAMS:
		ret = s2226_default_params(dev);
		break;
	case S2226_IOC_DPB_SIZE:
		dprintk(3, "%s set DPB SIZE %d\n", __func__, (int)arg);
		dev->dpb_size = (int)arg;
		break;
	case S2226_IOC_GOP_STRUCT:
		dprintk(3, "%s set GOP STRUCT %d\n", __func__, (int)arg);
		dev->gop_struct = (int)arg;
		break;
	case S2226_IOC_AINOFFSET:
		dprintk(3, "%s set AINOFFSET %d\n", __func__, (int)arg);
		dev->ainoffset = (int)arg;
		break;
	case S2226_IOC_AV_RESYNC:
		dprintk(3, "%s set AV_RESYNC_THRESHOLD %d\n", __func__, (int)arg);
		dev->avresync = (int)arg;
		break;
	case S2226_IOC_GET_LEVEL:
	{
		struct level_param lvl;
		dprintk(3, "get level\n");
		if (copy_from_user(&lvl, argp, sizeof(struct level_param)))
			return -EINVAL;
		switch (lvl.idx) {
		case S2226_LEVEL_BRIGHTNESS:
			lvl.val = dev->brightness;
			break;
		case S2226_LEVEL_CONTRAST:
			lvl.val = dev->contrast;
			break;
		case S2226_LEVEL_HUE:
			lvl.val = dev->hue;
			break;
		case S2226_LEVEL_SATURATION:
			lvl.val = dev->saturation;
			break;
		default:
			return -EINVAL;
		}
		dprintk(3,"get level: idx %d, val %d\n", lvl.idx, lvl.val);
		ret = copy_to_user(argp, &lvl, sizeof(struct level_param));
		break;
	}
	break;
	case S2226_IOC_SET_LEVEL:
	{
		struct level_param lvl;
		if (copy_from_user(&lvl, argp, sizeof(struct level_param)))
			return -EINVAL;
		dprintk(3,"set level: idx %d, val %d\n", lvl.idx, lvl.val);
		switch (lvl.idx) {
		case S2226_LEVEL_BRIGHTNESS:
			s2226_set_brightness(dev, lvl.val);
			break;
		case S2226_LEVEL_CONTRAST:
			s2226_set_contrast(dev, lvl.val);
			break;
		case S2226_LEVEL_HUE:
			s2226_set_hue(dev, lvl.val);
			break;
		case S2226_LEVEL_SATURATION:
			s2226_set_saturation(dev, lvl.val);
			break;
		default:
			return -EINVAL;
		}
		ret = 0;
		break;
	}
	case S2226_IOC_FX2_VER:
	{
		char buf[64];
		int siz = 64; // EP0 is 64 bytes max
		ret = s2226_vendor_request(dev,
					   0x30,	// IOCTL_USBSAMP_FIRMWARE_REV
					   0, 0,	// val, idx
					   buf, //pbuf
					   siz,	// len
					   0);	// bIn
		if (ret > 0)
			dprintk(0, "USB firmware version %x.%02x\n", buf[1], buf[0]);
		else
			dprintk(0, "error getting USB firmware version\n");
		ret = buf[0] + (buf[1] << 8);
		break;
	}
	case S2226_IOC_LOCK_OVERLAY:
	{
		if (!res_get(dev, fh, RES_OVERLAY)) {
			printk(KERN_INFO "overlay already locked\n");
			return -EBUSY;
		}
		ret = 0;
		break;
	}
	case S2226_IOC_UNLOCK_OVERLAY:
	{
		if (res_check(fh) & RES_OVERLAY) {
			res_free(dev, fh, RES_OVERLAY);
		}
		ret = 0;
	}

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}


static void s2226_delete(struct kref *kref)
{
	struct s2226_dev *dev = to_s2226_dev(kref);
	int i;
	
	kfree(dev->interrupt_buffer);
	kfree(dev->control_buffer);
	for (i = 0; i < WR_URBS; i++) {
		usb_free_urb(dev->write_vid[i].urb);
		kfree(dev->write_vid[i].buffer);
	}
	for (i = 0; i < RD_URBS; i++) {
		usb_free_urb(dev->read_vid[i].urb);
		kfree(dev->read_vid[i].buffer);
	}
	usb_free_urb(dev->interrupt_urb);
	usb_free_urb(dev->control_urb);
	usb_put_dev(dev->udev);
	s2226_exit_v4l(dev);
	kfree (dev);
	printk(KERN_INFO "s2226 memory released\n");
}

// when device is opened
static int s2226_open(struct inode *inode, struct file *file)
{
	struct s2226_dev *dev;
	struct s2226_fh  *fh = NULL;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;
	dprintk(2,"s2226_open\n");
	subminor = iminor(inode);
	interface = usb_find_interface(&s2226_driver, subminor);
	if (!interface) {
		printk("%s - error, can't find device for minor %d", __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit_err;
	}
	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit_err;
	}

	if (s2226_mutex_lock_interruptible(&s2226_devices_lock)) {
		printk("could not acquire lock s2226_open\n");
		return -ERESTARTSYS;
	}
	if (dev->users > 0)	{
		dprintk(4, "s2226:  device already open.\n");
	}
    
	// allocate file context data
	fh = kmalloc(sizeof(struct s2226_fh),GFP_KERNEL);
	if (NULL == fh) {
		printk(KERN_INFO "s2226: out of memory\n");
		s2226_mutex_unlock(&s2226_devices_lock);
		return -ENOMEM;
	}
	memset(fh,0,sizeof(struct s2226_fh));
	dprintk(1, "opened dev [%d] %p\n", subminor, dev);
	file->private_data = fh;
	fh->dev            = dev;
	// increment our usage count for the device 
	dev->users++;
	dprintk(1, "s2226: number of users %d\n", dev->users);
	kref_get(&dev->kref);
	s2226_mutex_unlock(&s2226_devices_lock);
	return 0;
exit_err:
	if (fh) {
		kfree( fh);
	}
	return retval;
}

// when device is released
static int s2226_release(struct inode *inode, struct file *file)
{
	struct s2226_fh         *fh = file->private_data;
	struct s2226_dev        *dev;
	int i;
	int res;
	if (s2226_mutex_lock_interruptible(&s2226_devices_lock)) {
		return -ERESTARTSYS;
	}
	if (fh == NULL) {
		s2226_mutex_unlock( &s2226_devices_lock);
		return -ENODEV;
	}
	dev = fh->dev;
	if (dev == NULL) { 
		s2226_mutex_unlock( &s2226_devices_lock);
		return -ENODEV;
	}
	res = res_check(fh);
	if (res) {
		if (res & RES_STREAM) {
			if (dev->h51_state == STREAM_MODE_ENCODE)
				s2226_stop_encode(dev, 0);
			if (dev->h51_state == STREAM_MODE_DECODE)
				s2226_stop_decode(dev, 0);
			for (i = 0; i < WR_URBS; i++) {
				usb_kill_urb(dev->write_vid[i].urb);
				dev->write_vid[i].ready = 1;
			}
			for (i = 0; i < RD_URBS; i++) {
				usb_kill_urb(dev->read_vid[i].urb);
				dev->read_vid[i].ready = 0;
			}
		}
		res_free(dev, fh, res);
	}
	kfree (fh);
	dev->users--;

	s2226_mutex_unlock( &s2226_devices_lock);
	// decrement the count on our device
	kref_put(&dev->kref, s2226_delete);
	return 0;
}


static void s2226_read_vid_callback(struct urb *u)
{
	struct s2226_urb *urb;
	struct s2226_dev *dev;
	int i;
	int urb_context;
	urb = (struct s2226_urb *)u->context;
	dev = urb->dev;
	urb_context = urb->context;
	urb->ready = 1;

	for (i = 0; urb != &urb->dev->read_vid[i]; i++)
		; // find the index of the urb that completed

	dprintk(4, "s2226_read_vid_callback %d\n", i);
	if (urb_context == S2226_CONTEXT_V4L) {	
		s2226_got_data(dev, i);
		// submit the urb again
		dprintk(4, "data from URB %d, resubmit URB\n", i);
		usb_fill_bulk_urb(dev->read_vid[i].urb, dev->udev,
				  usb_rcvbulkpipe(dev->udev, dev->ep[INDEX_EP_H264]),
				  (void *) dev->read_vid[i].buffer,
				  S2226_RB_PKT_SIZE,
				  s2226_read_vid_callback,
				  &dev->read_vid[i]);
		//dev->read_vid[i].ready = 0;
		(void) usb_submit_urb(dev->read_vid[i].urb, GFP_ATOMIC);
	} else {
		wake_up(&urb->dev->read_vid_wq);
	}
	return;
}


static ssize_t s2226_read_vid(struct file *file, char *buffer, size_t nbytes,
			      loff_t *ppos)
{
	struct s2226_fh *fh = (struct s2226_fh *) file->private_data;
	struct s2226_dev *dev = (struct s2226_dev *) fh->dev;
	int i;
	int rc;
	// only let the handle that started the encode do the reading
	if (res_locked(dev, fh) & RES_STREAM) {
		// if locked
		if (!(res_check(fh) & RES_STREAM)) {
			printk(KERN_INFO "s2226: other file handle owns resource. read failed\n");
			return 0;
		}
	}

	if (nbytes != S2226_RB_PKT_SIZE)
		return -EINVAL;

	i = dev->read_vid_head++;
	if (i+1 == RD_URBS) dev->read_vid_head = 0;
	while (dev->read_vid[i].ready == 0) {
		rc = wait_event_interruptible_timeout(dev->read_vid_wq,
						      (dev->read_vid[i].ready != 0),
						      msecs_to_jiffies(6000));
		dprintk(4, "%s wait ready: %d\n", __func__, rc);
		if (rc <= 0) // interrupted or timeout
			return rc;
	}

	rc = copy_to_user(buffer, dev->read_vid[i].buffer, S2226_RB_PKT_SIZE);
	if (dev->udev == NULL)
		return -ENODEV;
	usb_fill_bulk_urb(dev->read_vid[i].urb, dev->udev,
			  usb_rcvbulkpipe(dev->udev, dev->ep[INDEX_EP_H264]),
			  (void *) dev->read_vid[i].buffer,
			  S2226_RB_PKT_SIZE,
			  s2226_read_vid_callback,
			  &dev->read_vid[i]);
	dev->read_vid[i].ready = 0;
	rc = usb_submit_urb(dev->read_vid[i].urb, GFP_KERNEL);
	if (rc != 0) {
		dprintk(0, "%s: urb failed 0x%x\n", __func__, rc);
		return rc;
	}

	return S2226_RB_PKT_SIZE;
}
	
static void s2226_write_vid_callback(struct urb *u)
{
	struct s2226_urb *urb;
	int i;
	urb = (struct s2226_urb *)u->context;
	urb->ready = 1;
	urb->dev->write_vid_ready = 1;
	for (i = 0; urb != &urb->dev->write_vid[i]; i++)
		; // find the index of the urb that completed
	dprintk(4, "s2226_write_vid_callback %d\n", i);
	wake_up(&urb->dev->write_vid_wq);
	return;
}

static ssize_t s2226_write_vid(struct file *file, const char *buffer, size_t nbytes,
			       loff_t *ppos)
{
	int retval;
	struct s2226_fh *fh = (struct s2226_fh *) file->private_data;
	struct s2226_dev *dev = (struct s2226_dev *) fh->dev;
	size_t writesize;
	int i = 0;
	
	writesize = (nbytes > MAX_USB_SIZE) ? MAX_USB_SIZE : nbytes;

	if (writesize == 0)
		return 0;

	// find an available urb
	while (!dev->write_vid[i].ready) {
		i++;
		if (i >= WR_URBS) {
			i = 0;
			dev->write_vid_ready = 0;
			retval = wait_event_interruptible_timeout(dev->write_vid_wq,
								  (dev->write_vid_ready != 0),
								  msecs_to_jiffies(5000));
			if (retval <= 0) {
				printk("retval %x\n", retval);
				return retval;
			}
			if (!dev->write_vid_ready) {
				return -EFAULT;
			}
		}
	}
	dprintk(4,"s2226_write_vid: %d %p %d\n", i, dev->write_vid[i].buffer, (int)writesize);
    
	if (copy_from_user(dev->write_vid[i].buffer, buffer, writesize)) {
		printk(KERN_ERR "s2226 write video\n");
		return -EFAULT;
	}
	if (dev->udev == NULL)
		return -ENODEV;
	//dprintk(4, "s2226_vwrite 2\n");
	dev->write_vid[i].ready = 0;
	usb_fill_bulk_urb(dev->write_vid[i].urb, dev->udev,
			  usb_sndbulkpipe(dev->udev, dev->ep[INDEX_EP_H264]),
			  (void *) dev->write_vid[i].buffer,
			  writesize,
			  s2226_write_vid_callback, (void *) &dev->write_vid[i]);
	//dprintk(4, "s2226_vwrite 3\n");
	retval = usb_submit_urb(dev->write_vid[i].urb, GFP_KERNEL);
	
	if (retval) {
		printk(KERN_ERR "failed to submit URB %d\n", retval);
		return retval;
	}

	return writesize;
}


static int read_interrupt_endpoint(struct s2226_dev *dev, int timeout)
{
	int interruptsize = MAX_USB_INT_SIZE;
	int retval;
	int actual_length;
	char* pdata;
	retval = usb_bulk_msg(dev->udev,
			      usb_rcvbulkpipe(dev->udev, dev->ep[INDEX_EP_RESP]),
			      dev->interrupt_buffer,
			      interruptsize,
			      &actual_length,
			      timeout);
	pdata = (char *) dev->interrupt_buffer;
	dprintk(4, "%s :[%d] read %d bytes: %x %x %x %x\n", __func__,
		retval,
		actual_length,
		pdata[0],
		pdata[1],
		pdata[2],
		pdata[3]);

	return retval;
}

static int write_control_endpoint(struct s2226_dev *dev, int timeout)
{
	int controlsize = 512;
	int retval;
	int actual_length;
	if (!dev->udev) return -ENODEV;
    
	retval = usb_bulk_msg(dev->udev,
			      usb_sndbulkpipe(dev->udev, dev->ep[INDEX_EP_CONTROL]),
			      dev->control_buffer,
			      controlsize,
			      &actual_length,
			      timeout);

	if (retval < 0)
		dprintk(1, "write control endpoint got %d\n", retval);
    
	return retval;
}

unsigned int s2226_poll (struct file *file, poll_table *wait)
{
	struct s2226_fh *fh = (struct s2226_fh *) file->private_data;
	struct s2226_dev *dev = (struct s2226_dev *) fh->dev;
	unsigned int mask = 0;
	
	poll_wait(file, &dev->read_vid_wq, wait);
	if (dev->read_vid[dev->read_vid_head].ready == 1) {
		mask |= POLLIN | POLLRDNORM;
	}
	return mask;
}

static struct file_operations s2226_vfops = {
	.owner =    THIS_MODULE,
	.open =     s2226_open,
	.compat_ioctl = s2226_ioctl_compat,
	.unlocked_ioctl = s2226_ioctl_unlocked,
	.read =     s2226_read_vid,
	.write =    s2226_write_vid,
	.release =  s2226_release,
	.poll =     s2226_poll,
};


#define EXPECTED_IN_EPS  1   //(depending on board rev, may have 1 or 2 in eps)
#define EXPECTED_OUT_EPS 2   // TODO: currently the interrupt endpoint is a bulk out ep

/* 
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver s2226_class = {
	.name =         "s2226v%d",
	.fops =         &s2226_vfops,
	.minor_base =   USB_S2226_MINOR_BASE,
};
static void s2226_init_audio(struct s2226_audio *aud)
{
	memset(aud, 0, sizeof(struct s2226_audio));
	aud->iRoute = -1;
	return;
}
// probe function.  when USB device is detected by system
static int s2226_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct s2226_dev *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	unsigned char *pmem = NULL;
	int i;
	int in_count = 0; /* number of in endpoints */
	int out_count = 0; /* number of out endpoints */
	int retval = -ENOMEM;
	// required sleep for device to boot.  do not remove
	msleep(1150);
	printk(KERN_INFO "[s2226] %s\n", S2226_DRIVER_VERSION_STRING);

	/* allocate memory for our device state and initialize it to zero */
	dev = (struct s2226_dev *) kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		printk("s2226: out of memory");
		goto error;
	}
	dev->fpga_ver = -1;

	kref_init(&dev->kref);
	s2226_init_audio(&dev->aud);
	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;
	// set up the endpoint information 
	iface_desc = interface->cur_altsetting;
	dprintk(4, "num endpoints %d\n" ,iface_desc->desc.bNumEndpoints);
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;
		if (i < MAX_ENDPOINTS)
			dev->ep[i] = endpoint->bEndpointAddress & 7;

		if (((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
		     == USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
		     == USB_ENDPOINT_XFER_BULK)) {
			in_count++;
		}
		if (((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
		     == USB_DIR_OUT) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
		     == USB_ENDPOINT_XFER_BULK)) {
			out_count++;
		}
	}

	if ((in_count < EXPECTED_IN_EPS) || (out_count != EXPECTED_OUT_EPS)) {
		printk("Could not find correct number of endpoints \n");
		goto error;
	}

	// save our data pointer in this interface device 
	usb_set_intfdata(interface, dev);

	// initialize locks 
	s2226_mutex_init(&dev->ioctl_lock);
	s2226_mutex_init(&dev->audlock);
	s2226_mutex_init(&dev->cmdlock);
	/* we can register the device now, as it is ready */

	dprintk(4,"before usb register video register intfdata %p\n", dev);
	retval = usb_register_dev(interface, &s2226_class);
	dprintk(4,"after usb register video register intfdata %p\n", dev);
	if (retval) {
		/* something prevented us from registering this driver */
		printk("Not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	dprintk(0,"USB 2226 device now attached to USBs2226-%d", interface->minor);

	dev->interrupt_buffer = kmalloc(MAX_USB_INT_SIZE, GFP_KERNEL);
	if (dev->interrupt_buffer == NULL) {
		retval = -ENOMEM;
		goto errorUR;
	}
	dev->control_buffer = kmalloc(MAX_USB_SIZE, GFP_KERNEL);
	if (dev->control_buffer == NULL) {
		retval = -ENOMEM;
		goto errorUR;
	}

	for (i = 0; i < WR_URBS; i++) {
		dev->write_vid[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (dev->write_vid[i].urb == NULL) {
			retval = -ENOMEM;
			goto errorUR;
		}
		dev->write_vid[i].ready = 1;
		dev->write_vid[i].dev = dev;
		dev->write_vid[i].buffer = kmalloc(MAX_USB_SIZE, GFP_KERNEL);
		if (dev->write_vid[i].buffer == NULL) {
			retval = -ENOMEM;
			goto errorUR;
		}
	}
	
	for (i = 0; i < RD_URBS; i++) {
		dev->read_vid[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (dev->read_vid[i].urb == NULL) {
			retval = -ENOMEM;
			goto errorUR;
		}
		dev->read_vid[i].ready = 0;
		dev->read_vid[i].dev = dev;
		dev->read_vid[i].buffer = kmalloc(S2226_RB_PKT_SIZE, GFP_KERNEL);
		if (dev->read_vid[i].buffer == NULL) {
			retval = -ENOMEM;
			goto errorUR;
		}
	}

	dev->interrupt_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (dev->interrupt_urb == NULL) {
		retval = -ENOMEM;
		goto errorUR;
	}
	dev->control_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (dev->control_urb == NULL) {
		retval = -ENOMEM;
		goto errorUR;
	}

	init_waitqueue_head(&dev->read_vid_wq);
	init_waitqueue_head(&dev->write_vid_wq);

	dev->write_vid_ready = 1;

	
	kfree(pmem);

	printk("s2226 successfully loaded\n");
	{
		char buf[64];
		int siz = 64; // EP0 is 64 bytes max
		retval = s2226_vendor_request(dev,
					      0x30,	// IOCTL_USBSAMP_FIRMWARE_REV
					      0, 0,	// val, idx
					      buf, //pbuf
					      siz,	// len
					      0);	// bIn
		if (retval > 0)
			dprintk(0, "USB firmware version %x.%02x\n", buf[1], buf[0]);
		else
			dprintk(0, "error getting USB firmware version\n");
		dev->usb_ver = buf[0] + (buf[1] << 8);
	}
	s2226_prime_fx2(dev);
	retval = s2226_set_attr(dev, ATTR_INT_EP_PKTEND, 0);
	
	{
		int fwver;
		retval = s2226_get_attr(dev, ATTR_ARM_FW_VERSION, &fwver);
		if (retval >= 0) {
			printk(KERN_INFO "s2226: ARM fw version: 0x%04x\n", fwver);
			dev->arm_ver = fwver;
		} else
			printk(KERN_INFO "s2226: err getting ARM fw version %d\n",
			       retval);

		if (fwver & 0xff000000)
			printk(KERN_INFO "s2226: development firmware ONLY!\n");
		else if (fwver < 0x24)
			printk(KERN_INFO "s2226: firmware out of date!\n");
	}
	// set default parameters
	s2226_default_params(dev);

	dprintk(4,"before probe done %p\n",dev);
	dev->h51_mode.gop = 0;
	dev->h51_mode.vbr = 0;
	dev->h51_mode.vBitrate = S2226_DEF_VBITRATE;
	dev->h51_mode.aBitrate = S2226_DEF_ABITRATE;
	dev->h51_mode.aMode =  AMODE_MP1L2;
	dev->closed_gop = 0;
	dev->dpb_size = -1;
	dev->gop_struct = -1;
	dev->ainoffset = -1;
	dev->avresync = -1;
	dev->brightness = 0x80;
	dev->hue = 0;
	dev->saturation = 0x40;
	dev->contrast = 0x40;
	s2226_probe_v4l(dev);
	printk("s2226: probe success\n");
	return 0;
errorUR:
	// unregister the driver.
	usb_deregister_dev(interface, &s2226_class);
error:
	if (pmem)
		kfree(pmem);

	usb_set_intfdata(interface, NULL);
	kref_put(&dev->kref, s2226_delete);
	printk("s2226: probe failed\n");
	return retval;
}



// disconnect routine.  when board is removed physically or with rmmod
static void s2226_disconnect(struct usb_interface *interface)
{
	struct s2226_dev *dev = NULL;
	int minor = interface->minor;
	dprintk(4,"s2226_DISCONNECT %d\n", minor);



	// lock to prevent s2226_open() from racing s2226_disconnect() 
	s2226_mutex_lock( &s2226_devices_lock);
	dev = usb_get_intfdata(interface);
	// give back our minor 
	s2226_mutex_lock(&dev->ioctl_lock);
	usb_deregister_dev(interface, &s2226_class);

	s2226_mutex_unlock(&dev->ioctl_lock);

	if (dev) {
		/* to do */
		/* wake up the read thread in case it is waiting for data */
		wake_up(&dev->read_vid_wq);
		/* wake up the write queue */
		wake_up(&dev->write_vid_wq);
	}
	else { 
		printk("dev null\n");
	}
	dprintk(4,"s2226_DISCONNECT2\n");
	// close 2226 board core
	dev->udev = NULL;
	usb_set_intfdata(interface, NULL);
	dprintk(4,"get intfdata %p\n", dev);

	dprintk(4,"s2226_DISCONNECT5\n");

	// decrement our usage count 
	kref_put(&dev->kref, s2226_delete);
	s2226_mutex_unlock(&s2226_devices_lock);
	printk(KERN_INFO "USB s2226 #%d now disconnected", minor);
}



static struct usb_driver s2226_driver = {
	.name =         "s2226",
	.probe =        s2226_probe,
	.disconnect =   s2226_disconnect,
	.id_table =     s2226_table,
};


static int __init usb_s2226_init(void)
{
	int result;
	// register this driver with the USB subsystem 
	s2226_mutex_init( &s2226_devices_lock);
	result = usb_register(&s2226_driver);
	if (result) {
		printk("usb_register failed. Error number %d", result);
	}

	dprintk(2, "s2226_init: done\n");
	return result;
}

static void __exit usb_s2226_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&s2226_driver);
}

int s2226_vendor_request(void *pdev, unsigned char req, 
			 unsigned short idx, unsigned short val,
			 void *pbuf, unsigned int len,
			 int bOut)
{
	int r;
	struct s2226_dev *dev = (struct s2226_dev *) pdev;
	if (len > 64) {
		printk(KERN_INFO "%s : invalid vendor request length. truncating to EP0 size.\n", __func__);
		len = 64;
	}
	if (!bOut) {
		r = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
				    req,
				    USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
				    val,
				    idx,
				    pbuf, len, S2226_CTRLMSG_TO);
	} else {
		r = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
				    req, USB_TYPE_VENDOR | USB_RECIP_DEVICE,
				    val, idx, pbuf,
				    len,  S2226_CTRLMSG_TO);
	}
	return r;
}


static int s2226_flush_in_ep(struct s2226_dev *dev)
{
	int i;
	int rlen;

	for (i = 0; i < 4; i++) {
		(void) usb_bulk_msg(dev->udev,
				    usb_rcvbulkpipe(dev->udev, dev->ep[INDEX_EP_H264]),
				    dev->interrupt_buffer,
				    512,
				    &rlen,
				    1);
	}
	return 0;
}

#define S2226_VR_ARM_RESET_ASSERT    0x56
#define S2226_VR_ARM_RESET_DEASSERT  0x55


static int s2226_reset_board(struct s2226_dev *dev)
{
	char buf[64];
	int siz = 64;
	int ret;
	dprintk(3, "reset h51\n");
	// does not apply to latest firmware
	if (dev->usb_ver >= 0x20) {
		return -EINVAL;
	}

	// send vendor cmd to assert ARM reset
	ret = s2226_vendor_request(dev, S2226_VR_ARM_RESET_ASSERT,
				   0, 0,	// val, idx
				   buf, //pbuf
				   siz,	// len
				   0);	// bIn
	msleep(10);
	// send vendor cmd to de-assert ARM reset
	ret = s2226_vendor_request(dev,
				   S2226_VR_ARM_RESET_DEASSERT,
				   0, 0,	// val, idx
				   buf, //pbuf
				   siz,	// len
				   0);	// bIn
	// wait for board to come up
	msleep(1150);
	// board reset, make sure board put back at default interface.
	s2226_set_interface(dev, 0, 0);
	return ret;
}

#define S2226_VR_LOAD_NEWFW   0x57  // FX2SAM high
#define S2226_VR_LOAD_ORIGFW  0x58  // FX2SAM low

static int s2226_fx2sam(struct s2226_dev *dev, int bNewFw)
{
	
	char buf[64];
	int siz = 64;
	int rc;
	rc = s2226_vendor_request(dev,
				  bNewFw ? S2226_VR_LOAD_NEWFW : S2226_VR_LOAD_ORIGFW,
				  0, 0,
				  buf, //pbuf
				  siz,	// len
				  0);	// bIn
	return rc;
}

static int s2226_default_params(struct s2226_dev *dev)
{
	int rc;
	// switch to pktend mode for efficiency
	rc = s2226_set_attr(dev, ATTR_INT_EP_PKTEND, 1);
	if (rc != 0) return rc;
	return rc;
}


static int SetAudioRightAGC(struct s2226_dev *dev, int bOn, int gain)
{
	if (gain < 0)
		gain = 0;
	if (gain > 255)
		gain = 255;
	dev->aud.bAGC_R = bOn;
	dev->aud.iAGCRightGain = gain;
	if (!dev->input_set)
		return 0;
	if (dev->is_decode)
		return 0;
	if (bOn) {
		// turn on AGC right
		dprintk(2, "AGC R on, gain %d\n", gain);
		send_aic33_wr(dev, 29, 0x80);
		send_aic33_wr(dev, 30, (unsigned char) gain);
		send_aic33_wr(dev, 31, 0x3e);
	} else {
		// turn off AGC right
		dprintk(2, "AGC R off\n");
		send_aic33_wr(dev, 29, 0x00);
	}
	return 0;
}

static int SetAudioLeftAGC(struct s2226_dev *dev, int bOn, int gain)
{
	if (gain < 0)
		gain = 0;
	if (gain > 255)
		gain = 255;
	dev->aud.bAGC_L = bOn;
	dev->aud.iAGCLeftGain = gain;
	if (!dev->input_set)
		return 0;
	if (dev->is_decode)
		return 0;
	if (dev->aud.bAGC_L) {
		// turn on AGC left
		dprintk(2, "AGC L on, gain %d\n", gain);
		send_aic33_wr(dev, 26, 0x80);
		send_aic33_wr(dev, 27, (unsigned char) gain);
		send_aic33_wr(dev, 28, 0x3e);
	} else {
		// turn off AGC left
		dprintk(2, "AGC L off\n");
		send_aic33_wr(dev, 26, 0x00);
	}
	return 0;
}

static int SetAudioLeftGain(struct s2226_dev *dev, int gain)
{
	if (gain < 0)
		gain = 0;
	if (gain > 0x7f)
		gain = 0x7f;
	dev->aud.iLeftGain = gain;

	if (!dev->input_set)
		return 0;
	if (dev->is_decode)
		return 0;
	dprintk(2, "Audio Left gain %d\n", gain);
	send_aic33_wr(dev, 15, (unsigned char) gain);
	return 0;
}

static int SetAudioRightGain(struct s2226_dev *dev, int gain)
{
	if (gain < 0)
		gain = 0;
	if (gain > 0x7f)
		gain = 0x7f;
	dev->aud.iRightGain = gain;
	if (!dev->input_set)
		return 0;
	if (dev->is_decode)
		return 0;
	dprintk(2, "Audio Right gain %d\n", gain);
	send_aic33_wr(dev, 16, (unsigned char) gain);
	return 0;
}

static int SetAudioBalL(struct s2226_dev *dev, int bBal)
{
	dev->aud.in_balL = bBal;
	if (!dev->input_set)
		return 0;
	if (dev->aud.iRoute == -1)
		return -1;
	dprintk(2, "BAL L %d\n", bBal);
	if (dev->is_decode)
		return 0;
	switch (dev->aud.iRoute) {
	case AUDIOROUTE_LINE1L:
	case AUDIOROUTE_LINE1L_BYPASS:
		send_aic33_wr(dev, 19, bBal ? 0x84 : 0x04); //LINE1L to Left ADC
		break;
	}
	return 0;
}

static int SetAudioBalR(struct s2226_dev *dev, int bBal)
{
	dev->aud.in_balR = bBal;
	if (!dev->input_set)
		return 0;
	if (dev->aud.iRoute == -1)
		return -1;
	if (dev->is_decode)
		return 0;
	dprintk(2, "BAL R %d\n", bBal);
	switch (dev->aud.iRoute) {
	case AUDIOROUTE_LINE1L:
	case AUDIOROUTE_LINE1L_BYPASS:
		send_aic33_wr(dev, 22, bBal ? 0x84 : 0x04); //LINE1R to Left ADC
		break;
	}
	return 0;
}

static int SetAudioIn(struct s2226_dev *dev)
{
	// input must be set
	if (!dev->input_set)
		return -1;
	SetAudioLeftAGC(dev, dev->aud.bAGC_L, dev->aud.iAGCLeftGain);
	SetAudioRightAGC(dev, dev->aud.bAGC_R, dev->aud.iAGCRightGain);
	SetAudioLeftGain(dev, dev->aud.iLeftGain);
	SetAudioRightGain(dev, dev->aud.iRightGain);
	SetAudioBalL(dev, dev->aud.in_balL);
	SetAudioBalR(dev, dev->aud.in_balR);
	return 0;
}


static int SetAudioVol(struct s2226_dev *dev, int gainL, int gainR, int muteL, int muteR)
{
	unsigned char val;
	dev->aud.bVolMuteL = muteL;
	dev->aud.bVolMuteR = muteR;
	dev->aud.iVolGainL = gainL;
	dev->aud.iVolGainR = gainR;

	if (!dev->input_set)
		return -1;
	// set audio out DAC settings (right, left)
	val = dev->aud.bVolMuteL ? 0x80 : 0;
	if (dev->aud.iVolGainL > 0x7f)
		dev->aud.iVolGainL = 0x7f;
	val |= dev->aud.iVolGainL;
	send_aic33_wr(dev, 43, val);
	val = dev->aud.bVolMuteR ? 0x80 : 0;
	if (dev->aud.iVolGainR > 0x7f)
		dev->aud.iVolGainR = 0x7f;
	val |= dev->aud.iVolGainR;
	send_aic33_wr(dev, 44, val);
	return 0;
}

static int SetAudioMono(struct s2226_dev *dev, int gain, int mute)
{
	unsigned char val;
	dev->aud.iMonoGain = gain;
	dev->aud.bMonoMute = mute;

	// set audio out Mono settings(extra gain, balanced)
	if (dev->aud.iMonoGain > 9)
		dev->aud.iMonoGain = 9;
	if (!dev->input_set)
		return -1;
	val = dev->aud.iMonoGain << 4 | (dev->aud.bMonoMute ? 0x01 : 0x09);
	send_aic33_wr(dev, 79, val);
	return 0;
}


static int SetAudioHp(struct s2226_dev *dev, int gainL, int gainR, int muteL, int muteR)
{
	dev->aud.iHpGainL = gainL;
	dev->aud.iHpGainR = gainL;
	dev->aud.bHpMuteR = muteR;
	dev->aud.bHpMuteL = muteL;

	// set audio out Hp settings(extra gain, balanced)
	if (dev->aud.iHpGainL > 9)
		dev->aud.iHpGainL = 9;
	if (dev->aud.iHpGainR > 9)
		dev->aud.iHpGainR = 9;
	if (!dev->input_set)
		return -1;
	send_aic33_wr(dev, 58, dev->aud.iHpGainL << 4 | 0x05);//common left
	send_aic33_wr(dev, 51, dev->aud.iHpGainL << 4 | (dev->aud.bHpMuteL ? 0x05 : 0x0d));//out left
	send_aic33_wr(dev, 65, dev->aud.iHpGainR << 4 | (dev->aud.bHpMuteL ? 0x05 : 0x0d));//out right
	// power down the common on 2226, not connected
	send_aic33_wr(dev, 72, dev->aud.iHpGainR << 4 | 0x05);//right common
	return 0;
}


static int SetAudioStereo(struct s2226_dev *dev, int gainL, int gainR, int muteL, int muteR)
{
	unsigned char val;
	dev->aud.iStereoGainL = gainL;
	dev->aud.iStereoGainR = gainL;
	dev->aud.bStereoMuteR = muteR;
	dev->aud.bStereoMuteL = muteL;

	// set audio out Stereo settings(extra gain, balanced)
	if (dev->aud.iStereoGainL > 9)
		dev->aud.iStereoGainL = 9;
	if (dev->aud.iStereoGainR > 9)
		dev->aud.iStereoGainR = 9;
	if (!dev->input_set)
		return -1;
	val = dev->aud.iStereoGainL << 4 | (dev->aud.bStereoMuteL ? 0x01 : 0x09);
	send_aic33_wr(dev, 86, val);
	val = dev->aud.iStereoGainR << 4 | (dev->aud.bStereoMuteR ? 0x01 : 0x09);
	send_aic33_wr(dev, 93, val);
	return 0;
}

static int SetAudioOut(struct s2226_dev *dev)
{
	if (!dev->input_set)
		return -1;
	SetAudioVol(dev, dev->aud.iVolGainR, dev->aud.iVolGainL, dev->aud.bVolMuteL, dev->aud.bVolMuteR);
	SetAudioMono(dev, dev->aud.iMonoGain, dev->aud.bMonoMute);
	SetAudioHp(dev, dev->aud.iHpGainL, dev->aud.iHpGainR, dev->aud.bHpMuteL, dev->aud.bHpMuteR);
	SetAudioStereo(dev, dev->aud.iStereoGainL, dev->aud.iStereoGainR, dev->aud.bStereoMuteL, dev->aud.bStereoMuteR);
	return 0;
}

// also sets line input balanced or not
static int SetAudioRoute(struct s2226_dev *dev, int route)
{
	if (route == -1) {
		route = S2226_AUDIOROUTE_LINE1L;
	}
	dev->aud.iRoute = route;
	if (!dev->input_set)
		return -1;
	dprintk(2, "audio route %d\n", route);
	// MIC3L/R to Left ADC
	send_aic33_wr(dev, 17, 0xff);
	// MIC3L/R to Right ADC
	send_aic33_wr(dev, 18, 0xff);
	if (dev->is_decode) {
		// ADC is powered down
		dprintk(0, "is_decode, turning AGC off\n");
		send_aic33_wr(dev, 15, 0x80); //ADC PGA muted
		send_aic33_wr(dev, 16, 0x80); //ADC PGA muted
		send_aic33_wr(dev, 19, 0x78);//powered down
		send_aic33_wr(dev, 20, 0x7c);//LINE2L to Left ADC (off)
		send_aic33_wr(dev, 21, 0x78);//LINE1R to Left ADC (off)
		send_aic33_wr(dev, 22, 0x78);//LINE1R to right ADC
		send_aic33_wr(dev, 23, 0x7c);//LINE2R to right ADC (off)
		send_aic33_wr(dev, 24, 0x78);//LINE1L to right ADC (off)
	} else {	
		switch (route) {
		case AUDIOROUTE_LINE1L:
		case AUDIOROUTE_LINE1L_BYPASS:
		default:
			send_aic33_wr(dev, 19, dev->aud.in_balL ? 0x84 : 0x04); //LINE1L to Left ADC
			send_aic33_wr(dev, 20, 0x7c);//LINE2L to Left ADC (off)
			send_aic33_wr(dev, 21, 0x78);//LINE1R to Left ADC (off)
			send_aic33_wr(dev, 22, dev->aud.in_balR ? 0x84 : 0x04); //LINE1R to right ADC
			send_aic33_wr(dev, 23, 0x7c);//LINE2R to right ADC (off)
			send_aic33_wr(dev, 24, 0x78);//LINE1L to right ADC (off)
			break;
		case AUDIOROUTE_LINE2L:
		case AUDIOROUTE_LINE2L_BYPASS:
			send_aic33_wr(dev, 19, 0x7c);// LINE1L OFF (+ADC powered up)
			send_aic33_wr(dev, 20, 0x84);//LINE2L to Left ADC (on, differential)
			send_aic33_wr(dev, 21, 0x78);//LINE1R to Left ADC (off)
			send_aic33_wr(dev, 22, 0x7c);// LINE1R OFF (+ADC powered up)
			send_aic33_wr(dev, 23, 0x84);//LINE2R to right ADC (on, differential)
			send_aic33_wr(dev, 24, 0x78);//LINE1L to right ADC (off)
			break;
		}
	}

	switch (route) {
	case AUDIOROUTE_LINE1L: /* LINE1L routed */
	case AUDIOROUTE_LINE2L: /* LINE2L routed */
		// to HPLOUT
		send_aic33_wr(dev, 45, 0x00); //LINE2L
		send_aic33_wr(dev, 46, 0x00); //PGA_L
		send_aic33_wr(dev, 47, 0x80); //DAC_L1
		send_aic33_wr(dev, 48, 0x00); //LINE2R
		send_aic33_wr(dev, 49, 0x00); //PGA_R
		send_aic33_wr(dev, 50, 0x00); //DAC_R1
		// to HPLCOM
		send_aic33_wr(dev, 52, 0x00); //LINE2L
		send_aic33_wr(dev, 53, 0x00); //PGA_L
		send_aic33_wr(dev, 54, 0x00); //DAC_L1
		send_aic33_wr(dev, 55, 0x00); //LINE2R
		send_aic33_wr(dev, 56, 0x00); //PGA_R
		send_aic33_wr(dev, 57, 0x00); //DAC_R1
		// to HPROUT
		send_aic33_wr(dev, 59, 0x00); //LINE2L
		send_aic33_wr(dev, 60, 0x00); //PGA_L
		send_aic33_wr(dev, 61, 0x00); //DAC_L1
		send_aic33_wr(dev, 62, 0x00); //LINE2R
		send_aic33_wr(dev, 63, 0x00); //PGA_R
		send_aic33_wr(dev, 64, 0x80); //DAC_R1
		// to HPRCOM
		send_aic33_wr(dev, 66, 0x00); //LINE2L
		send_aic33_wr(dev, 67, 0x00); //PGA_L
		send_aic33_wr(dev, 68, 0x00); //DAC_L1
		send_aic33_wr(dev, 69, 0x00); //LINE2R
		send_aic33_wr(dev, 70, 0x00); //PGA_R
		send_aic33_wr(dev, 71, 0x00); //DAC_R1
		// to mono settings
		send_aic33_wr(dev, 73, 0x00); //LINE2L
		send_aic33_wr(dev, 74, 0x00); //PGA_L
		send_aic33_wr(dev, 75, 0x80); //DAC_L1
		send_aic33_wr(dev, 76, 0x00); //LINE2R
		send_aic33_wr(dev, 77, 0x00); //PGA_R
		send_aic33_wr(dev, 78, 0x80); //DAC_R1
		// to stereo left settings
		send_aic33_wr(dev, 80, 0x00); //LINE2L
		send_aic33_wr(dev, 81, 0x00); //PGA_L
		send_aic33_wr(dev, 82, 0x80); //DAC_L1
		send_aic33_wr(dev, 83, 0x00); //LINE2R
		send_aic33_wr(dev, 84, 0x00); //PGA_R
		send_aic33_wr(dev, 85, 0x00); //DAC_R1
		// to stereo right
		send_aic33_wr(dev, 87, 0x00); //LINE2L
		send_aic33_wr(dev, 88, 0x00); //PGA_L
		send_aic33_wr(dev, 89, 0x00); //DAC_L1
		send_aic33_wr(dev, 90, 0x00); //LINE2R
		send_aic33_wr(dev, 91, 0x00); //PGA_R
		send_aic33_wr(dev, 92, 0x80); //DAC_R1
		break;
	case AUDIOROUTE_LINE1L_BYPASS:
		// to HPLOUT
		send_aic33_wr(dev, 45, 0x00); //LINE2L
		send_aic33_wr(dev, 46, 0x80); //PGA_L
		send_aic33_wr(dev, 47, 0x00); //DAC_L1
		send_aic33_wr(dev, 48, 0x00); //LINE2R
		send_aic33_wr(dev, 49, 0x00); //PGA_R
		send_aic33_wr(dev, 50, 0x00); //DAC_R1
		// to HPLCOM
		send_aic33_wr(dev, 52, 0x00); //LINE2L
		send_aic33_wr(dev, 53, 0x00); //PGA_L
		send_aic33_wr(dev, 54, 0x00); //DAC_L1
		send_aic33_wr(dev, 55, 0x00); //LINE2R
		send_aic33_wr(dev, 56, 0x00); //PGA_R
		send_aic33_wr(dev, 57, 0x00); //DAC_R1
		// to HPROUT
		send_aic33_wr(dev, 59, 0x00); //LINE2L
		send_aic33_wr(dev, 60, 0x00); //PGA_L
		send_aic33_wr(dev, 61, 0x00); //DAC_L1
		send_aic33_wr(dev, 62, 0x00); //LINE2R
		send_aic33_wr(dev, 63, 0x80); //PGA_R
		send_aic33_wr(dev, 64, 0x00); //DAC_R1
		// to HPRCOM
		send_aic33_wr(dev, 66, 0x00); //LINE2L
		send_aic33_wr(dev, 67, 0x00); //PGA_L
		send_aic33_wr(dev, 68, 0x00); //DAC_L1
		send_aic33_wr(dev, 69, 0x00); //LINE2R
		send_aic33_wr(dev, 70, 0x00); //PGA_R
		send_aic33_wr(dev, 71, 0x00); //DAC_R1
		// to mono settings
		send_aic33_wr(dev, 73, 0x00); //LINE2L
		send_aic33_wr(dev, 74, 0x80); //PGA_L
		send_aic33_wr(dev, 75, 0x00); //DAC_L1
		send_aic33_wr(dev, 76, 0x00); //LINE2R
		send_aic33_wr(dev, 77, 0x80); //PGA_R
		send_aic33_wr(dev, 78, 0x00); //DAC_R1
		// to stereo left settings
		send_aic33_wr(dev, 80, 0x00); //LINE2L
		send_aic33_wr(dev, 81, 0x80); //PGA_L
		send_aic33_wr(dev, 82, 0x00); //DAC_L1
		send_aic33_wr(dev, 83, 0x00); //LINE2R
		send_aic33_wr(dev, 84, 0x00); //PGA_R
		send_aic33_wr(dev, 85, 0x00); //DAC_R1
		// to stereo right
		send_aic33_wr(dev, 86, 0x00); //LINE2L
		send_aic33_wr(dev, 87, 0x00); //PGA_L
		send_aic33_wr(dev, 88, 0x00); //DAC_L1
		send_aic33_wr(dev, 89, 0x00); //LINE2R
		send_aic33_wr(dev, 90, 0x80); //PGA_R
		send_aic33_wr(dev, 91, 0x00); //DAC_R1
		dprintk(2, "bypass line 1\n");
		break;
	case AUDIOROUTE_LINE2L_BYPASS:
		// to HPLOUT
		send_aic33_wr(dev, 45, 0x80); //LINE2L
		send_aic33_wr(dev, 46, 0x00); //PGA_L
		send_aic33_wr(dev, 47, 0x00); //DAC_L1
		send_aic33_wr(dev, 48, 0x00); //LINE2R
		send_aic33_wr(dev, 49, 0x00); //PGA_R
		send_aic33_wr(dev, 50, 0x00); //DAC_R1
		// to HPLCOM
		send_aic33_wr(dev, 52, 0x00); //LINE2L
		send_aic33_wr(dev, 53, 0x00); //PGA_L
		send_aic33_wr(dev, 54, 0x00); //DAC_L1
		send_aic33_wr(dev, 55, 0x00); //LINE2R
		send_aic33_wr(dev, 56, 0x00); //PGA_R
		send_aic33_wr(dev, 57, 0x00); //DAC_R1
		// to HPROUT
		send_aic33_wr(dev, 59, 0x00); //LINE2L
		send_aic33_wr(dev, 60, 0x00); //PGA_L
		send_aic33_wr(dev, 61, 0x00); //DAC_L1
		send_aic33_wr(dev, 62, 0x80); //LINE2R
		send_aic33_wr(dev, 63, 0x00); //PGA_R
		send_aic33_wr(dev, 64, 0x00); //DAC_R1
		// to HPRCOM
		send_aic33_wr(dev, 66, 0x00); //LINE2L
		send_aic33_wr(dev, 67, 0x00); //PGA_L
		send_aic33_wr(dev, 68, 0x00); //DAC_L1
		send_aic33_wr(dev, 69, 0x00); //LINE2R
		send_aic33_wr(dev, 70, 0x00); //PGA_R
		send_aic33_wr(dev, 71, 0x00); //DAC_R1
		// to mono settings
		send_aic33_wr(dev, 73, 0x80); //LINE2L
		send_aic33_wr(dev, 74, 0x00); //PGA_L
		send_aic33_wr(dev, 75, 0x00); //DAC_L1
		send_aic33_wr(dev, 76, 0x80); //LINE2R
		send_aic33_wr(dev, 77, 0x00); //PGA_R
		send_aic33_wr(dev, 78, 0x00); //DAC_R1
		// to stereo left settings
		send_aic33_wr(dev, 80, 0x80); //LINE2L
		send_aic33_wr(dev, 81, 0x00); //PGA_L
		send_aic33_wr(dev, 82, 0x00); //DAC_L1
		send_aic33_wr(dev, 83, 0x00); //LINE2R
		send_aic33_wr(dev, 84, 0x00); //PGA_R
		send_aic33_wr(dev, 85, 0x00); //DAC_R1
		// to stereo right
		send_aic33_wr(dev, 80, 0x00); //LINE2L
		send_aic33_wr(dev, 81, 0x00); //PGA_L
		send_aic33_wr(dev, 82, 0x00); //DAC_L1
		send_aic33_wr(dev, 83, 0x00); //LINE2R
		send_aic33_wr(dev, 84, 0x80); //PGA_R
		send_aic33_wr(dev, 85, 0x00); //DAC_R1
		break;
	}
	dev->aud.iRoute = route;
	return 0;
}

int s2226_get_fpga_ver(struct s2226_dev *dev)
{
	// get fpga firmware version
	int fwver = 0;
	int retval;
	retval = get_reg(dev, DEVID_FPGA, (0x21 << 1), &fwver);
	if (retval >= 0)
		printk(KERN_INFO "s2226: board_id %d\n"
		       "s2226: FPGA ver: 0x%04x\n", 
		       (fwver >> 11), (fwver & 0x07ff));
	else {
		printk(KERN_INFO "s2226: err FPGA ver %d\n", retval);
		return -1;
	}
	
	dev->fpga_ver = fwver & 0x07ff;
	dev->board_id = fwver >> 11;
	return 0;
}




/*
 *  begin V4L code
 */


#define S2226_NORMS		(V4L2_STD_PAL | V4L2_STD_NTSC)



static int s2226_open_v4l(struct file *file)
{
	int minor = video_devdata(file)->minor;
	struct s2226_dev *h, *dev = NULL;
	struct s2226_fh *fh;
	struct list_head *list;
	dprintk(1, "%s\n", __func__);

	if (s2226_mutex_lock_interruptible(&s2226_devices_lock)) {
		printk("could not acquire lock s2226_open\n");
		return -ERESTARTSYS;
	}

	list_for_each(list, &s2226_devlist) {
		h = list_entry(list, struct s2226_dev, s2226_devlist);
		if (h->vdev->minor == minor) {
			dev = h;
		}
	}

	if (NULL == dev) {
		printk(KERN_INFO "s2226: openv4l no dev\n");
		s2226_mutex_unlock(&s2226_devices_lock);
		return -ENODEV;
	}

	// allocate file context data
	fh = kmalloc(sizeof(struct s2226_fh),GFP_KERNEL);
	if (NULL == fh) {
		s2226_mutex_unlock(&s2226_devices_lock);
		return -ENOMEM;
	}
	memset(fh,0,sizeof(struct s2226_fh));
	dprintk(1, "opened dev %p\n", dev);
	file->private_data = fh;
	fh->dev            = dev;
	fh->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	// increment our usage count for the device 
	dev->users++;
	kref_get(&dev->kref);
	videobuf_queue_vmalloc_init(&fh->vb_vidq, &s2226_video_qops,
				    NULL, &dev->slock,
				    fh->type,
				    V4L2_FIELD_NONE,
				    sizeof(struct s2226_buffer), fh
				    , NULL /* ext_lock */
		);


	s2226_mutex_unlock(&s2226_devices_lock);
	return 0;
}


static int s2226_release_v4l(struct file *file)
{
	struct s2226_fh *fh = file->private_data;
	struct s2226_dev *dev = fh->dev;
	int i;
	int res;
	if (!dev)
		return -ENODEV;

	if (s2226_mutex_lock_interruptible(&s2226_devices_lock)) {
		return -ERESTARTSYS;
	}
	if (fh == NULL) {
		s2226_mutex_unlock(&s2226_devices_lock);
		return -ENODEV;
	}
	dev = fh->dev;
	if (dev == NULL) { 
		s2226_mutex_unlock(&s2226_devices_lock);
		return -ENODEV;
	}

	dev->users--;
	for (i = 0; i < WR_URBS; i++) {
		usb_kill_urb(dev->write_vid[i].urb);
		dev->write_vid[i].ready = 1;
	}
	for (i = 0; i < RD_URBS; i++) {
		usb_kill_urb(dev->read_vid[i].urb);
		dev->read_vid[i].ready = 0;
	}
	/* turn off stream */
	res = res_check(fh);
	if (res) {
		if (res & RES_STREAM) {
			if (dev->h51_state == STREAM_MODE_ENCODE)
				s2226_stop_encode(dev, 0);
			videobuf_streamoff(&fh->vb_vidq);
		}
		res_free(dev, fh, res);
	}
	videobuf_mmap_free(&fh->vb_vidq);
	kfree(fh);
	s2226_mutex_unlock(&s2226_devices_lock);
	// decrement the count on our device
	kref_put(&dev->kref, s2226_delete);
	return 0;
}


static struct v4l2_queryctrl s2226_qctrl[] = {
	{
		.id = V4L2_CID_USER_CLASS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "user class",
		.minimum = 0,
		.maximum = 0,
		.step = 0,
		.default_value = 0,
		.flags = 0,
	}, {
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Brightness",
		.minimum = 0,
		.maximum = 255,
		.step = 1,
		.default_value = 0x80,
		.flags = 0,
	}, {
		.id = V4L2_CID_CONTRAST,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Contrast",
		.minimum = 0,
		.maximum = 0x7f,
		.step = 0x1,
		.default_value = 0x40,
		.flags = 0,
	}, {
		.id = V4L2_CID_SATURATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Saturation",
		.minimum = 0,
		.maximum = 0x7f,
		.step = 0x1,
		.default_value = 0x40,
		.flags = 0,
	}, {
		.id = V4L2_CID_HUE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Hue",
		.minimum = -0x40,
		.maximum = 0x40,
		.step = 0x1,
		.default_value = 0,
		.flags = 0,
	}, {
		.id = V4L2_CID_MPEG_CLASS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "MPEG class",
		.minimum = 0,
		.maximum = 0,
		.step = 0,
		.default_value = 0,
		.flags = 0,
	}, {
		.id = V4L2_CID_MPEG_STREAM_TYPE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Stream Type",
		.minimum = V4L2_MPEG_STREAM_TYPE_MPEG2_TS,
		.maximum = V4L2_MPEG_STREAM_TYPE_MPEG2_TS,
		.step = 0,
		.default_value = V4L2_MPEG_STREAM_TYPE_MPEG2_TS,
		.flags = 0,
	}, {
		.id = V4L2_CID_MPEG_AUDIO_SAMPLING_FREQ,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Audio Sampling Frequency",
		.minimum = V4L2_MPEG_AUDIO_SAMPLING_FREQ_48000,
		.maximum = V4L2_MPEG_AUDIO_SAMPLING_FREQ_48000,
		.step = 0,
		.default_value = V4L2_MPEG_AUDIO_SAMPLING_FREQ_48000,
		.flags = 0,
	}, {
		.id = V4L2_CID_MPEG_AUDIO_ENCODING,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Audio Encoding",
		.minimum = V4L2_MPEG_AUDIO_ENCODING_LAYER_2,
		.maximum = V4L2_MPEG_AUDIO_ENCODING_LAYER_2,
		.step = 0,
		.default_value = V4L2_MPEG_AUDIO_ENCODING_LAYER_2,
		.flags = 0,
	}, {
		.id = V4L2_CID_MPEG_AUDIO_L2_BITRATE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Audio Bitrate",
		.minimum = V4L2_MPEG_AUDIO_L2_BITRATE_128K,
		.maximum = V4L2_MPEG_AUDIO_L2_BITRATE_256K,
		.step = 1,
		.default_value = V4L2_MPEG_AUDIO_L2_BITRATE_256K,
		.flags = 0,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_ENCODING,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Video Encoding",
		.minimum = V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC,
		.maximum = V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC,
		.step = 0,
		.default_value = V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC,
		.flags = 0,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_GOP_CLOSURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "GOP closure",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
		.flags = 0,
	}, {
		.id = V4L2_CID_MPEG_VIDEO_BITRATE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Video Bitrate",
		.minimum = S2226_MIN_VBITRATE * 1000,
		.maximum = S2226_MAX_VBITRATE * 1000,
		.step = 0,
		.default_value = S2226_DEF_VBITRATE * 1000,
		.flags = 0,
	}
};

static int frame_count = 0;

static int s2226_got_data(struct s2226_dev *dev, int urb_idx)
{
	struct s2226_dmaqueue *dma_q = &dev->vidq;
	struct s2226_buffer *buf;
	unsigned long flags = 0;
	char *vbuf;
	struct timeval ts;
	dprintk(4, "s2226_got_data: %p\n", &dma_q);
	spin_lock_irqsave(&dev->slock, flags);

	if (list_empty(&dma_q->active)) {
		dprintk(3, "No active queue to serve\n");
		goto unlock;
	}
	buf = list_entry(dma_q->active.next,
			 struct s2226_buffer, vb.queue);


	list_del(&buf->vb.queue);
	do_gettimeofday(&buf->vb.ts);
	dprintk(100, "[%p/%d] wakeup\n", buf, buf->vb.i);
	vbuf = videobuf_to_vmalloc(&buf->vb);
	if (vbuf == NULL) {
		printk(KERN_ERR "%s vbuf error\n", __func__);
		goto unlock;
	}

	memcpy(vbuf, dev->read_vid[urb_idx].buffer, S2226_RB_PKT_SIZE);
	buf->vb.size = S2226_RB_PKT_SIZE;
	do_gettimeofday(&ts);
	buf->vb.ts = ts;
	frame_count++;
	buf->vb.field_count+=2;
	buf->vb.state = VIDEOBUF_DONE;
	wake_up(&buf->vb.done);
	dprintk(2, "wakeup [buf/i] [%p/%d]\n", buf, buf->vb.i);
unlock:
	spin_unlock_irqrestore(&dev->slock, flags);

	return 0;
}

/*
 * Videobuf operations
 */

static int buffer_setup(struct videobuf_queue *vq, unsigned int *count,
			unsigned int *size)
{
	*size = S2226_BUFFER_SIZE;
	
	if (0 == *count)
		*count = S2226_DEF_BUFS;

	while (*size * (*count) > vid_limit * 1024 * 1024)
		(*count)--;
	
	dprintk(4, "buffer setup %d\n", *count);
	return 0;
}


static void free_buffer(struct videobuf_queue *vq, struct s2226_buffer *buf)
{
	dprintk(4, "%s\n", __func__);
	videobuf_vmalloc_free(&buf->vb);
	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

static int buffer_prepare(struct videobuf_queue *vq, struct videobuf_buffer *vb,
			  enum v4l2_field field)
{
//	struct s2226_fh *fh = vq->priv_data;
	struct s2226_buffer *buf = container_of(vb, struct s2226_buffer, vb);
	int rc;
	dprintk(4, "%s, field=%d\n", __func__, field);

	if (0 != buf->vb.baddr && buf->vb.bsize < buf->vb.size) {
		dprintk(0, "invalid buffer prepare\n");
		return -EINVAL;
	}

//	buf->fmt = fh->fmt;
	buf->vb.width = 720;//fh->width;
	buf->vb.height = 480;//fh->height;
	buf->vb.field = field;

	if (VIDEOBUF_NEEDS_INIT == buf->vb.state) {
		rc = videobuf_iolock(vq, &buf->vb, NULL);
		if (rc < 0)
			goto fail;
	}

	buf->vb.state = VIDEOBUF_PREPARED;
	return 0;
fail:
	free_buffer(vq, buf);
	return rc;
}

static void buffer_queue(struct videobuf_queue *vq, struct videobuf_buffer *vb)
{
	struct s2226_buffer *buf = container_of(vb, struct s2226_buffer, vb);
	struct s2226_fh *fh = vq->priv_data;
	struct s2226_dev *dev = fh->dev;
	struct s2226_dmaqueue *vidq = &dev->vidq;;
	dprintk(4, "%s\n", __func__);
	buf->vb.state = VIDEOBUF_QUEUED;
	list_add_tail(&buf->vb.queue, &vidq->active);
}

static void buffer_release(struct videobuf_queue *vq,
			   struct videobuf_buffer *vb)
{
	struct s2226_buffer *buf = container_of(vb, struct s2226_buffer, vb);
//	struct s2226_fh *fh = vq->priv_data;
	dprintk(4, "%s\n", __func__);
	free_buffer(vq, buf);
}

static struct videobuf_queue_ops s2226_video_qops = {
	.buf_setup = buffer_setup,
	.buf_prepare = buffer_prepare,
	.buf_queue = buffer_queue,
	.buf_release = buffer_release,
};


// now part of standard driver
static int res_get(struct s2226_dev *dev, struct s2226_fh *fh, int res)
{
	/* is it free? */
	dprintk(4, "res get\n");
	s2226_mutex_lock(&dev->reslock);
	dprintk(2, "resources %x, res %x \n", dev->resources, res);
	if (dev->resources & res) {
		/* no, someone else uses it */
		s2226_mutex_unlock(&dev->reslock);
		return 0;
	}
	/* it's free, grab it */
	dev->resources |= res;
	fh->resources |= res;
	dprintk(1, "s2226: res: get\n");
	s2226_mutex_unlock(&dev->reslock);
	return 1;
}

static int res_locked(struct s2226_dev *dev, struct s2226_fh *fh)
{
	dprintk(4, "res locked\n");
	return dev->resources;
}

static int res_check(struct s2226_fh *fh)
{
	dprintk(4, "res check %d\n", fh->resources);
	return fh->resources;
}


static void res_free(struct s2226_dev *dev, struct s2226_fh *fh, int res)
{
	dprintk(4, "res free\n");
	s2226_mutex_lock(&dev->reslock);
	dev->resources &= ~res;
	fh->resources &= ~res;
	s2226_mutex_unlock(&dev->reslock);
	dprintk(4, "res: put\n");
}

static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct s2226_fh *fh = file->private_data;
	struct s2226_dev *dev = fh->dev;
	memset(cap, 0, sizeof(*cap));

	strlcpy(cap->driver, "s2226", sizeof(cap->driver));
	strlcpy(cap->card, "s2226", sizeof(cap->card));
	strlcpy(cap->bus_info, dev_name(&dev->udev->dev),
		sizeof(cap->bus_info));
	usb_make_path(dev->udev, cap->bus_info, sizeof(cap->bus_info));
	cap->version = 0;//TODO S2226_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	int index = 0;
	dprintk(4, "%s 2226\n", __func__);
	if (f == NULL)
		return -EINVAL;

	index = f->index;

	if (index >= 1)
		return -EINVAL;
	f->index = index;
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->flags = V4L2_FMT_FLAG_COMPRESSED;
	strlcpy(f->description, "MPEGTS_H264", sizeof(f->description));
	f->pixelformat = V4L2_PIX_FMT_MPEG;
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
//	struct s2226_fh *fh = priv;
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->fmt.pix.width = 720;//fh->height;
	f->fmt.pix.height = 480;//fh->height;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat = V4L2_PIX_FMT_MPEG;
	f->fmt.pix.bytesperline = 0;
	f->fmt.pix.sizeimage = S2226_BUFFER_SIZE;
	f->fmt.pix.colorspace = 0;//V4L2_COLORSPACE_SMPTE170M ???
	f->fmt.pix.priv = 0;
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dprintk(4, "%s", __func__);
	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct s2226_fh *fh = priv;
	struct s2226_dev *dev = fh->dev;
	int is_ntsc;

	is_ntsc = !dev->v4l_is_pal;
	
	dprintk(4, "%s\n", __func__);

	if (f->fmt.pix.pixelformat != V4L2_PIX_FMT_MPEG)
		return -EINVAL;

	f->fmt.pix.pixelformat = V4L2_PIX_FMT_MPEG;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.bytesperline = 0;
	f->fmt.pix.sizeimage = S2226_BUFFER_SIZE;
	switch (dev->v4l_input) {
	case S2226_INPUT_COMPOSITE_0:
	case S2226_INPUT_COMPOSITE_1:
	case S2226_INPUT_SVIDEO_0:
	case S2226_INPUT_SVIDEO_1:
	case S2226_INPUT_SD_COLORBARS:
		f->fmt.pix.width = 720;
		f->fmt.pix.height = is_ntsc ? 480 : 576;
		break;
	case S2226_INPUT_720P_COLORBARS:
	case S2226_INPUT_SDI_720P_5994:
	case S2226_INPUT_SDI_720P_60:
	case S2226_INPUT_SDI_720P_50:
		f->fmt.pix.width = 1920;
		f->fmt.pix.height = 720;
		break;
	case S2226_INPUT_1080I_COLORBARS:
	case S2226_INPUT_SDI_1080I_5994:
	case S2226_INPUT_SDI_1080I_60:
	case S2226_INPUT_SDI_1080I_50:
		f->fmt.pix.width = 1920;
		f->fmt.pix.height = 1080;
		break;
	}
	return 0;
}

// MPEG capture device so this really has no meaning.
// There is no scalar.  accept format specified, but width and height 
// of image will be fixed in the MPEG stream.
static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct s2226_fh *fh = priv;
	int ret;
	printk("set fmt\n");
	ret = vidioc_try_fmt_vid_cap(file, fh, f);
	if (ret < 0)
		return ret;
	return 0;
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *p)
{
	int rc;
	struct s2226_fh *fh = priv;
	dprintk(4, "%s\n", __func__);
	rc = videobuf_reqbufs(&fh->vb_vidq, p);
	return rc;
}

static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	int rc;
	struct s2226_fh *fh = priv;
	dprintk(4, "%s\n", __func__);
	rc = videobuf_querybuf(&fh->vb_vidq, p);
	return rc;
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	int rc;
	struct s2226_fh *fh = priv;
	dprintk(4, "%s\n", __func__);
	rc = videobuf_qbuf(&fh->vb_vidq, p);
	return rc;
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	int rc;
	struct s2226_fh *fh = priv;
	dprintk(4, "%s\n", __func__);
	rc = videobuf_dqbuf(&fh->vb_vidq, p, file->f_flags & O_NONBLOCK);
	return rc;
}

#ifdef CONFIG_VIDEO_V4L1_COMPAT
static int vidioc_cgmbuf(struct file *file, void *priv, struct video_mbuf *mbuf)
{
	struct s2226_fh *fh = priv;
	dprintk(4, "%s\n", __func__);
	return videobuf_cgmbuf(&fh->vb_vidq, mbuf, 8);
}
#endif

static int s2226_new_v4l_input(struct s2226_dev *dev, int inp);

static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	int res;
	struct s2226_fh *fh = priv;
	struct s2226_dev *dev = fh->dev;
	dprintk(4, "%s\n", __func__);

	if (fh->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev_err(&dev->udev->dev, "invalid fh type0\n");
		return -EINVAL;
	}
	if (i != fh->type) {
		dev_err(&dev->udev->dev, "invalid fh type1\n");
		return -EINVAL;
	}

	if (!res_get(dev, fh, RES_STREAM)) {
		printk(KERN_ERR "stream busy\n");
		return -EBUSY;
	}


	res = videobuf_streamon(&fh->vb_vidq);
	if (res == 0) {
		s2226_set_attr(dev, ATTR_INPUT, dev->cur_input);
        if (dev->fpga_ver <= 0) {
            s2226_get_fpga_ver(dev);
            s2226_new_v4l_input(dev, dev->v4l_input);
        }
		s2226_start_encode(dev, 0, S2226_CONTEXT_V4L);
	} else {
		res_free(dev, fh, RES_STREAM);
	}
	return res;
}

static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct s2226_fh *fh = priv;
	struct s2226_dev *dev = fh->dev;

	dprintk(5, "%s\n", __func__);
	if (fh->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		printk(KERN_ERR "invalid fh type0\n");
		return -EINVAL;
	}
	if (i != fh->type) {
		printk(KERN_ERR "invalid type i\n");
		return -EINVAL;
	}
	if (dev->h51_state == STREAM_MODE_ENCODE)
		s2226_stop_encode(dev, 0);
	videobuf_streamoff(&fh->vb_vidq);
	res_free(dev, fh, RES_STREAM);
	return 0;
}

static int s2226_new_v4l_input(struct s2226_dev *dev, int inp)
{
	switch (inp) {
	case S2226_INPUT_COMPOSITE_0:
		s2226_new_input(dev, dev->v4l_is_pal ? INPUT_COMP0_576I : INPUT_COMP0_480I);
		break;
	case S2226_INPUT_SVIDEO_0:
		s2226_new_input(dev, dev->v4l_is_pal ? INPUT_SVIDEO0_576I : INPUT_SVIDEO0_480I);
		break;
	case S2226_INPUT_COMPOSITE_1:
		s2226_new_input(dev, dev->v4l_is_pal ? INPUT_COMP1_576I : INPUT_COMP1_480I);
		break;
	case S2226_INPUT_SVIDEO_1:
		s2226_new_input(dev, dev->v4l_is_pal ? INPUT_SVIDEO1_576I : INPUT_SVIDEO1_480I);
		break;
	case S2226_INPUT_SDI_SD:
		s2226_new_input(dev, dev->v4l_is_pal ? INPUT_SDI_576I : INPUT_SDI_480I);
		break;
	case S2226_INPUT_SD_COLORBARS:
		s2226_new_input(dev, dev->v4l_is_pal ? INPUT_SDI_576I_CB : INPUT_SDI_480I_CB);
		break;
	case S2226_INPUT_720P_COLORBARS:
		s2226_new_input(dev, dev->v4l_is_pal ? INPUT_SDI_720P_50_CB : INPUT_SDI_720P_60_CB);
		break;
	case S2226_INPUT_1080I_COLORBARS:
		s2226_new_input(dev, dev->v4l_is_pal ? INPUT_SDI_1080I_50_CB : INPUT_SDI_1080I_60_CB);
		break;
	case S2226_INPUT_SDI_720P_5994:
		s2226_new_input(dev, INPUT_SDI_720P_5994);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_720P_60:
		s2226_new_input(dev, INPUT_SDI_720P_60);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_1080I_50:
		s2226_new_input(dev, INPUT_SDI_1080I_50);
		dev->v4l_is_pal = 1;
		break;
	case S2226_INPUT_SDI_1080I_5994:
		s2226_new_input(dev, INPUT_SDI_1080I_5994);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_1080I_60:
		s2226_new_input(dev, INPUT_SDI_1080I_60);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_720P_50:
		s2226_new_input(dev, INPUT_SDI_720P_50);
		dev->v4l_is_pal = 1;
		break;
	case S2226_INPUT_SDI_720P_24:
		s2226_new_input(dev, INPUT_SDI_720P_24);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_720P_2398:
		s2226_new_input(dev, INPUT_SDI_720P_2398);
		dev->v4l_is_pal = 1;
		break;
	case S2226_INPUT_SDI_1080P_24:
		s2226_new_input(dev, INPUT_SDI_1080P_24);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_1080P_2398:
		s2226_new_input(dev, INPUT_SDI_1080P_2398);
		dev->v4l_is_pal = 1;
		break;
	default:
		return -EINVAL;
	}
	dev->v4l_input = inp;
	return 0;
}

static int vidioc_g_std(struct file *file, void *priv, v4l2_std_id *i)
{
	struct s2226_fh *fh = priv;
	struct s2226_dev *dev = fh->dev;
	*i = dev->current_norm;
	return 0;
}

static int vidioc_s_std(struct file *file, void *priv, v4l2_std_id _i)
{
	v4l2_std_id *i = &_i;
	struct s2226_fh *fh = priv;
	struct s2226_dev *dev = fh->dev;
	int ret = 0;
	if (res_locked(fh->dev, fh) & RES_STREAM) {
		dprintk(1, "can't change standard after started\n");
		return -EBUSY;
	}
	if (*i & V4L2_STD_NTSC) {
		switch (dev->v4l_input) {
		case S2226_INPUT_SDI_1080I_50:
		case S2226_INPUT_SDI_720P_50:
		case S2226_INPUT_SDI_720P_2398:
		case S2226_INPUT_SDI_1080P_2398:
			return -EINVAL;
		case S2226_INPUT_SD_COLORBARS:
			if (dev->v4l_is_pal)
				return -EINVAL;
		default:
			break;
		}
		dev->current_norm = V4L2_STD_NTSC;
	}
 	if (*i & V4L2_STD_PAL) {
		switch (dev->v4l_input) {
		case S2226_INPUT_SDI_1080I_5994:
		case S2226_INPUT_SDI_1080I_60:
		case S2226_INPUT_SDI_720P_60:
		case S2226_INPUT_SDI_720P_5994:
		case S2226_INPUT_SDI_720P_24:
		case S2226_INPUT_SDI_1080P_24:
			return -EINVAL;
		case S2226_INPUT_SD_COLORBARS:
			if (!dev->v4l_is_pal)
				return -EINVAL;
		default:
			break;
		}
		dev->current_norm = V4L2_STD_PAL;
	}
	dev->v4l_is_pal = (*i & V4L2_STD_PAL) ? 1 : 0;
	// change input based on new standard
	ret = s2226_new_v4l_input(dev, dev->v4l_input);
	printk("%s %d\n",__func__, ret);
	return ret;
}

static int vidioc_enum_input(struct file *file, void *priv,
			     struct v4l2_input *inp)
{
	if (inp->index >= S2226_INPUT_MAX)
		return -EINVAL;
	
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std = S2226_NORMS;
	inp->status = 0;
	switch (inp->index) {
	case S2226_INPUT_COMPOSITE_0:
		strlcpy(inp->name, "Composite 0", sizeof(inp->name));
		break;
	case S2226_INPUT_SVIDEO_0:
		strlcpy(inp->name, "SVideo 0", sizeof(inp->name));
		break;
	case S2226_INPUT_COMPOSITE_1:
		strlcpy(inp->name, "Composite 1", sizeof(inp->name));
		break;
	case S2226_INPUT_SVIDEO_1:
		strlcpy(inp->name, "SVideo 1", sizeof(inp->name));
		break;
	case S2226_INPUT_SDI_SD:
		strlcpy(inp->name, "SDI Input(SD)", sizeof(inp->name));
		break;
	case S2226_INPUT_SD_COLORBARS:
		strlcpy(inp->name, "SD Colorbars", sizeof(inp->name));
		break;
	case S2226_INPUT_720P_COLORBARS:
		strlcpy(inp->name, "720p Colorbars", sizeof(inp->name));
		break;
	case S2226_INPUT_1080I_COLORBARS:
		strlcpy(inp->name, "1080i Colorbars", sizeof(inp->name));
		break;
	case S2226_INPUT_SDI_720P_50:
		strlcpy(inp->name, "SDI Input(720p 50Hz PAL)", sizeof(inp->name));
		inp->std = V4L2_STD_PAL;
		break;
	case S2226_INPUT_SDI_720P_5994:
		strlcpy(inp->name, "SDI Input(720p 59.94Hz NTSC)", sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_720P_60:
		strlcpy(inp->name, "SDI Input(720p 60Hz NTSC)", sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_1080I_50:
		strlcpy(inp->name, "SDI Input(1080i 50Hz PAL)", sizeof(inp->name));
		inp->std = V4L2_STD_PAL;
		break;
	case S2226_INPUT_SDI_1080I_5994:
		strlcpy(inp->name, "SDI Input(1080i 59.94Hz NTSC)", sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_1080I_60:
		strlcpy(inp->name, "SDI Input(1080i 60Hz NTSC)", sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_720P_24:
		strlcpy(inp->name, "SDI Input(720p 24Hz NTSC)", sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_720P_2398:
		strlcpy(inp->name, "SDI Input(720p 23.97Hz PAL)", sizeof(inp->name));
		inp->std = V4L2_STD_PAL;
		break;
	case S2226_INPUT_SDI_1080P_24:
		strlcpy(inp->name, "SDI Input(1080p 24Hz NTSC)", sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_1080P_2398:
		strlcpy(inp->name, "SDI Input(1080p 23.97Hz PAL)", sizeof(inp->name));
		inp->std = V4L2_STD_PAL;
		break;
	}
	//inp->status =  (status & 0x01) ? 0
	//: V4L2_IN_ST_NO_SIGNAL;
	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct s2226_fh *fh = file->private_data;
	struct s2226_dev *dev = fh->dev;
	*i = dev->v4l_input;
	dprintk(4, "g_input\n");
	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct s2226_fh *fh = file->private_data;
	struct s2226_dev *dev = fh->dev;
	dprintk(4, "s_input %d\n", i);
	if (i >= S2226_INPUT_MAX)
		return -EINVAL;
	s2226_new_v4l_input(dev, i);
	return 0;
}

static int vidioc_enumaudio(struct file *file, void *priv,
			    struct v4l2_audio *aud)
{
	int idx;
	idx = aud->index;
	if (idx >= S2226_AUDIOINPUT_MAX)
		return -EINVAL;
	memset(aud, 0, sizeof(struct v4l2_audio));
	aud->index = idx;
	switch (idx) {
	case S2226_AUDIOINPUT_LINE:
		strcpy(aud->name, "Line In");
		break;
	case S2226_AUDIOINPUT_TONE:
		strcpy(aud->name, "Test Tone");
		break;
	case S2226_AUDIOINPUT_SDI:
		strcpy(aud->name, "SDI Embedded Audio");
		break;
	}
	return 0;
}


static int vidioc_g_audio(struct file *file, void *priv,
			  struct v4l2_audio *aud)
{
	struct s2226_fh *fh = file->private_data;
	struct s2226_dev *dev = fh->dev;
	memset(aud, 0, sizeof(struct v4l2_audio));
	switch (dev->cur_audiompeg) {
	case S2226_AUDIOINPUT_LINE:
		strcpy(aud->name, "Line In");
		break;
	case S2226_AUDIOINPUT_TONE:
		strcpy(aud->name, "Test Tone");
		break;
	case S2226_AUDIOINPUT_SDI:
		strcpy(aud->name, "SDI Embedded Audio");
		break;
	}
	aud->index = dev->cur_audiompeg;
	return 0;
}

static int vidioc_s_audio(struct file *file, void *priv,
			  const struct v4l2_audio *aud)
{
	struct s2226_fh *fh = file->private_data;
	struct s2226_dev *dev = fh->dev;
	if (aud->index >= S2226_AUDIOINPUT_MAX)
		return -EINVAL;
	dev->cur_audiompeg = aud->index;
	(void) s2226_set_audiomux_mpegin(dev, dev->cur_audiompeg);
	return 0;
}



/* --- controls ---------------------------------------------- */
static int mpeg_queryctrl(u32 id, struct v4l2_queryctrl *ctrl)
{
	int i;
	// must be in increasing order
	static const u32 mpeg_ctrls[] = {
		V4L2_CID_MPEG_CLASS,
		V4L2_CID_MPEG_STREAM_TYPE,
		V4L2_CID_MPEG_AUDIO_SAMPLING_FREQ,
		V4L2_CID_MPEG_AUDIO_ENCODING,
		V4L2_CID_MPEG_AUDIO_L2_BITRATE,
		V4L2_CID_MPEG_VIDEO_ENCODING,
		V4L2_CID_MPEG_VIDEO_GOP_CLOSURE,
		V4L2_CID_MPEG_VIDEO_BITRATE,
		0
	};
	static const u32 *ctrl_classes[] = {
		mpeg_ctrls,
		NULL
	};

	/* The ctrl may already contain the queried i2c controls,
	 * query the mpeg controls if the existing ctrl id is
	 * greater than the next mpeg ctrl id.
	 */
	id = v4l2_ctrl_next(ctrl_classes, id);
	if (id >= ctrl->id && ctrl->name[0])
		return 0;

	memset(ctrl, 0, sizeof(*ctrl));
	ctrl->id = id;

	for (i = 0; i < ARRAY_SIZE(s2226_qctrl); i++)
		if (ctrl->id && ctrl->id == s2226_qctrl[i].id) {
			// don't just copy.  use query fill to set the V4L name
			// correctly based on kernel function
			v4l2_ctrl_query_fill(ctrl,
					     s2226_qctrl[i].minimum,
					     s2226_qctrl[i].maximum,
					     s2226_qctrl[i].step,
					     s2226_qctrl[i].default_value);
			return 0;
		}

	return -EINVAL;
}


static int vidioc_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *qc)
{
	int id;
	int i;
	id = qc->id;
	for (i = 0; i < ARRAY_SIZE(s2226_qctrl); i++)
		if (qc->id && qc->id == s2226_qctrl[i].id) {
			memcpy(qc, &(s2226_qctrl[i]), sizeof(*qc));
			return 0;
		}

	if (id & V4L2_CTRL_FLAG_NEXT_CTRL || qc->name[0] == 0)
		return mpeg_queryctrl(id, qc);
	return -EINVAL;
}

static int vidioc_querymenu(struct file *file, void *priv,
			    struct v4l2_querymenu *qmenu)
{
	return v4l2_ctrl_query_menu(qmenu, NULL, NULL);
}

static int vidioc_g_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct s2226_fh *fh = file->private_data;
	struct s2226_dev *dev = fh->dev;

	switch (ctrl->id) {
	case V4L2_CID_MPEG_STREAM_TYPE:
		ctrl->value = V4L2_MPEG_STREAM_TYPE_MPEG2_TS;
		break;
	case V4L2_CID_MPEG_VIDEO_ENCODING:
		ctrl->value = V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC;
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_CLOSURE:
		ctrl->value = dev->closed_gop;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ctrl->value = dev->h51_mode.vBitrate * 1000;
		break;
	case V4L2_CID_MPEG_AUDIO_ENCODING:
		ctrl->value = V4L2_MPEG_AUDIO_ENCODING_LAYER_2;
		break;
	case V4L2_CID_MPEG_AUDIO_SAMPLING_FREQ:
		ctrl->value = V4L2_MPEG_AUDIO_SAMPLING_FREQ_48000;
		break;
	case V4L2_CID_MPEG_AUDIO_L2_BITRATE:
		switch (dev->h51_mode.aBitrate) {
		case 128:
			ctrl->value = V4L2_MPEG_AUDIO_L2_BITRATE_128K;
			break;
		case 160:
			ctrl->value = V4L2_MPEG_AUDIO_L2_BITRATE_160K;
			break;
		case 192:
			ctrl->value = V4L2_MPEG_AUDIO_L2_BITRATE_192K;
			break;
		case 224:
			ctrl->value = V4L2_MPEG_AUDIO_L2_BITRATE_224K;
			break;
		case 256:
			ctrl->value = V4L2_MPEG_AUDIO_L2_BITRATE_256K;
			break;
		default:
			printk(KERN_INFO "unknown audio bitrate specified.\n");
			break;
		}
		break;
	case V4L2_CID_BRIGHTNESS:
		ctrl->value = dev->brightness;
		break;
	case V4L2_CID_CONTRAST:
		ctrl->value = dev->contrast;
		break;
	case V4L2_CID_SATURATION:
		ctrl->value = dev->saturation;
		break;
	case V4L2_CID_HUE:
		ctrl->value = dev->hue;
		break;
	default:
		return -EINVAL;
	}
	return 0;

}

static int vidioc_s_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	int i;
	struct s2226_fh *fh = file->private_data;
	struct s2226_dev *dev = fh->dev;
	printk("set ctrl %d %d\n", ctrl->id, V4L2_CID_MPEG_VIDEO_BITRATE);
	// verify range
	for (i = 0; i < ARRAY_SIZE(s2226_qctrl); i++) {
		if (ctrl->id == s2226_qctrl[i].id) {
			if (ctrl->value < s2226_qctrl[i].minimum ||
			    ctrl->value > s2226_qctrl[i].maximum)
				return -ERANGE;
		}
	}
	switch (ctrl->id) {
	case V4L2_CID_MPEG_STREAM_TYPE:
		break;
	case V4L2_CID_MPEG_VIDEO_ENCODING:
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_CLOSURE:
		dev->closed_gop = ctrl->value;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		dev->h51_mode.vBitrate = ctrl->value / 1000;
		dprintk(4, "setting vBitrate to %d kbps\n", dev->h51_mode.vBitrate);
		break;
	case V4L2_CID_MPEG_AUDIO_L2_BITRATE:
		switch (ctrl->value) {
		case V4L2_MPEG_AUDIO_L2_BITRATE_128K:
			dev->h51_mode.aBitrate = 128;
			break;
		case V4L2_MPEG_AUDIO_L2_BITRATE_160K:
			dev->h51_mode.aBitrate = 160;
			break;
		case V4L2_MPEG_AUDIO_L2_BITRATE_192K:
			dev->h51_mode.aBitrate = 192;
			break;
		case V4L2_MPEG_AUDIO_L2_BITRATE_224K:
			dev->h51_mode.aBitrate = 224;
			break;
		case V4L2_MPEG_AUDIO_L2_BITRATE_256K:
			dev->h51_mode.aBitrate = 256;
			break;
		default:
			printk(KERN_INFO "invalid audio bitrate. using default 256k\n");
			dev->h51_mode.aBitrate = 256;
			break;
		}
		break;
	case V4L2_CID_BRIGHTNESS:
		s2226_set_brightness(dev, ctrl->value);
		break;
	case V4L2_CID_CONTRAST:
		s2226_set_contrast(dev, ctrl->value);
		break;
	case V4L2_CID_SATURATION:
		s2226_set_saturation(dev, ctrl->value);
		break;
	case V4L2_CID_HUE:
		s2226_set_hue(dev, ctrl->value);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static unsigned int s2226_poll_v4l(struct file *file,
				   struct poll_table_struct *wait)
{
	struct s2226_fh *fh = file->private_data;
	int rc;
	dprintk(4, "%s\n", __func__);
	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != fh->type)
		return POLLERR;
	rc = videobuf_poll_stream(file, &fh->vb_vidq, wait);
	dprintk(4, "%s return 0x%x\n", __func__, rc);
	return rc;
}

static int s2226_mmap_v4l(struct file *file, struct vm_area_struct *vma)
{
	struct s2226_fh *fh = file->private_data;
	struct videobuf_queue *q = &fh->vb_vidq;
	int ret;
	dprintk(4, "%s\n", __func__);
	if (!fh)
		return -ENODEV;
	dprintk(4, "mmap called, vma=0x%08lx\n", (unsigned long)vma);
	// workaround deadlock bug introduced in 3.11 (in videobuf)
	//https://www.mail-archive.com/linux-media@vger.kernel.org/msg68345.html
	//Note: we will migrate to videobuf2 soon.
	mutex_lock(&q->vb_lock);
	q->ext_lock = (struct mutex *) 1;
	dprintk(4, "mmap called, vma=0x%08lx\n", (unsigned long)vma);
	ret = videobuf_mmap_mapper(q, vma);
	q->ext_lock = NULL;
	mutex_unlock(&q->vb_lock);
    
	dprintk(4, "vma start=0x%08lx, size=%ld, ret=%d\n",
		(unsigned long)vma->vm_start,
		(unsigned long)vma->vm_end - (unsigned long)vma->vm_start, ret);
	return ret;
}



static const struct v4l2_file_operations s2226_fops_v4l = {
	.owner = THIS_MODULE,
	.open = s2226_open_v4l,
	.release = s2226_release_v4l,
	.poll = s2226_poll_v4l,
	.ioctl = video_ioctl2,
	.mmap = s2226_mmap_v4l,
};

static const struct v4l2_ioctl_ops s2226_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = vidioc_s_fmt_vid_cap,
	.vidioc_reqbufs = vidioc_reqbufs,
	.vidioc_querybuf = vidioc_querybuf,
	.vidioc_qbuf = vidioc_qbuf,
	.vidioc_dqbuf = vidioc_dqbuf,
	.vidioc_s_std = vidioc_s_std,
	.vidioc_g_std = vidioc_g_std,
	.vidioc_enum_input = vidioc_enum_input,
	.vidioc_g_input = vidioc_g_input,
	.vidioc_s_input = vidioc_s_input,
	.vidioc_queryctrl = vidioc_queryctrl,
	.vidioc_querymenu = vidioc_querymenu,
	.vidioc_g_ctrl = vidioc_g_ctrl,
	.vidioc_s_ctrl = vidioc_s_ctrl,
	.vidioc_streamon = vidioc_streamon,
	.vidioc_streamoff = vidioc_streamoff,
	.vidioc_enumaudio = vidioc_enumaudio,
	.vidioc_s_audio = vidioc_s_audio,
	.vidioc_g_audio = vidioc_g_audio,
#ifdef CONFIG_VIDEO_V4L1_COMPAT
	.vidiocgmbuf = vidioc_cgmbuf,
#endif
};

static struct video_device template = {
	.name = "s2226v4l",
	.fops = &s2226_fops_v4l,
	.ioctl_ops = &s2226_ioctl_ops,
	.minor = -1,
	.release = video_device_release,
	.tvnorms = S2226_NORMS,
};

static int s2226_probe_v4l(struct s2226_dev *dev)
{
	int ret;
	int cur_nr = video_nr;
	list_add_tail(&dev->s2226_devlist, &s2226_devlist);
	INIT_LIST_HEAD(&dev->vidq.active);

	spin_lock_init(&dev->slock);
	s2226_mutex_init(&dev->reslock);

	dev->vidq.dev = dev;
	/* register video device */
	dev->vdev = video_device_alloc();
	memcpy(dev->vdev, &template, sizeof(struct video_device));
	dev->vdev->v4l2_dev = &dev->v4l2_dev;
	ret = v4l2_device_register(&dev->interface->dev, &dev->v4l2_dev);
	if (ret) {
		printk(KERN_ERR "s2226: could not register device\n");
		return ret;
	}

	if (video_nr == -1)
		ret = video_register_device(dev->vdev,
					    VFL_TYPE_GRABBER,
					    video_nr);
	else
		ret = video_register_device(dev->vdev,
					    VFL_TYPE_GRABBER,
					    cur_nr);
	video_set_drvdata(dev->vdev, dev);
	dev->current_norm = V4L2_STD_NTSC_M;
	
	if (ret != 0) {
		dev_err(&dev->udev->dev,
			"failed to register video device!\n");
		v4l2_device_unregister(&dev->v4l2_dev);
		return ret;
	}
	printk(KERN_INFO "Sensoray 2226 V4L driver \n"); //todo add revision
	return ret;
}



static void s2226_exit_v4l(struct s2226_dev *dev)
{
	struct list_head *list;
	v4l2_device_disconnect(&dev->v4l2_dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	if (-1 != dev->vdev->minor) {
		video_unregister_device(dev->vdev);
		printk(KERN_INFO "s2226 unregistered\n");
	} else {
		video_device_release(dev->vdev);
		printk(KERN_INFO "s2226 released\n");
	}
	while (!list_empty(&s2226_devlist)) {
		list = s2226_devlist.next;
		list_del(list);
	}
}




/*
 * End V4L code
 */


    
module_init (usb_s2226_init);
module_exit (usb_s2226_exit);

MODULE_DESCRIPTION("Sensoray 2226 Linux driver: Version 1.03(Copyright 2008-2012)");
MODULE_LICENSE("GPL");



// AIC33 sanity checking
static int s2226_aic33_check(u8 page, u8 reg, u8 val)
{
	int rc = 0;
	if (page == 1) {
		dprintk(2, "%s page 1\n", __func__);
		return 0; // no checking for page 1 yet
	}
	switch (reg) {
	default:
		break;
	case 0:
		if (val & 0xfe)
			rc = -EINVAL;
		break;
	case 1:
		if (val & 0x7f)
			rc = -EINVAL;
		break;
	case 6:
	case 95:
		if (val & 0x03)
			rc = -EINVAL;
		break;
	case 40:
		if ((val & 0x03) == 0x03)
			rc = -EINVAL;
		break;
	case 41:
		if (((val >> 6) & 0x03) == 0x03)
			rc = -EINVAL;
		if (((val >> 4) & 0x03) == 0x03)
			rc = -EINVAL;
		if (((val >> 0) & 0x03) == 0x03)
			rc = -EINVAL;
		if ((val >> 2) & 0x03)
			rc = -EINVAL;
		break;
	case 42:
		if ((val >> 4) >= 0x0c)
			rc = -EINVAL;
		if (val & 1)
			rc = -EINVAL;
		break;
	case 7:
	case 94:
		if (val & 1)
			rc = -EINVAL;
		break;
	case 99:
		if (((val >> 4) & 1) == 1)
			rc = -EINVAL;
		if (val & 1)
			rc = -EINVAL;
		break;
	case 13:
		if (((val >> 2) & 0x07) >= 6)
			rc = -EINVAL;
		break;
	case 14:
		if (val & 0x07)
			rc = -EINVAL;
		// do not set bits D3 and D6 high at same time
		if ((val & 0x08) && (val & 0x20))
			rc = -EINVAL;
		break;
	case 21:
		if ((((val >> 3) & 0x0f) >= 9) && 
		    (((val >> 3) & 0x0f) < 15))
			rc = -EINVAL;
		if (val & 0x07)
			rc = -EINVAL;
		break;
	case 17:
	case 18:
		if ((((val >> 4) & 0x0f) >= 9) && 
		    (((val >> 4) & 0x0f) < 15))
			rc = -EINVAL;
		if ( ((val & 0x0f) >= 9) && 
		     ((val & 0x0f) < 15) )
			rc = -EINVAL;
		break;
	case 19:
	case 20:
	case 22:
		if ((((val >> 3) & 0x0f) >= 9) && 
		    (((val >> 3) & 0x0f) < 15))
			rc = -EINVAL;
		break;
	case 23:
		if (val & 0x03)
			rc = -EINVAL;
		if ((((val >> 3) & 0x0f) >= 9) && 
		    (((val >> 3) & 0x0f) < 15))
			rc = -EINVAL;
		break;
	case 24:
		if (val & 0x07)
			rc = -EINVAL;
		if ((((val >> 3) & 0x0f) >= 9) && 
		    (((val >> 3) & 0x0f) < 15))
			rc = -EINVAL;
		break;
	case 37:
		if (val & 0x07)
			rc = -EINVAL;
		if (((val >> 4) & 0x03) == 0x03)
			rc = -EINVAL;
		break;
	case 38:
		if ((val & 0x01) || (val & 0xc0))
			rc = -EINVAL;
		if (((val >> 3) & 0x07) >= 5)
			rc = -EINVAL;
		break;
	case 39:
		rc = -EINVAL;
		break;
	case 51:
	case 58:
	case 65:
	case 72:
		if ((val >> 4) >= 0x0a)
			rc = -EINVAL;
		break;
	case 86:
	case 79:
		if ((val >> 4) >= 0x0a) // D7-D4 (1010-1111 reserved)
			rc = -EINVAL;
		//fall thru
	case 93:
		if (val & 0x04) // write only zero to D2
			rc = -EINVAL;
		break;
	case 100:
		if (((val >> 2) & 0x03) == 0x03) // DO not write 11 to D3-D2
			rc = -EINVAL;
		break;

	}
	if (rc != 0)
		printk("%s: invalid write [page %d] register %d of val 0x%x\n", __func__, page, reg, val);
	return rc;
}
