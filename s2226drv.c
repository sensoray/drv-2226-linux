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
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <linux/videodev2.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>

/* update with each new version for easy detection of driver */
#define S2226_VERSION "1.0.9"


#include "s2226mod.h"
#include "s2226ioctl.h"
#include "h51reg.h"
#include "h51reg_dec.h"
#include "dh2226.h"
#define S2226_MPEG_URB_SIZE 8192
#define S2226_PREVIEW_URB_SIZE (16*1024)
#define WR_URBS		  4
#define RD_URBS		  4

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
#define S2226_CONTEXT_USBDEV 0  /* for /dev/s2226vX */
#define S2226_CONTEXT_V4L    1  /* for /dev/videoX */

#define DRIVERNAME "s2226"

#define USB_S2226_VENDOR_ID   0x1943
#define USB_S2226_PRODUCT_ID  0x2226

#define INDEX_EP_H264 0    /* video in, video out.  endpoint index */
#define INDEX_EP_CONTROL 1 /* control messages */
#define INDEX_EP_RESP    2 /* responses */
#define INDEX_EP_RAW     3 /* raw pipe (USB FW 0x20+ only) */



#define s2226_mutex_init     mutex_init
#define s2226_mutex_lock     mutex_lock
#define s2226_mutex_lock_interruptible mutex_lock_interruptible
#define s2226_mutex_unlock   mutex_unlock

/*
 * USB interrupt message types. All intmsg's have one of these types.
 */
#define INTTYPE_CMDREPLY 1 /* Response to usb host command. */
#define INTTYPE_ASYNCEVENT 2 /* asynchronous event notification. */

/* Types of async events found in interrupt messages. */
#define ASYNCEVENT_VSYNC 1 /* VSYNC detected. */
#define ASYNCEVENT_H51_MSG 2 /* H51 issued an unsolicited message. */
#define ASYNCEVENT_SYSERROR 3 /* H51 internal error. */
#define ASYNCEVENT_RAWDATA 4
#define ASYNCEVENT_RAWDATA_END 5
#define ASYNCEVENT_SNAPSHOT_START 6
#define ASYNCEVENT_SNAPSHOT_END 7

/* default preview scales */
#define S2226_DEF_PREVIEW_X 320
#define S2226_DEF_PREVIEW_Y 240

/* audio muxing */
#define AMUX_MPEG_IN_LINE_IN 0   /* MPEG-In  gets Line-In */
#define AMUX_MPEG_IN_TONE 1      /* MPEG-In  gets Tone Test */
#define AMUX_MPEG_IN_SDI_IN 2    /* MPEG-In  gets SDI-In embedded audio*/
#define AMUX_LINE_OUT_LINE_IN 0  /* Line-Out gets Line-In */
#define AMUX_LINE_OUT_MPEG_OUT 1 /* Line-Out gets MPEG-Out */
#define AMUX_LINE_OUT_SDI_IN 2   /* Line-Out gets SDI-In embedded audio */
#define AMUX_SDI_OUT_LINE_IN 0   /* SDI-Out  gets Line-In */
#define AMUX_SDI_OUT_MPEG_OUT 1  /* SDI-Out  gets MPEG-Out */
#define AMUX_SDI_OUT_TONE 2      /* SDI-Out  gets Tone Test */
#define AMUX_SDI_OUT_SDI_IN 3    /* SDI-Out  gets SDI-In embedded audio */



/* table of devices that work with this driver */
static struct usb_device_id s2226_table[] = {
	{ USB_DEVICE(USB_S2226_VENDOR_ID, USB_S2226_PRODUCT_ID) },
	{ }                                     /* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, s2226_table);

/* Get a minor range for your devices from the usb maintainer */
#define USB_S2226_MINOR_BASE    0

static int debug;
static int filter_mode; /* 0 = autodetect, 1=fixed */
int *s2226_debug = &debug;
int *s2226_filter_mode = &filter_mode;


static int majorv = 126;
static int video_nr = -1;	/* /dev/videoN, -1 for autodetect */

module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level(0-4)");
module_param(filter_mode, int, 0);
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
module_param(audiomode, int, 0);
MODULE_PARM_DESC(audiomode, "Audiomode (0-manual, 1-auto)");

/* altboard is for 2226 units (possibly for 2446 streaming product) 
 * with alternate configurations (non-2226S).  It allows configuration
 * of all audio outputs and whether input audio is balanced or not
 */
static int altboard = 0;
module_param(altboard, int, 0);
MODULE_PARM_DESC(altboard, "Alternate board.  for non-2226S boards configuration  (0-off default).");

/* mfgmode for internal use only.  Will void warranty if mis-used */
static int mfgmode = 1;
module_param(mfgmode, int, 0);
MODULE_PARM_DESC(mfgmode, "Mfg mode (0-off default) internal use.  do not change");


module_param(majorv, int, 0);
MODULE_PARM_DESC(majorv, "major for video devname");
module_param(video_nr, int, 0644);
MODULE_PARM_DESC(video_nr, "start video minor(-1 default autodetect)");

#define dprintk(level, fmt, arg...)					\
	do {								\
		if (*s2226_debug >= (level)) {				\
			pr_debug("s2226: " fmt, ##arg);	\
		}							\
	} while (0)

static struct usb_driver s2226_driver;

#define MAX_CHANNELS 5

struct s2226_dev;

static void s2226_read_preview_callback(struct urb *u);

#define AUDIOROUTE_LINE1L 0
#define AUDIOROUTE_LINE2L 1
#define AUDIOROUTE_LINE1L_BYPASS 2
#define AUDIOROUTE_LINE2L_BYPASS 3

static int SetAudioIn(struct s2226_dev *dev);
static int SetAudioRoute(struct s2226_dev *dev, int route);
static int SetAudioOut(struct s2226_dev *dev);
static int SetAudioLeftGain(struct s2226_dev *dev, int gain);
static int SetAudioRightGain(struct s2226_dev *dev, int gain);
static int SetAudioRightAGC(struct s2226_dev *dev, int bOn, int gain);
static int SetAudioLeftAGC(struct s2226_dev *dev, int bOn, int gain);
static int SetAudioBalR(struct s2226_dev *dev, int bBal);
static int SetAudioBalL(struct s2226_dev *dev, int bBal);
static int s2226_set_audiomux_mpegin(struct s2226_dev *dev, int aud);
static int s2226_set_audiomux_lineout(struct s2226_dev *dev, int aud);
static int s2226_set_audiomux_sdiout(struct s2226_dev *dev, int aud);
#define S2226_STREAM_MPEG 0 /* MPEG H.264 in MPEG-TS container */

/* 2226 raw preview video stream. downscaled for USB2.0 */
#define S2226_STREAM_PREVIEW 1

/* decode stream.  may not be used at same time as mpeg stream */
#define S2226_STREAM_DECODE 2

struct s2226_dev;

/* inputs for MPEG encoding and preview */
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

/* inputs for MPEG decode */
#define S2226_DECODE_480I       0
#define S2226_DECODE_576I       1
#define S2226_DECODE_1080I_60   2
#define S2226_DECODE_1080I_5994 3
#define S2226_DECODE_1080I_50   4
#define S2226_DECODE_720P_60    5
#define S2226_DECODE_720P_5994  6
#define S2226_DECODE_720P_50    7
#define S2226_DECODE_720P_24    8
#define S2226_DECODE_720P_2398  9
#define S2226_DECODE_1080P_24   10
#define S2226_DECODE_1080P_2398 11
#define S2226_DECODE_MAX        12

/* MPEG audio source */
#define    S2226_AUDIOINPUT_LINE       0
#define    S2226_AUDIOINPUT_TONE       1 /* test tone */
#define    S2226_AUDIOINPUT_SDI        2 /* SDI-IN embedded audio */
#define    S2226_AUDIOINPUT_MAX        3

/* maximum size of URB buffers */
#define MAX_USB_SIZE (16*1024)
#define MAX_USB_INT_SIZE 512 /* interrupt endpoint */

#define S2226_MPEG_BUFFER_SIZE (S2226_MPEG_URB_SIZE)
#define S2226_MIN_BUFS    10

#define S2226_MIN_BUFS_PR 3 /* minimum number of preview bufs */

/* buffer for one video frame */
struct s2226_buffer {
	/* common v4l buffer stuff -- must be first */
	struct vb2_buffer vb;
	struct list_head list;
};

struct s2226_fh {
	struct s2226_dev *dev;
	struct s2226_stream *strm;
	unsigned int resources;	/* resource management for device open */
	int type;
	int is_mpeg;
};




struct s2226_dev;
struct s2226_urb {
	struct s2226_dev *dev;
	struct s2226_stream *strm;
	struct urb *urb;
	int ready;
	int context;
	void *buffer;
};

struct s2226_dmaqueue {
	struct list_head active;
	struct s2226_dev *dev;
};

/* 2226 stream */
struct s2226_stream {
	int type;
	int active;
	int height;
	int fourcc;
	int width;
	int field;
	int single;
	unsigned char *f1; /* tmp buffer for merging frames */
	struct video_device vdev;
	struct s2226_urb write_vid[WR_URBS];
	struct s2226_urb read_vid[RD_URBS];
	int			write_vid_ready;
	wait_queue_head_t       write_vid_wq;
	struct s2226_dev *dev;
	int m_sync; /* for raw video */
	int m_height;
	int m_width;
	int m_padding;
	int m_lines;
	int m_pos;
	int m_copied;
	int framecount;
	spinlock_t qlock;
	struct vb2_queue vb_vidq;
	struct vb2_queue *queue; /* pointer to vb_vidq */
	struct list_head buf_list;
	struct mutex vb_lock;
};

struct s2226_audio {
	int bAGC_R; /* if AGC on or off*/
	int bAGC_L; /* if AGC on or off*/
	int in_balR; /* if balanced or not */
	int in_balL; /* if balanced or not */
	int iAGCRightGain;
	int iAGCLeftGain;
	int iRightGain;
	int iLeftGain;
	/* audio output gains and balance settings */
	int iVolGainR;
	int iVolGainL;
	int bVolMuteR;
	int bVolMuteL;

	int iMonoGain;
	int bMonoMute;
	int iStereoGainR;
	int iStereoGainL;
	int bStereoMuteR;
	int bStereoMuteL;
	int iHpGainR;
	int iHpGainL;
	int bHpMuteR;
	int bHpMuteL;

	int iRoute;
};


#define S2226_MAX_STREAMS 3
#define MAX_ENDPOINTS 4

/* 2226 device */
struct s2226_dev {
	struct s2226_stream strm[S2226_MAX_STREAMS];
	struct s2226_stream *m; /* shortcut to strm[S2226_STREAM_MPEG] encode */
	struct s2226_stream *p; /* shortcut to strm[S2226_STREAM_PREVIEW] */
	struct s2226_stream *d; /* shortcut to strm[S2226_STREAM_DECODE] */
	struct v4l2_device v4l2_dev;
	int			users;
	int			debug;
	struct mutex ioctl_lock;
	struct mutex            audlock;
	struct mutex            reslock;
	struct usb_device	*udev;
	struct usb_interface	*interface;
	int			h51_state;
	int			persona;
	void			*interrupt_buffer;
	void			*control_buffer;
	struct urb		*interrupt_urb;
	struct urb		*control_urb;
	int			interrupt_ready;
	int			control_ready;
	struct MODE2226		h51_mode;
	int			cur_input;

	int                     cur_audio_mpeg; /* current audio input */
	int                     cur_audio_lineout;
	int                     cur_audio_sdiout;

	int                     is_decode; /* set to decode input */
	int                     input_set;
	unsigned                id; /* message id */
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
	int ovl_res;
	struct file *ovl_in_use_by;
	struct mutex            lock;
	int                     v4l_is_pal; /* is PAL standard */
	int                     v4l_input;
	spinlock_t		slock;
	v4l2_std_id		current_norm;
	struct s2226_audio      aud;
	int h51_reload_required;
	int usb_ver;
	int ep[MAX_ENDPOINTS]; /* endpoint address */
	int audio_page;
	int cfg_intf;
	int alt_intf;
};


/* function prototypes (forwards)*/
#define RES_ENCODE  1
#define RES_PREVIEW 2
#define RES_DECODE  4
#define RES_STREAM (RES_ENCODE | RES_PREVIEW | RES_DECODE)
#define RES_ALL  (RES_STREAM)

static int s2226_new_v4l_input(struct s2226_dev *dev, int inp);
static int s2226_new_v4l_input_decode(struct s2226_dev *dev, int inp);
static int write_control_endpoint(struct s2226_dev *dev, int timeout);
static int read_interrupt_endpoint(struct s2226_dev *dev, int timeout);
static int s2226_get_attr(struct s2226_dev *dev, int attr, int *value);
extern int setH51regs(struct MODE2226 *mode);
static void s2226_read_vid_callback(struct urb *u);
static int s2226_vendor_request(void *pdev, unsigned char req,
			 unsigned short idx, unsigned short val,
			 void *pbuf, unsigned int len,
			 int bOut);
static int send_sdi_write(struct s2226_dev *dev, unsigned char addr,
			  unsigned char val, int bIn);
static int send_persona_run(struct s2226_dev *dev, unsigned char persona);
static int send_persona_halt(struct s2226_dev *dev, unsigned char index, unsigned int fw);
static int s2226_flush_in_ep(struct s2226_dev *dev);
static int s2226_set_interface(struct s2226_dev *dev, int cfg, int alt, int force);

/*
 * for controlling which firmware is loaded.  see comments for
 * S2226_IOC_SET_BASEFW and S2226_IOC_SET_NEWFW in ioctl file.
 */
static int s2226_fx2sam(struct s2226_dev *dev, int bNewFw);

/*
 * resets the ARM CPU to bring up all board devices(h51, FPGA, ARM)
 * except the FX2 usb chip in a fresh state
 */
static int s2226_reset_board(struct s2226_dev *dev);

static int s2226_default_params(struct s2226_dev *dev);
static int s2226_probe_v4l(struct s2226_dev *dev);
static int s2226_got_data(struct s2226_stream *strm, unsigned char *buf, unsigned int len);
static int s2226_got_preview_data(struct s2226_stream *strm, unsigned char *buf, unsigned int len);

static struct vb2_ops s2226_vb2_ops;

static int s2226_allocate_urbs(struct s2226_stream *strm);

static int ovl_get(struct s2226_dev *dev, struct file *file)
{
	/* is it free? */
	dprintk(4, "res get\n");
	s2226_mutex_lock(&dev->reslock);
	if (dev->ovl_res) {
		/* no, someone else uses it */
		s2226_mutex_unlock(&dev->reslock);
		return 0;
	}
	/* it's free, grab it */
	dev->ovl_res |= 1;
	dev->ovl_in_use_by = file;
	s2226_mutex_unlock(&dev->reslock);
	return 1;
}

static void ovl_free(struct s2226_dev *dev)
{
	dprintk(4, "res free\n");
	s2226_mutex_lock(&dev->reslock);
	dev->ovl_res = 0;
	dev->ovl_in_use_by = NULL;
	s2226_mutex_unlock(&dev->reslock);
	dprintk(4, "res: put\n");
}

static int ovl_check(struct s2226_dev *dev, struct file *filep)
{
	s2226_mutex_lock(&dev->reslock);
	if (!(dev->ovl_res)) {
		s2226_mutex_unlock(&dev->reslock);
		return 0;
	}
	if (dev->ovl_in_use_by == filep) {
		s2226_mutex_unlock(&dev->reslock);
		return 1;
	}
	s2226_mutex_unlock(&dev->reslock);
	return 0;
}

static int res_get(struct s2226_dev *dev, struct s2226_fh *fh, int res)
{
	/* is it free? */
	dprintk(4, "res get\n");
	s2226_mutex_lock(&dev->reslock);
	dprintk(2, "resources %x, res %x\n", dev->resources, res);
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

static unsigned char compute_checksum(unsigned char *pdata, int len)
{
	int i;
	unsigned char chksum = 0;
	for (i = 0; i < len; i++)
		chksum += pdata[i];
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

/* this function must be called with cmdlock locked */
static int s2226_send_msg_noresp(struct s2226_dev *dev)
{
	int rc;
	rc = write_control_endpoint(dev, S2226_BULKMSG_TO);
	return rc;
}

/* this function must be called with cmdlock locked
   timeout is time to wait for response in ms */
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
	if (rc < 0)
		return rc;
	/* normal reply structure */
	/*
	  buf[0]: INTYPE_CMDREPLY==1
	  buf[1]: opcode
	  buf[2]: echo ID
	  buf[3..]: response
	*/
	buf = (unsigned char *) dev->interrupt_buffer;
	if (buf[0] != INTTYPE_CMDREPLY) {
		rc = read_interrupt_endpoint(dev, timeout);
		if (rc < 0)
			return rc;
	}
	if (buf[0] != INTTYPE_CMDREPLY)
		return -EAGAIN;
	/* some error replies
	   buf[0]: INTYPE_CMDREPLY==1
	   buf[1]: 0  (same as NOP opcode, so check for NOP below)
	   buf[2]: reason */
	if (buf[1] == 0 && buf[0] == 1 && cmd != 0) {
		s2226_display_err(buf[2]);
		return S2226_ERR_BASE - buf[2];
	}
	if ((buf[1] != cmd) || (buf[2] != id)) {
		dev_info(&dev->udev->dev,
			 "command mismatch, re-reading: %x:%x, %x:%x\n",
			 buf[1], cmd, buf[2], id);
		dev_info(&dev->udev->dev,
			 "%x %x %x %x %x %x\n",
			 buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		/* wait for full timeout.  if mismatch, endpoint was
		   already populated */
		rc = read_interrupt_endpoint(dev, timeout);
		if (rc < 0) {
			dev_info(&dev->udev->dev,
				 "%s failed retry: %d\n", __func__, rc);
			return rc;
		}
	}
	if ((buf[1] != cmd) || (buf[2] != id)) {
		dev_info(&dev->udev->dev,
			 "command mismatch\n");
		return -EAGAIN;
	}
	if (buf[3] != 0) {
		s2226_display_err(buf[3]);
		return S2226_ERR_BASE - buf[3];
	}
	/* rc must be 0 to get here */
	return 0;
}

/* assumes i counter initialized to 0 */
#define fill_cmd_buf(buf, i, cmd, id)		\
	do { \
		buf[i++] = 0;			\
		buf[i++] = cmd;			\
		buf[i++] = id;			\
		buf[i++] = 0;			\
	} while (0)



#define fill_cmd_buf_val32(buf, i, _val_)	\
	do { \
		buf[i++] = (_val_ >> 24) & 0xff;	\
		buf[i++] = (_val_ >> 16) & 0xff;	\
		buf[i++] = (_val_ >> 8) & 0xff;		\
		buf[i++] = (_val_ & 0xff);		\
	} while (0)


#define fill_cmd_buf_val16(buf, i, _val_)	\
	do { \
		buf[i++] = (_val_ >> 8) & 0xff;	\
		buf[i++] = (_val_ & 0xff);	\
	} while (0)



/* form sdi write command
   addr is sdi address to write to
   val is value
   bIn is 1 if writing to the SDI input chip,
   0 if writing to the output chip */
static int send_sdi_write(struct s2226_dev *dev, unsigned char addr,
			  unsigned char val, int bIn)
{
	int i = 0;
	unsigned char *buf = dev->control_buffer;
	unsigned char id;
	int rc;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = bIn ? DEVID_SDI_IN : DEVID_SDI_OUT;
	buf[i++] = 1; /* mode (no address increment, 8 bit values) */
	buf[i++] = 0; /* values (MSB) */
	buf[i++] = 1; /* 1 8 bit data field present */
	buf[i++] = 0; /* MSB of addr(32 bits in command) */
	buf[i++] = 0;
	buf[i++] = 0;
	buf[i++] = addr;
	buf[i++] = val;
	buf[0] = (unsigned char) i;
	dprintk(4, "%s: %x:%x\n", __func__, addr, val);
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}


/* form sdi read command
   addr is sdi address to read from
   bIn is 1 if writing to the SDI input chip, 0 if writing to the output chip */
static int form_sdi_read(unsigned char *buf, unsigned char addr,
			 int bIn, unsigned char id)
{
	int i = 0;
	fill_cmd_buf(buf, i, HOSTCMD_REGREAD, id);
	buf[i++] = bIn ? DEVID_SDI_IN : DEVID_SDI_OUT;
	buf[i++] = 1; /* mode (no address increment, 8 bit values) */
	buf[i++] = 0; /* values (MSB) */
	buf[i++] = 1; /* no data present */
	buf[i++] = 0; /* MSB of addr(32 bits in command) */
	buf[i++] = 0;
	buf[i++] = 0;
	buf[i++] = addr;
	buf[0] = (unsigned char) i;
	dprintk(4, "%s: %x\n", __func__, addr);
	buf[i++] = compute_checksum(buf, buf[0]);
	return i;
}

/* form FPGA write command
   addr is FPGA address to write to
   val is value
   bIn is 1 if writing to the SDI input chip, 0 if writing to the output chip */
static int send_fpga_write(struct s2226_dev *dev, unsigned int addr,
			   unsigned int val)
{
	int i = 0;
	unsigned char *buf = dev->control_buffer;
	unsigned char id;
	int rc;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = DEVID_FPGA;
	buf[i++] = 2; /* mode (no address increment, 16 bit values) */
	buf[i++] = 0; /* values (MSB) */
	buf[i++] = 1; /* 1 16 bit data field present */
	buf[i++] = 0; /* MSB of addr(32 bits in command) */
	buf[i++] = 0;
	fill_cmd_buf_val16(buf, i, addr);
	fill_cmd_buf_val16(buf, i, val);
	buf[0] = (unsigned char) i;
	dprintk(2, "%s: %x:%x\n", __func__, addr, val);
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}

#define FPGA_WRITE_NORMAL     0
#define FPGA_WRITE_FAST       1
#define FPGA_WRITE_ADDRDATA   2
/* Note: length can not be greater than 256-14
   (14 for the header size plus checksum) if firmware under 0x44.
   If firmware > 0x44, length can't be greater than 512-14
   type == 0 (normal write)
   type == 1 (fast write)
   type == 2 (addrdata) */
static int send_fpga_write_burst(struct s2226_dev *dev, unsigned int addr,
				 unsigned char *data, int len, int type)
{
	int i = 0;
	int siz;
	int j;
	unsigned char *buf = dev->control_buffer;
	unsigned char id;
	int rc;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = DEVID_FPGA;
	buf[i] = 2; /* mode (no address increment, 16 bit values) */
	if (type == FPGA_WRITE_FAST)
		buf[i] |= (REG_UNPACK_ENABLE | REG_NORESP_ENABLE);
	else if (type == FPGA_WRITE_ADDRDATA)
		buf[i] |= (REG_ADDRDATA_MODE | REG_NORESP_ENABLE);

	i++;
	buf[i++] = 0; /* values (MSB) */
	buf[i++] = len / 2;
	buf[i++] = 0; /* MSB of addr(32 bits in command) */
	buf[i++] = 0;
	fill_cmd_buf_val16(buf, i, addr);
	/* values */
	for (j = 0; j < len; j++)
		buf[i++] = data[j];
	/* Firmware 0x44 and above supports extended commands
	   (larger than 256) use reserved field for MSB */
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



/* Note: length can not be greater than
   256-14(14 for the header size plus checksum). */
static int send_fpga_read_burst(struct s2226_dev *dev, unsigned int addr,
				unsigned char *data, int len)
{
	int i = 0;
	unsigned char *buf = dev->control_buffer;
	unsigned char id;
	int rc;
	if (len > 508)
		return -EINVAL;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGREAD, id);
	buf[i++] = DEVID_FPGA;
	buf[i++] = 2; /* mode (no address increment, 16 bit values) */
	buf[i++] = 0; /* values (MSB) */
	buf[i++] = (len / 2);
	buf[i++] = 0; /* MSB of addr(32 bits in command) */
	buf[i++] = 0;
	fill_cmd_buf_val16(buf, i, addr);
	buf[0] = (unsigned char) i;
	dprintk(2, "%s: %x\n", __func__, addr);
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	if (rc != 0)
		return rc;
	buf = (unsigned char *) dev->interrupt_buffer;
	memcpy(data, buf+4, len);
	return rc;
}


/* form fpga read command
   addr is fpga address to read from */
static int form_fpga_read(unsigned char *buf, unsigned int addr,
			  unsigned char id)
{
	int i = 0;
	fill_cmd_buf(buf, i, HOSTCMD_REGREAD, id);
	buf[i++] = DEVID_FPGA;
	buf[i++] = 2; /* mode (no address increment, 16 bit values) */
	buf[i++] = 0; /* values (MSB) */
	buf[i++] = 1; /* 1 value */
	buf[i++] = 0; /* MSB of addr(32 bits in command) */
	buf[i++] = 0;
	fill_cmd_buf_val16(buf, i, addr);
	buf[0] = (unsigned char) i;
	dprintk(4, "%s: %x\n", __func__, addr);
	buf[i++] = compute_checksum(buf, buf[0]);
	return i;
}

static int send_h51_regwr(struct s2226_dev *dev, unsigned int addr,
			  unsigned int val)
{
	int i = 0;
	unsigned char *buf = dev->control_buffer;
	unsigned char id;
	int rc;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = DEVID_H51; /* device ID */
	buf[i++] = 2; /* mode (value size) */
	buf[i++] = 0; /* values (MSB) */
	buf[i++] = 1; /* values (LSB) */
	fill_cmd_buf_val32(buf, i, addr);
	fill_cmd_buf_val16(buf, i, val);
	buf[0] = (unsigned char) i;
	dprintk(2, "H51: { 0x%08x,0x%04x},\n", addr,
		(unsigned int) val & 0xffff);
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}


static int form_h51_regrd(unsigned char *buf, unsigned int addr,
			  unsigned char id)
{
	int i = 0;
	fill_cmd_buf(buf, i, HOSTCMD_REGREAD, id);
	buf[i++] = DEVID_H51; /* device ID */
	buf[i++] = 2; /* mode (value size) */
	buf[i++] = 0; /* values (MSB) */
	buf[i++] = 1; /* values (LSB) */
	fill_cmd_buf_val32(buf, i, addr);
	buf[0] = (unsigned char) i;
	buf[i++] = compute_checksum(buf, buf[0]);
	return i;
}

static int form_generic_regrd_u8(unsigned char *buf, unsigned int addr,
				 unsigned char id, int devid)
{
	int i = 0;
	fill_cmd_buf(buf, i, HOSTCMD_REGREAD, id);
	buf[i++] = devid; /*device ID */
	buf[i++] = 1; /*mode (value size) */
	buf[i++] = 0; /* values (MSB) */
	buf[i++] = 1; /* values (LSB) */
	fill_cmd_buf_val32(buf, i, addr);
	buf[0] = (unsigned char) i;
	buf[i++] = compute_checksum(buf, buf[0]);
	return i;
}




static int get_reg(struct s2226_dev *dev, int devID,
		   unsigned int addr, unsigned int *val)
{
	unsigned char *buf;
	int rsize;
	s2226_mutex_lock(&dev->cmdlock);
	switch (devID) {
	case DEVID_AUDIO: /*readable*/
	case DEVID_SDI_SPLIT: /* may be write-only like the 7121 */
	case DEVID_VIDDEC: /* readable */
	case DEVID_VIDENC: /* Note: saa7121 is not readable */
		form_generic_regrd_u8(dev->control_buffer, addr,
				      dev->id++, devID);
		rsize = 1;
		break;
	case DEVID_H51:
		form_h51_regrd(dev->control_buffer, addr, dev->id++);
		rsize = 2;/* 2 bytes */
		break;
	case DEVID_FPGA:
		form_fpga_read(dev->control_buffer, addr, dev->id++);
		rsize = 2;/* 2 bytes */
		break;
	case DEVID_SDI_IN:
		form_sdi_read(dev->control_buffer, addr, 1, dev->id++);
		rsize = 1;/* 1 byte */
		break;
	case DEVID_SDI_OUT:
		form_sdi_read(dev->control_buffer, addr, 0, dev->id++);
		rsize = 1;/* 1 byte */
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
	int i = 0;
	unsigned char *buf = dev->control_buffer;
	int rc;
	unsigned char id;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	buf[i++] = 0;
	buf[i++] = HOSTCMD_BOOT;
	buf[i++] = id;
	buf[i++] = 0;
	buf[i++] = 0x00; /*addr MSB*/
	buf[i++] = 0x09; /*addr*/
	buf[i++] = 0x00; /*addr*/
	buf[i++] = 0x00; /*addr LSB*/
	buf[i++] = 0x00; /*size MSB*/
	buf[i++] = 0x00;/*0x0a;*/ /*size*/
	buf[i++] = 0x00;/*0xf0;*/ /*size*/
	buf[i++] = 0x00;/*0xf4;*/ /**size LSB*/
	buf[i++] = DEVID_H51;
	buf[i++] = 0;
	buf[0] = (unsigned char) i;
	dprintk(1, "%s\n", __func__);
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}
#if 0
static int send_fpga_boot(struct s2226_dev *dev)
{
	int i = 0;
	unsigned char *buf = dev->control_buffer;
	int rc;
	unsigned char id;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_BOOT, id);
	buf[i++] = 0x00; /*addr MSB*/
	buf[i++] = 0x01; /*addr*/
	buf[i++] = 0x00; /*addr*/
	buf[i++] = 0x00; /*addr LSB*/
	buf[i++] = 0x00; /*size MSB*/
	buf[i++] = 0x07; /*size*/
	buf[i++] = 0xcb; /*size*/
	buf[i++] = 0x88; /*size LSB*/
	buf[i++] = DEVID_FPGA;
	buf[i++] = 0;
	buf[0] = (unsigned char) i;
	dprintk(4, "%s\n", __func__);
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}
#endif

static int send_aic33_wr(struct s2226_dev *dev, unsigned char addr,
			 unsigned char val)
{
	int i = 0;
	unsigned char *buf = dev->control_buffer;
	int rc;
	unsigned char id;

	if (addr == 0)
		dev->audio_page = val & 1;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = DEVID_AUDIO;
	buf[i++] = 1; /* mode (no address increment, 8 bit values) */
	buf[i++] = 0; /* values (MSB) */
	buf[i++] = 1; /* 1 8 bit data field present */
	buf[i++] = 0; /* MSB of addr(32 bits in command) */
	buf[i++] = 0;
	buf[i++] = 0;
	buf[i++] = addr;
	buf[i++] = val;
	buf[0] = (unsigned char) i;
	dprintk(2, "%s: 0x%02x:0x%02x\n", __func__, addr, val);
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	if (rc != 0)
		dev_warn(&dev->udev->dev, "failed to send message %d\n", rc);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}



static int send_saa7115_wr(struct s2226_dev *dev, unsigned char addr,
			   unsigned char val)
{
	int i = 0;
	unsigned char *buf = dev->control_buffer;
	int rc;
	unsigned char id;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_REGWRITE, id);
	buf[i++] = DEVID_VIDDEC;
	buf[i++] = 1; /* mode (no address increment, 8 bit values) */
	buf[i++] = 0; /* values (MSB) */
	buf[i++] = 1; /* 1 8 bit data field present */
	buf[i++] = 0; /* MSB of addr(32 bits in command) */
	buf[i++] = 0;
	buf[i++] = 0;
	buf[i++] = addr;
	buf[i++] = val;
	buf[0] = (unsigned char) i;
	dprintk(2, "%s: %x:%x\n", __func__, addr, val);
	buf[i++] = compute_checksum(buf, buf[0]);
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

static int send_h51_setmode(struct s2226_dev *dev,
			    unsigned char streamID, unsigned char mode)
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

static int send_flash_read(struct s2226_dev *dev, unsigned int raddr,
			   int rlen, unsigned char *data, int *len)
{
	int i = 0;
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
		/* use length = 0 for 508 bytes */
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

static int send_flash_write(struct s2226_dev *dev, unsigned int addr,
			    unsigned char *data, int len)
{
	int i = 0;
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
		/* use reserved field for upper 8 bits
		   support on ARM FW 0x44 and above */
		buf[3] = (unsigned char) (i >> 8) & 0xff;
	}
	siz = i;
	buf[i++] = compute_checksum(buf, siz);
	rc = s2226_send_msg(dev, S2226_FLASHWRITE_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}

static int send_flash_erase(struct s2226_dev *dev,
			    unsigned int fw_addr, unsigned int fw_len)
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
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	s2226_mutex_unlock(&dev->cmdlock);
	return rc;
}

static int send_persona_halt(struct s2226_dev *dev, unsigned char index,
			     unsigned int fw)
{
	int i = 0;
	int rc;
	unsigned char id;
	unsigned char *buf = dev->control_buffer;
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_PERSONA_HALT, id);
	dprintk(1, "%s fw %x\n", __func__, fw);
	if (fw > 0x43) {
		buf[i++] = index;
		buf[i++] = 0;
	}
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
	for (i = 0; i < RD_URBS; i++) {
		usb_fill_bulk_urb(dev->m->read_vid[i].urb, dev->udev,
				  usb_rcvbulkpipe(dev->udev,
						  dev->ep[INDEX_EP_H264]),
				  (void *) dev->m->read_vid[i].buffer,
				  S2226_MPEG_URB_SIZE,
				  s2226_read_vid_callback,
				  &dev->m->read_vid[i]);
		dev->m->read_vid[i].ready = 0;
		dev->m->read_vid[i].context = context;
		rc = usb_submit_urb(dev->m->read_vid[i].urb, GFP_KERNEL);
		if (rc != 0) {
			dprintk(0, "%s: submit urb failed %d\n", __func__, rc);
			return rc;
		}
	}
	return 0;
}

static int s2226_start_preview_urbs(struct s2226_dev *dev, int context)
{
	int i;
	int rc;
	dev->p->m_sync = 1;
	dprintk(2, "start preview urbs\n");
	for (i = 0; i < RD_URBS; i++) {
		usb_fill_bulk_urb(dev->p->read_vid[i].urb, dev->udev,
				  usb_rcvbulkpipe(dev->udev, dev->ep[INDEX_EP_RAW]),
				  (void *) dev->p->read_vid[i].buffer,
				  S2226_PREVIEW_URB_SIZE,
				  s2226_read_preview_callback,
				  &dev->p->read_vid[i]);
		dev->p->read_vid[i].ready = 0;
		dev->p->read_vid[i].context = context;
		rc = usb_submit_urb(dev->p->read_vid[i].urb, GFP_KERNEL);
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
	if (!dev->udev)
		return -ENODEV;
	dprintk(1, "cur_input=%d\n", dev->cur_input);
	dprintk(1, "vFormat=%d\n", dev->h51_mode.vFormat);
	dprintk(1, "frRed=%d\n", dev->h51_mode.frRed);
	dprintk(1, "vBitrate=%d\n", dev->h51_mode.vBitrate);
	dprintk(1, "aBitrate=%d\n", dev->h51_mode.aBitrate);
	dprintk(1, "aMode=%d\n", dev->h51_mode.aMode);
	/* 1) Change to alternate configuration */
	/*lliface_desc = dev->interface->cur_altsetting;*/
	s2226_set_interface(dev, 0, 1, 0);
	if (dev->h51_reload_required) {
		send_h51_boot(dev);
		dev->h51_reload_required = 0;
		dprintk(2, "s2226: chipset reloaded\n");
	}
	setH51regs(&dev->h51_mode);
	for (bank = 0; bank < 4; bank++) {
		int numregs = 0;
		switch (bank) {
		case 0:
			numregs = h51APRMregs;
			break;
		case 1:
			numregs = h51IPRMregs;
			break;
		case 2:
			numregs = h51IPRVregs;
			break;
		case 3:
			numregs = h51IPRAregs;
			break;
		}
		for (i = 0; i < numregs; i++) {
			unsigned int address = 0, value = 0;
			switch (bank) {
			case 0:
				address = h51APRM[i][0];
				value = h51APRM[i][1];
				break;
			case 1:
				address = h51IPRM[i][0];
				value = h51IPRM[i][1];
				break;
			case 2:
				address = h51IPRV[i][0];
				value = h51IPRV[i][1];
				break;
			case 3:
				address = h51IPRA[i][0];
				value = h51IPRA[i][1];
				break;
			}
			dprintk(3, "h51 register setting: %08x: %04x\n",
				address, value);
			if (address == H51REG_GOP_CLK) {
				if (dev->closed_gop) {
					dev_info(&dev->udev->dev,
						 "s2226: closed gop\n");
					value |= 0x0400;
				} else {
					dev_info(&dev->udev->dev,
						 "s2226: open gop\n");
					value &= ~0x0400;
				}
			}
			if (address == H51REG_GOP_STRM) {
				if (dev->gop_struct != -1) {
					dev_info(&dev->udev->dev,
						 "GOP struct %d\n",
						 dev->gop_struct);
					value &= 0x00ff;
					value |=
						((dev->gop_struct & 0xff) << 8);
					if (dev->gop_struct == 2 ||
					    (dev->gop_struct > 3))
						dprintk(1, "rsrvd val\n");
				}
			}
			if (address == H51REG_AIN_OFFSET) {
				if (dev->ainoffset != -1) {
					dev_info(&dev->udev->dev,
						 "AIN_OFFSET override 0x%x\n",
						 dev->ainoffset);

					if (dev->ainoffset > AINOFFSET_MAX) {
						dev->ainoffset = AINOFFSET_MAX;
						dev_info(&dev->udev->dev,
							 "clamping ainoffset");
					}
					value = dev->ainoffset;
				}
			}
			rc = send_h51_regwr(dev, address, value);
			if (rc < 0) {
				dprintk(1,  "%s regwr %d\n", __func__, rc);
				return rc;
			}
		}
	}
	/* 3) Send Halt to config persona: */
	rc = send_persona_halt(dev, 0, dev->arm_ver);
	if (rc < 0) {
		dprintk(1,  "%s persona_halt %d\n", __func__, rc);
		return rc;
	}
	dev->persona = PERSONA_DEFAULT;

	/* 4) Set encode persona: */
	dprintk(1, "done persona encode\n");
	rc = send_persona_run(dev, 1);
	if (rc < 0) {
		dprintk(1,  "%s persona_run %d\n", __func__, rc);
		return rc;
	}

	dev->persona = PERSONA_ENCODE;
	s2226_flush_in_ep(dev);
	/* start the read pipe */
	s2226_start_rb_urbs(dev, context);

	/* 5)  Start encode */
	rc = send_h51_setmode(dev, 0, STREAM_MODE_ENCODE);
	if (rc < 0) {
		dprintk(1,  "%s persona_halt %d\n", __func__, rc);
		return rc;
	}
	dev->h51_state = STREAM_MODE_ENCODE;
	dprintk(2, "start encode\n");
	return 0;
}

int s2226_stop_encode(struct s2226_dev *dev, int idx)
{
	int rc;
	int streaid = 0;

	rc = send_h51_setmode(dev, streaid, STREAM_MODE_IDLE);
	if (rc < 0)
		return rc;
	rc = send_persona_halt(dev, 0, dev->arm_ver);
	if (rc < 0)
		return rc;
	dev->persona = PERSONA_DEFAULT;
	dev->h51_state = STREAM_MODE_IDLE;
	return 0;
}

static int s2226_start_preview_urbs(struct s2226_dev *dev, int context);

static int s2226_start_preview(struct s2226_dev *dev, int idx, int context)
{
	int rc;
	if (!dev->udev)
		return -ENODEV;
	/* send halt */
	rc = send_persona_halt(dev, 1, dev->arm_ver);
	if (rc != 0) {
		/* retry */
		rc = send_persona_halt(dev, 1, dev->arm_ver);
	}
	if (rc < 0) {
		dprintk(0, "%s persona_halt %d\n", __func__, rc);
		return rc;
	}
	/* dev->persona = PERSONA_DEFAULT; */
	/* start the preview read pipe */
	s2226_start_preview_urbs(dev, context);
	dprintk(2, "raw preview started!\n");
	rc = send_persona_run(dev, 3); /*PERSONA_RAWVID);*/
	dprintk(2, "send persona run %d\n", rc);
	if (rc < 0) {
		dprintk(1, "%s persona_run %d\n", __func__, rc);
		return rc;
	}
	return 0;
}

int s2226_stop_preview(struct s2226_dev *dev, int idx)
{
	int rc;
	rc = send_persona_halt(dev, 3, dev->arm_ver);
	rc = send_persona_halt(dev, 1, dev->arm_ver);
	if (rc < 0) {
		dprintk(3, "stop preview failed %d, retrying\n", rc);
		rc = send_persona_halt(dev, 1, dev->arm_ver);
		if (rc < 0) {
			dprintk(3, "stop preview failed %d, retrying\n", rc);
			rc = send_persona_halt(dev, 1, dev->arm_ver);
		}
		return rc;
	}
	if (rc != 0)
		dprintk(3, "halt failed\n");
	return rc;
}


int s2226_start_decode(struct s2226_dev *dev, int idx)
{
	int i;
	int rc;
	int numregs;

	if (!dev->udev)
		return -ENODEV;

	dprintk(1, "%s vFormat=%d\n", __func__, dev->h51_mode.vFormat);
	dprintk(1, "vBitrate=%d\n", dev->h51_mode.vBitrate);
	dprintk(1, "aBitrate=%d\n", dev->h51_mode.aBitrate);
	dprintk(1, "aMode=%d\n", dev->h51_mode.aMode);
	/* 1) Change to alternate configuration */
	s2226_set_interface(dev, 0, 0, 0);
	if (dev->h51_reload_required) {
		send_h51_boot(dev);
		dev->h51_reload_required = 0;
	}
	numregs = sizeof(G_h51reg_dec) / (2*sizeof(u32));
	/* 2) send h51 registers */
	/* update decode registers based on format */
	for (i = 0; i < numregs; i++) {
		int address, value;
		address = G_h51reg_dec[i][0];
		value = G_h51reg_dec[i][1];
		switch (address) {
		case H51REG_V_V420_FILTER_00:
			/* default is interlaced */
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

		if ((address == H51REG_V_IP_DETECT) &&
		    IS_PROG_INPUT(dev->cur_input)) {
			value = 0x0001;
			if (*s2226_filter_mode == 1) {
				dprintk(1,
				       "s2226: turning off V_IP_DETECT\n");
				value = 0x0000;
			}
		}

		if (address == H51REG_DPB_SIZE && dev->dpb_size > -1) {
			value = dev->dpb_size;
			dprintk(1, "using dpb size %d\n", value);
		}

		/* frRed is set after the set_input
		   74.1758 not defined in the ARM, so we piggy back
		   on the non frRed inputs and set frRed value.
		   only need to change V_VINPELCLK H51 register */
		if (address == H51REG_GOP_CLK) {
			if (dev->h51_mode.frRed == 0)
				value |= 0x0001; /* 74.25 (no reduction) */
			else
				value &= ~0x0001;
		}

		if ((address == H51REG_AV_RESYNC) && (dev->avresync != -1)) {
			if (dev->avresync && (dev->avresync < AVRESYNC_MIN))
				dev->avresync = AVRESYNC_MIN;
			if (dev->avresync > AVRESYNC_MAX)
				dev->avresync = AVRESYNC_MAX;
			value |= (dev->avresync << 2);
			dprintk(1,
				"using AV_RESYNC_THRESHOLD 0x%x\n",
				dev->avresync);
		}
		dprintk(3, "h51 register setting: %08x: %04x\n",
			address, value);
		rc = send_h51_regwr(dev, address, value);
		if (rc < 0) {
			dprintk(1, "%s regwr %d\n", __func__, rc);
			return rc;
		}
	}
	/* 3) Send Halt to config persona: */
	rc = send_persona_halt(dev, 0, dev->arm_ver);
	if (rc < 0) {
		dprintk(0, "%s persona_halt %d\n", __func__, rc);
		return rc;
	}
	dev->persona = PERSONA_DEFAULT;
	/* 4) Set decode persona: */
	dprintk(1, "done persona\n");
	rc = send_persona_run(dev, PERSONA_DECODE);
	if (rc < 0) {
		dprintk(1, "%s persona_run %d\n", __func__, rc);
		return rc;
	}
	dev->persona = PERSONA_DECODE;
	/* 5)  Start decode */
	rc = send_h51_setmode(dev, 0, STREAM_MODE_DECODE);
	if (rc < 0) {
		dprintk(1, "%s persona_halt %d\n", __func__, rc);
		return rc;
	}
	dev->h51_state = STREAM_MODE_DECODE;
	return 0;
}

int s2226_stop_decode(struct s2226_dev *dev, int idx)
{
	int rc;

	rc = send_h51_setmode(dev, 0, STREAM_MODE_IDLE);
	if (rc < 0)
		return rc;
	rc = send_persona_halt(dev, 0, dev->arm_ver);
	if (rc < 0)
		return rc;
	dev->persona = PERSONA_DEFAULT;
	dev->h51_state = STREAM_MODE_IDLE;
	return 0;
}

static int s2226_set_attr(struct s2226_dev *dev, int attr, int value);

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
	dprintk(0, "setting H51 audio INTF to %s, rc: %d\n",
		bMaster ? "MASTER" : "SLAVE", ret);
	return ret;
}

static int s2226_set_attr(struct s2226_dev *dev, int attr, int value)
{
	unsigned char *buf = (unsigned char *) dev->control_buffer;
	int rc;
	unsigned char id;
	int i = 0;
	int timeout = S2226_BULKMSG_TO;

	dprintk(2, "set attr %d %x\n", attr, value);
	s2226_mutex_lock(&dev->cmdlock);
	id = dev->id++;
	fill_cmd_buf(buf, i, HOSTCMD_ATTR_WRITE, id);
	fill_cmd_buf_val32(buf, i, attr);
	fill_cmd_buf_val32(buf, i, value);
	buf[0] = i;
	buf[i++] = compute_checksum(buf, buf[0]);
	if (attr == ATTR_INPUT) {
		dprintk(1, "setting the input\n");
		timeout = S2226_SETINPUT_TO;
		dev->h51_reload_required  = 0;
	}

	rc = s2226_send_msg(dev, timeout);

	if (rc < 0) {
		dev_info(&dev->udev->dev, "%s fail %d\n", __func__, rc);
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

static int s2226_get_attr_ext(struct s2226_dev *dev, int attr, int *value,
			      int *v2);
static int s2226_get_attr(struct s2226_dev *dev, int attr, int *value)
{
	return s2226_get_attr_ext(dev, attr, value, NULL);
}

static int s2226_get_attr_ext(struct s2226_dev *dev, int attr, int *value,
			      int *v2)
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
	buf[i++] = compute_checksum(buf, buf[0]);
	rc = s2226_send_msg(dev, S2226_BULKMSG_TO);
	if (rc < 0) {
		dev_info(&dev->udev->dev, "%s fail %d\n", __func__, rc);
		s2226_mutex_unlock(&dev->cmdlock);
		return rc;
	}
	buf = (unsigned char *) dev->interrupt_buffer;
	if (buf[3] != 0) {
		dev_info(&dev->udev->dev,
			 "%s: invalid returned value %d\n",
		       __func__, buf[3]);
		s2226_mutex_unlock(&dev->cmdlock);
		return buf[3];
	}
	if ((attr == ATTR_LAST_ERR) || (attr == ATTR_LAST_MESSAGE)) {
		dprintk(0, "%s dump: %x %x %x %x\n %x %x %x %x\n %x %x %x %x\n",
			(attr == ATTR_LAST_ERR) ? "last err" : "last msg",
			buf[3], buf[4],
			buf[5], buf[6],
			buf[7], buf[8],
			buf[9], buf[10],
			buf[11], buf[12],
			buf[13], buf[14]);
	}
	*value = (buf[4] << 24) + (buf[5] << 16) + (buf[6] << 8) + buf[7];
	if (v2)
		*v2 = (buf[8] << 24) + (buf[9] << 16) +
			(buf[10] << 8) + buf[11];
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
		/* read interrupt endpoint with short timeout
		   in case FX2 is just priming itself
		   but long enough in case of POR when there is
		   a delay before the firmware in the ARM loads
		   3s should be plenty */
		(void) usb_bulk_msg(dev->udev,
				    usb_rcvbulkpipe(dev->udev,
						    dev->ep[INDEX_EP_RESP]),
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

	/* write some nops to prime the control out buffer */
	for (i = 0; i < 2; i++)
		(void) s2226_send_nop(dev, 0); /* NOP without response */
	iface_desc = dev->interface->cur_altsetting;
	/* flush all the in endpoints */
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;
		epnum = endpoint->bEndpointAddress & 7;
		if (((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
		     == USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
		     == USB_ENDPOINT_XFER_BULK)) {
			/* flush any data in FX2, just in case */
			int j = 0;
			do {
				rc = usb_bulk_msg(dev->udev,
						  usb_rcvbulkpipe(dev->udev,
								  epnum),
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


static int s2226_set_interface(struct s2226_dev *dev, int cfg, int alt, int force)
{
	int rc;
	/* ARM should be halted before changing the interface.
	   Also, it should not be streaming or decoding */
	dprintk(1, "%s\n", __func__);
	if ((dev->cfg_intf == cfg) && (dev->alt_intf == alt) && !force)
		return 0;
	if (dev->h51_state == STREAM_MODE_ENCODE)
		s2226_stop_encode(dev, 0);
	if (dev->h51_state == STREAM_MODE_DECODE)
		s2226_stop_decode(dev, 0);
	/* always send halt */
	(void) send_persona_halt(dev, 0, dev->arm_ver);

	/* if startup or reset, send halt again */
	if (force)
		(void) send_persona_halt(dev, 0, dev->arm_ver);
	/* set the interface */
	rc = usb_set_interface(dev->udev, cfg, alt);
	s2226_prime_fx2(dev);
	dev->cfg_intf = cfg;
	dev->alt_intf = alt;
	return rc;
}

static int s2226_new_input(struct s2226_dev *dev, int input)
{
	int vFormat = 0;
	int frRed = 0;
	int is_decode = 0;
	int rc;
	switch (input) {
	case INPUT_COMP0_480I:
	case INPUT_COMP1_480I:
	case INPUT_SVIDEO0_480I:
	case INPUT_SVIDEO1_480I:
	case INPUT_H51_SD_480I:
		vFormat = VFMT_480_60i;
		frRed = 1; /* for SD, this field is a don't care */
		break;
	case INPUT_COMP0_576I:
	case INPUT_COMP1_576I:
	case INPUT_SVIDEO0_576I:
	case INPUT_SVIDEO1_576I:
	case INPUT_H51_SD_576I:
		vFormat = VFMT_576_50i;
		frRed = 0; /* for SD, this field is a don't care */
		break;
	case INPUT_SDI_480I:
	case INPUT_SDI_480I_CB:
		vFormat = VFMT_480_60i;
		frRed = 1; /* for SD, this field is a don't care */
		break;
	case INPUT_SDI_576I:
	case INPUT_SDI_576I_CB:
		vFormat = VFMT_576_50i;
		frRed = 0; /* for SD, this field is a don't care */
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
			dev_info(&dev->udev->dev,
				 "upgrade FPGA curfw: %x\n",
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
			/* 59.94 uses 60hz colorbar reference */
			/* for decode */
			if (input == INPUT_SDI_720P_5994_CB) {
				dev_info(&dev->udev->dev,
					 "upgrade FPGA curfw: %x\n",
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
		dev_warn(&dev->udev->dev,
			 "s2226: unknown input %d!\n", input);
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
	rc = 0;
	dprintk(3, "%s cur: %d new: %d v4l: %d\n", __func__, dev->cur_input, input, dev->v4l_input);
	if (dev->cur_input != input) {
		rc = s2226_set_attr(dev, ATTR_INPUT, input);
		if (rc != 0) /* retry*/
			rc = s2226_set_attr(dev, ATTR_INPUT, input);
	}

	if (rc != 0) {
		dprintk(3, "%s: input %d rc %d\n", __func__, input, rc);
		dev->cur_input = -1;
		return rc;
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
		(void) s2226_set_audiomux_mpegin(dev, dev->cur_audio_mpeg);
		(void) s2226_set_audiomux_lineout(dev, dev->cur_audio_lineout);
		(void) s2226_set_audiomux_sdiout(dev, dev->cur_audio_sdiout);
	}
	dprintk(2, "%s: input %d rc %d\n", __func__, input, rc);
	return rc;
}

#define S2226_REG_AUD_INPK_CTL  (0x015<<1)

static int s2226_set_audiomux_mpegin(struct s2226_dev *dev, int aud)
{
	int reg;
	int rc;
	rc = get_reg(dev, DEVID_FPGA, S2226_REG_AUD_INPK_CTL, &reg);
	if (rc != 0)
		dev_warn(&dev->udev->dev, "s2226 setaudio mux fail\n");

	switch (aud) {
	case S2226_AUDIOINPUT_LINE:
		reg = reg & 0x010c;/* Keep bit 8(GENCK),3, 2 */
		reg = reg | 0x0000;/* line in to MPEG in */
		break;
	case S2226_AUDIOINPUT_TONE:
		reg = reg & 0x010C;
		reg = reg | 0x00C2; /* Tone test data to MPEG in */
		break;
	case S2226_AUDIOINPUT_SDI:
		reg = reg & 0x010C;
		reg = reg | 0x0042; /* SDIiAud data to MPEG in */
		break;
	default:
		dprintk(0, "s2226 setaudio mux, invalid input %d\n", aud);
		return -EINVAL;
	}
	rc = send_fpga_write(dev, S2226_REG_AUD_INPK_CTL, reg);
	return rc;
}

static int s2226_set_audiomux_lineout(struct s2226_dev *dev, int aud)
{
	int reg;
	int rc;
	rc = get_reg(dev, DEVID_FPGA, S2226_REG_AUD_INPK_CTL, &reg);
	if (rc != 0)
		dev_warn(&dev->udev->dev, "s2226 setaudio mux fail\n");

	switch (aud) {
	case AMUX_LINE_OUT_MPEG_OUT:
		reg &= ~0x000C; /* Bit 2,3 select what is going to Line-Out */
		reg |=  0x0000; /* Line-Out gets MPEG-Out */
		break;
	case AMUX_LINE_OUT_SDI_IN:
		reg &= ~0x000C;
		reg |=  0x0004;
		break;
	case AMUX_LINE_OUT_LINE_IN:
		reg &= ~0x000C; // Bit 2,3 selects what is going to Line-Out
		reg |=  0x0008; // Line-Out gets Line-In embedded audio
		break;
	default:
		dprintk(0, "s2226 setaudio mux, invalid input %d\n", aud);
		return -EINVAL;
	}
	dev->cur_audio_lineout = aud;
	rc = send_fpga_write(dev, S2226_REG_AUD_INPK_CTL, reg);
	return rc;
}

#define S2226_REG_AUD_OUTPK_CTL (0x01A<<1)
static int s2226_set_audiomux_sdiout(struct s2226_dev *dev, int aud)
{
	int reg;
	int rc;
	switch (aud) {
	case AMUX_SDI_OUT_LINE_IN:
		reg = 0x83;
		break;
	case AMUX_SDI_OUT_MPEG_OUT:
		reg = 0xC3;
		break;
	case AMUX_SDI_OUT_TONE:
		reg = 0x82;
		break;
	case AMUX_SDI_OUT_SDI_IN:
		reg = 0x00;
		break;
	default:
		dprintk(0, "s2226 setaudio mux, invalid input %d\n", aud);
		return -EINVAL;
	}
	rc = send_fpga_write(dev, S2226_REG_AUD_OUTPK_CTL, reg);
	dev->cur_audio_lineout = aud;
	return rc;
}

// Audio Level meter
#define S2226_REG_AUDLVLMTRCTRL (0x025<<1)    // r/w  Audio Level Meter Control
#define S2226_REG_MTR_AUDL0     (0x026<<1)    // Peak and Decayed Left   Audio Amplitude LSB
#define S2226_REG_MTR_AUDL1     (0x027<<1)    // MSB ( 6 :  0) = AMTR_AUDL(22 : 16)
#define S2226_REG_MTR_AUDR0     (0x028<<1)    // Peak and Decayed Right  Audio Amplitude LSB
#define S2226_REG_MTR_AUDR1     (0x029<<1)    // MSB ( 6 :  0) = AMTR_AUDR(22 : 16)
#define S2226_REG_MTR_DB_L      (0x02A<<1)    // Left  Decibel level, 11 bit, unsigned binary 0=max volume (11-bits) 2048 steps, -0.1 db each.
#define S2226_REG_MTR_DB_R      (0x02B<<1)    // Right Decibel level, 11 bit, unsigned binary 0=max volume (11-bits) 2048 steps, -0.1 db each.
#define S2226_REG_MTR_HLDL      (0x02C<<1)    // Left Clip detect and hold value. 
                                              //   MSB=1 indicates (0x7FFFFF or 0x800000 detected). 
                                              //   Bits(10:0) Left Held Decibel level, 11 bit, unsigned binary 0=max volume (11-bits) 2048 steps, -0.1 db each.
#define S2226_REG_MTR_HLDR      (0x02D<<1)    // Right Clip detect and hold value. 
                                              //   MSB=1 indicates (0x7FFFFF or 0x800000 detected). 
                                              //   Bits(10:0) Right Held Decibel level, 11 bit, unsigned binary 0=max volume (11-bits) 2048 steps, -0.1 db each.

/*
 * Get Audio Meter Peak and Decayed Level 23-bits, unsigned binary 0=min volume
 */
static int s2226_get_audiomtr_level(struct s2226_dev *dev, int *audl, int *audr)
{
	int reg;
	int rc;

	reg = 0;
	/* for efficiency, we may want to later prevent polling 
	 * faster than refresh rate in FPGA */
	if (audl != NULL) {
		rc = get_reg(dev, DEVID_FPGA, S2226_REG_MTR_AUDL0, &reg);
		if (rc != 0)
			return rc;
		*audl = reg;
		rc = get_reg(dev, DEVID_FPGA, S2226_REG_MTR_AUDL1, &reg);
		if (rc != 0)
			return rc;
		*audl = *audl | ( (unsigned int)((unsigned)reg) << 16);
	}

	if (audr != NULL) {
		rc = get_reg(dev, DEVID_FPGA, S2226_REG_MTR_AUDR0, &reg);
		if (rc != 0)
			return rc;
		*audr = reg;
		rc = get_reg(dev, DEVID_FPGA, S2226_REG_MTR_AUDR1, &reg);
		if (rc != 0)
			return rc;
		*audr = *audr | ( (unsigned int)((unsigned)reg) << 16);
	}
	return 0;     
}


/** Get Audio Meter Peak and Decayed Level in dB
 * 11-bits, unsigned binary 0=max volume 
 * 2048 steps, -0.1 db each
 */
static int s2226_get_audiomtr_leveldb(struct s2226_dev *dev, int *db_l, int *db_r)
{
	int reg;
	int rc;

	reg = 0;
	/* TODO: prevent polling the interval faster than refresh rate in FPGA */

	if (db_l != NULL) {
		rc = get_reg(dev, DEVID_FPGA, S2226_REG_MTR_DB_L, &reg);
		if (rc != 0)
			return rc;
		*db_l = reg;
	}

	if (db_r != NULL) {
		rc = get_reg(dev, DEVID_FPGA, S2226_REG_MTR_DB_R, &reg);
		if (rc != 0)
			return rc;
		*db_r = reg;
	}
	return 0;     
}

/* get the current audio meter channel */
static int s2226_get_audiomtr_channel(struct s2226_dev *dev, int *chan)
{
	int reg;
	int rc;

	reg = 0;
	rc = get_reg(dev, DEVID_FPGA, S2226_REG_AUDLVLMTRCTRL, &reg);
	if (rc != 0)
		return rc;
	reg = reg & 0x03;
	*chan = reg;
	return 0;     
}

static int s2226_set_audiomtr_channel(struct s2226_dev *dev, int chan)
{
	int reg;
	int rc;

	reg = 0;
	rc = get_reg(dev, DEVID_FPGA, S2226_REG_AUDLVLMTRCTRL, &reg);
	if (rc != 0)
		return rc;
	reg = reg & ~0x03;
	reg = reg | (chan & 0x03);
	rc = send_fpga_write(dev, S2226_REG_AUDLVLMTRCTRL, reg);
	return rc;
}

static int s2226_get_audiomtr_test(struct s2226_dev *dev, int *val)
{
	int reg;
	int rc;

	reg = 0;
	rc = get_reg(dev, DEVID_FPGA, S2226_REG_AUDLVLMTRCTRL, &reg);
	if (rc != 0)
		return rc;
	
	*val = (unsigned) reg;
	*val = *val >> 2;
	*val = (*val) & 0x3;
	return 0;     
}

static int s2226_set_audiomtr_test(struct s2226_dev *dev, int val)
{
	int reg;
	int rc;

	reg = 0;
	rc = get_reg(dev, DEVID_FPGA, S2226_REG_AUDLVLMTRCTRL, &reg);
	if (rc != 0)
		return rc;
	val = val & 0x3;
	reg = (reg & ~0x0C) | (val << 2);
	rc = send_fpga_write(dev, S2226_REG_AUDLVLMTRCTRL, reg);
	return rc;
}

static int s2226_set_audiomtr_holdrelease(struct s2226_dev *dev, int val)
{
	int reg;
	int rc;

	reg = 0;
	rc = get_reg(dev, DEVID_FPGA, S2226_REG_AUDLVLMTRCTRL, &reg);
	//printk("set amtr hold release %d\n", val);
	if (rc != 0)
		return rc;
	val = (~val) & 0x1;
#define S2226_ALMC_HOLD_RLS 0x0010
	reg = (reg & ~S2226_ALMC_HOLD_RLS) | 
		(val << 4);
	rc = send_fpga_write(dev, S2226_REG_AUDLVLMTRCTRL, reg);
	return rc;
}



static int s2226_get_audiomtr_holdrelease(struct s2226_dev *dev, int *val)
{
	int reg;
	int rc;

	reg = 0;
	rc = get_reg(dev, DEVID_FPGA, S2226_REG_AUDLVLMTRCTRL, &reg);
	if (rc != 0)
		return rc;
	
	*val = (unsigned) reg;
	*val = *val >> 4;
	*val = (~*val) & 0x1;
	return 0;     
}


static int s2226_set_audiomtr_holdtime(struct s2226_dev *dev, int val)
{
	int reg;
	int rc;

	reg = 0;
	rc = get_reg(dev, DEVID_FPGA, S2226_REG_AUDLVLMTRCTRL, &reg);
	if (rc != 0)
		return rc;
	// HOLD_TIM is inverted
	//printk("set amtr hold time %d\n", val);
	val    = (~val) & 0x7;
#define S2226_ALMC_HOLD_TIM 0x00E0
	reg = (reg & ~S2226_ALMC_HOLD_TIM) | 
		(val << 5);
	rc = send_fpga_write(dev, S2226_REG_AUDLVLMTRCTRL, reg);

	return rc;
}


static int s2226_get_audiomtr_holdtime(struct s2226_dev *dev, int *val)
{
	int reg;
	int rc;

	reg = 0;
	rc = get_reg(dev, DEVID_FPGA, S2226_REG_AUDLVLMTRCTRL, &reg);
	if (rc != 0)
		return rc;
	*val = (unsigned)reg;
	*val = *val >> 5;
	// HOLD_TIM is inverted
	*val = (~*val) & 0x7;
	return 0;     
}


static int s2226_get_audiomtr_holdclip(struct s2226_dev *dev, int *hld_l, int *hld_r, int *clip_l, int *clip_r)
{
	int reg;
	int rc;

	reg = 0;
	if (hld_l || clip_l) {
		rc = get_reg(dev, DEVID_FPGA, S2226_REG_MTR_HLDL, &reg);
		if (rc != 0)
			return rc;
		if (hld_l)
			*hld_l = reg & 0x07ff;
		if (clip_l)
			*clip_l = (reg & 0x8000) ? 1 : 0;
	}

	if (!(hld_r || clip_r))
		return 0;

	rc = get_reg(dev, DEVID_FPGA, S2226_REG_MTR_HLDR, &reg);
	if (rc != 0)
		return rc;
		
	if (hld_r)
		*hld_r = reg & 0x07ff;
	if (clip_r)
		*clip_r = (reg & 0x8000) ? 1 : 0;

	return 0;
}






static int s2226_get_fpga_ver(struct s2226_dev *dev);

long s2226_ioctl(struct file *file,
		 unsigned int cmd, unsigned long arg)
{
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;
	struct s2226_fh *fh = NULL;
	void __user *argp = (void __user *)arg;
	int ret = 0;
	dprintk(4, "strm %p ioctl %x %lx\n", strm, cmd, arg);
	if (dev == NULL) {
		printk(KERN_INFO "invalid device\n");
		return -EINVAL;
	}
	switch (cmd) {
	case S2226_VIDIOC_DECODE_STATE:
	{
		int j;
		for (j = 0; j < WR_URBS; j++)
			if (!dev->m->write_vid[j].ready)
				return 1; /* still decoding */
		/* driver idle,
		 * but H51 decoder could still be running */
		return 0;
	}
	case S2226_VIDIOC_I2C_TX:
	{
		i2c_param_t p;
		unsigned char addr;
		if (copy_from_user(&p, (void __user *) arg, sizeof(i2c_param_t)))
			return -EINVAL;
		addr = (p.addr);
		dprintk(1, "I2C_TX addr:[%x], len:%d, data: %x\n", addr, p.len, p.data[2]);
		ret = s2226_vendor_request(dev,
					   0x23,
					   addr, addr,	/* val, idx */
					   p.data, /*pbuf*/
					   p.len,/* len */
					   1);	/* bIn */
		dprintk(1, "I2C_TX %d\n", ret);
	}
	break;
	case S2226_VIDIOC_STARTDECODE:
	{
		start_param_t cmd;

		dprintk(1, "start decode\n");
		if (strm->type != S2226_STREAM_DECODE) {
			printk(KERN_INFO "S2226_VIDIOC_STARTDECODE invalid handle\n");
			return -EINVAL;
		}
		fh = file->private_data;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		if (vb2_is_streaming(&dev->m->vb_vidq)) {
			printk(KERN_INFO "s2226: stream busy m\n");
			return -EBUSY;
		}
		if (vb2_is_streaming(&dev->p->vb_vidq)) {
			printk(KERN_INFO "s2226: stream busy p\n");
			return -EBUSY;
		}
		dprintk(3, "start decode channel %d\n", cmd.idx);
		if (!res_get(dev, fh, RES_DECODE)) {
			printk(KERN_INFO "s2226: stream busy\n");
			return -EBUSY;
		}
		dprintk(2, "start decode %d\n", cmd.idx);
		ret = s2226_start_decode(dev, cmd.idx);
		dprintk(2, "start decode done %d\n", ret);
		if (ret != 0)
			res_free(dev, fh, RES_DECODE);
		return ret;
	}
	case S2226_VIDIOC_STOPDECODE:
	{
		stop_param_t cmd;
		int i;
		fh = file->private_data;
		dprintk(2, "stop decode\n");
		if ((res_locked(dev, fh) & RES_DECODE) && !(res_check(fh) & RES_DECODE)) {
			/* other handle owns the streaming resource */
			dprintk(1, "stop decode not ours\n");
			return 0;
		}
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		/* kill the urbs */
		dprintk(1, "s2226, stop decode, kill urbs\n");
		for (i = 0; i < WR_URBS; i++) {
			usb_kill_urb(dev->d->write_vid[i].urb);
			dev->d->write_vid[i].ready = 1;
		}
		ret = s2226_stop_decode(dev, cmd.idx);
		dprintk(2, "stop decode %d\n", ret);
		res_free(dev, fh, RES_DECODE);
		dev->h51_reload_required  = 1;
		return ret;
	}
	case S2226_VIDIOC_SDII_WR:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "sdii write %x:%x\n", cmd.addr, cmd.val);
		ret = send_sdi_write(dev, cmd.addr, cmd.val, 1);
		break;
	}
	case S2226_VIDIOC_SDII_RD:
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
	case S2226_VIDIOC_SDIO_WR:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "sdio write %x:%x\n", cmd.addr, cmd.val);
		ret = send_sdi_write(dev, cmd.addr, cmd.val, 0);
		break;
	}
	case S2226_VIDIOC_SDIO_RD:
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
	case S2226_VIDIOC_FPGA_WR:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "Fpga write %x:%x\n", cmd.addr, cmd.val);
		ret = send_fpga_write(dev, cmd.addr, cmd.val);
		if (ret < 0)
			return ret;
		break;
	}
	case S2226_VIDIOC_FPGA_WR_BURST:
	{
		struct io_burst cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "fpga write burst %x\n", cmd.addr);
		ret = send_fpga_write_burst(dev, cmd.addr, cmd.data,
					    cmd.len, FPGA_WRITE_NORMAL);
		if (ret < 0)
			return ret;
		break;
	}
	case S2226_VIDIOC_FPGA_WR_BURST_FAST:
	{
		struct io_burst cmd;
		if (dev->arm_ver < 0x50) {
			dev_info(&dev->udev->dev,
				 "s2226: need newer fw [0x%x]\n", dev->arm_ver);
			return -EINVAL;
		}
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "S2226_VIDIOC_FPGA_WR_BURST_FAST %x\n", cmd.addr);
		ret = send_fpga_write_burst(dev, cmd.addr, cmd.data,
					    cmd.len, FPGA_WRITE_FAST);
		if (ret < 0)
			return ret;
		break;
	}
	case S2226_VIDIOC_FPGA_WR_ADDRDATA:
	{
		struct io_burst cmd;
		if (dev->arm_ver < 0x58) {
			dev_info(&dev->udev->dev,
				 "s2226: need newer firmware[0x%x]\n", dev->arm_ver);
			return -EINVAL;
		}
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "S2226_VIDIOC_FPGA_WR_ADDRDATA %x\n", cmd.addr);
		ret = send_fpga_write_burst(dev, cmd.addr, cmd.data, cmd.len,
					    FPGA_WRITE_ADDRDATA);
		if (ret < 0)
			return ret;
		break;
	}
	case S2226_VIDIOC_FPGA_RD:
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
	case S2226_VIDIOC_FPGA_RD_BURST:
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
	case S2226_VIDIOC_H51_WR:
	{
		struct io_reg cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(2, "h51 write %x:%x\n", cmd.addr, cmd.val);
		ret = send_h51_regwr(dev, cmd.addr, cmd.val);
		if (ret < 0)
			return ret;
		break;
	}
	case S2226_VIDIOC_H51_RD:
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
#if 0
	case S2226_VIDIOC_SET_ATTR:
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
#endif
	case S2226_VIDIOC_GET_ATTR:
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
	case S2226_VIDIOC_FLASH_WR:
	{
		struct flash_param p;
		if (dev->users > 1) {
			dev_info(&dev->udev->dev,
				 "s2226: other process using device\n");
			return -EBUSY;
		}
		if (copy_from_user(&p, argp, sizeof(p)))
			return -EINVAL;
		/* protect our memory space */
		if (!mfgmode && (p.addr < 0x10000))
			return -EINVAL;
		dprintk(2, "flash write addr:%x, len:%x\n", p.addr, p.len);
		ret = send_flash_write(dev, p.addr, p.data, p.len);
		if (ret < 0)
			return ret;
		break;
	}
	case S2226_VIDIOC_FLASH_RD:
	{
		struct flash_param p;
		if (copy_from_user(&p, argp, sizeof(p)))
			return -EINVAL;
		dprintk(2, "flash read addr:%x, len:%x\n", p.addr, p.len);
		ret = send_flash_read(dev, p.addr, p.len, p.data, &p.len);
		if (ret < 0)
			return ret;
		ret = copy_to_user(argp, &p, sizeof(p));
		break;
	}
	case S2226_VIDIOC_FLASH_ERASE:
	{
		struct flash_param p;
		if (dev->users > 1) {
			dev_info(&dev->udev->dev,
				 "s2226: other process using device\n");
			return -EBUSY;
		}
		if (copy_from_user(&p, argp, sizeof(p)))
			return -EINVAL;
		/* protect our memory space for */
		if (!mfgmode && (p.addr < 0x10000))
			return -EINVAL;
		dprintk(2, "flash erase addr:%x, len:%x\n", p.addr, p.len);
		ret = send_flash_erase(dev, p.addr, p.len);
		dev_info(&dev->udev->dev, "s2226: flash erase %d\n", ret);
		if (ret < 0)
			return ret;
		break;
	}
	case S2226_VIDIOC_BOOT_H51:
		dev_info(&dev->udev->dev, "s226: ioc boot h51\n");
		if (res_locked(dev, fh) & RES_STREAM) {
			dev_info(&dev->udev->dev,
				 "s2226: boot, device busy(streaming)\n");
			return -EBUSY;
		}
		ret = send_h51_boot(dev);
		if (ret < 0)
			return ret;
		break;
#if 0
	case S2226_VIDIOC_BOOT_FPGA:
		if (res_locked(dev, fh) & RES_STREAM) {
			dev_info(&dev->udev->dev,
				 "s2226: boot, device busy(streaming)\n");
			return -EBUSY;
		}
		ret = send_fpga_boot(dev);
		if (ret < 0)
			return ret;
		break;
#endif
	case S2226_VIDIOC_GET_INPUT:
		dprintk(3, "get input %d\n", (int)dev->cur_input);
		ret = copy_to_user(argp, &dev->cur_input, sizeof(cmd));
		break;
	case S2226_VIDIOC_AUDIO_WR:
	{
		audio_reg_t cmd;
		if (copy_from_user(&cmd, argp, sizeof(cmd)))
			return -EINVAL;
		dprintk(3, "ioc_audio_wr  %d %x\n", cmd.addr, cmd.value);
		ret = send_aic33_wr(dev, cmd.addr, cmd.value);
		if (ret < 0)
			return ret;
		break;
	}
	case S2226_VIDIOC_AUDIO_RD:
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
	case S2226_VIDIOC_VIDDEC_RD:
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
	case S2226_VIDIOC_SDISPLIT_RD:
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
	case S2226_VIDIOC_RESET_BOARD:
		/* If resetting from user space, should do the following
		   S2226_VIDIOC_RESET_BOARD
		   sleep(2)
		   S2226_VIDIOC_PRIME_FX2
		   S2226_VIDIOC_DEFAULT_PARAMS */
		ret = s2226_reset_board(dev);
		break;
	case S2226_VIDIOC_ARM_VER:
		ret = copy_to_user(argp, &dev->arm_ver, sizeof(cmd));
		break;
	case S2226_VIDIOC_SET_BASEFW:
		ret = s2226_fx2sam(dev, 0);
		dprintk(2, "%s FX2SAM_LO %d\n", __func__, ret);
		break;
	case S2226_VIDIOC_SET_NEWFW:
		ret = s2226_fx2sam(dev, 1);
		dprintk(2, "%s FX2SAM_HI %d\n", __func__, ret);
		break;
	case S2226_VIDIOC_NOP:
		/* sends NOP to the ARM. For use in firmware
		   if ARM does not */
		ret = s2226_send_nop(dev, 1);
		dprintk(2, "%s NOP %d\n", __func__, ret);
		break;
	case S2226_VIDIOC_RESET_USB:
		/* if firmware corrupt, we may have to reset the USB also */
		if (dev->users > 1) {
			dev_info(&dev->udev->dev,
				 "s2226: RESET_USB device busy\n");
			return -EBUSY;
		}
		ret = usb_reset_device(dev->udev);
		dprintk(2, "%s USB_RESET %d\n", __func__, ret);
		break;
	case S2226_VIDIOC_PRIME_FX2:
		if (dev->users > 1) {
			dev_info(&dev->udev->dev,
				 "s2226: PRIME_FX2 device busy\n");
			return -EBUSY;
		}
		ret = s2226_prime_fx2(dev);
		dprintk(2, "%s PRIME_FX2 %d\n", __func__, ret);
		break;
	case S2226_VIDIOC_DEFAULT_PARAMS:
		ret = s2226_default_params(dev);
		break;
	case S2226_VIDIOC_DPB_SIZE:
		dprintk(3, "%s set DPB SIZE %d\n", __func__, (int)arg);
		dev->dpb_size = (int)arg;
		break;
	case S2226_VIDIOC_GOP_STRUCT:
		dprintk(3, "%s set GOP STRUCT %d\n", __func__, (int)arg);
		dev->gop_struct = (int)arg;
		break;
	case S2226_VIDIOC_AINOFFSET:
		dprintk(3, "%s set AINOFFSET %d\n", __func__, (int)arg);
		dev->ainoffset = (int)arg;
		break;
	case S2226_VIDIOC_AV_RESYNC:
		dprintk(3, "%s set AV_RESYNC_THRESHOLD %d\n",
			__func__, (int)arg);
		dev->avresync = (int)arg;
		break;
	case S2226_VIDIOC_FX2_VER:
	{
		char buf[64];
		int siz = 64; /* EP0 is 64 bytes max */
		ret = s2226_vendor_request(dev,
					   0x30,/* IOCTL_USBSAMP_FIRMWARE_REV*/
					   0, 0,	/* val, idx*/
					   buf,
					   siz,
					   0);	/* bIn */
		if (ret > 0)
			dprintk(0, "USB firmware version %x.%02x\n",
				buf[1], buf[0]);
		else
			dprintk(0, "error getting USB firmware version\n");
		ret = buf[0] + (buf[1] << 8);
		break;
	}
	case S2226_VIDIOC_LOCK_OVERLAY:
	{
		if (!ovl_get(dev, file)) {
			printk(KERN_INFO "overlay already locked\n");
			return -EBUSY;
		}
		ret = 0;
		break;
	}
	case S2226_VIDIOC_UNLOCK_OVERLAY:
	{
		if (ovl_check(dev, file))
			ovl_free(dev);
		ret = 0;
		break;
	}
	default:
		ret = video_ioctl2(file, cmd, arg);
		break;
	}

	return ret;
}

static void s2226_release(struct v4l2_device *v4l2_dev)
{
	struct s2226_dev *dev = container_of(v4l2_dev, struct s2226_dev,
					     v4l2_dev);
	int i;

	v4l2_device_unregister(&dev->v4l2_dev);
	kfree(dev->interrupt_buffer);
	kfree(dev->control_buffer);
	for (i = 0; i < WR_URBS; i++) {
		if (dev->d->write_vid[i].buffer)
			usb_free_urb(dev->d->write_vid[i].urb);
		kfree(dev->d->write_vid[i].buffer);
		dev->d->write_vid[i].buffer = NULL;
	}
	for (i = 0; i < RD_URBS; i++) {
		if (dev->m->read_vid[i].buffer)
			usb_free_urb(dev->m->read_vid[i].urb);
		kfree(dev->m->read_vid[i].buffer);
		dev->m->read_vid[i].buffer = NULL;
		if (dev->p->read_vid[i].buffer)
			usb_free_urb(dev->p->read_vid[i].urb);
		kfree(dev->p->read_vid[i].buffer);
		dev->p->read_vid[i].buffer = NULL;
	}
	usb_free_urb(dev->interrupt_urb);
	usb_free_urb(dev->control_urb);
    vfree(dev->p->f1);
	kfree(dev);
	pr_info("s2226 memory released\n");
}



static void s2226_read_vid_callback(struct urb *u)
{
	struct s2226_urb *urb;
	struct s2226_dev *dev;
	int urb_context;
	urb = (struct s2226_urb *)u->context;
	if (urb == NULL) {
		printk(KERN_DEBUG "read vid callback failed\n");
		return;
	}
	dev = urb->dev;
	if (dev == NULL) {
		printk(KERN_DEBUG "null device\n");
		return;
	}
	urb_context = urb->context;
	urb->ready = 1;
	dprintk(4, "s2226_read_vid_callback %d\n", 0);
	if (urb_context == S2226_CONTEXT_V4L) {
		s2226_got_data(urb->strm, u->transfer_buffer, u->actual_length);
		/* submit the urb again */
		dprintk(4, "data from URB %d, resubmit URB\n", 0);
		usb_fill_bulk_urb(u, dev->udev,
				  usb_rcvbulkpipe(dev->udev, dev->ep[INDEX_EP_H264]),
				  (void *) u->transfer_buffer,
				  u->transfer_buffer_length,
				  s2226_read_vid_callback,
				  urb);
		(void) usb_submit_urb(u, GFP_ATOMIC);
	}
	return;
}


static void s2226_read_preview_callback(struct urb *u)
{
	struct s2226_urb *urb;
	struct s2226_dev *dev;
	int urb_context;
	urb = (struct s2226_urb *)u->context;
	if (!urb->strm->active) {
		dprintk(1, "got urb, not active\n");
		return;
	}
	dev = urb->dev;
	urb_context = urb->context;
	urb->ready = 1;
	dprintk(4, "s2226_read_vid_callback %d\n", u->actual_length);
	s2226_got_preview_data(urb->strm, u->transfer_buffer, u->actual_length);

	/* submit the urb again */
	dprintk(4, "data from URB %d, resubmit URB\n", 0);
	usb_fill_bulk_urb(u, dev->udev,
			  usb_rcvbulkpipe(dev->udev, dev->ep[INDEX_EP_RAW]),
			  (void *) u->transfer_buffer,
			  u->transfer_buffer_length,
			  s2226_read_preview_callback,
			  urb);
	(void) usb_submit_urb(u, GFP_ATOMIC);
	return;
}

static void s2226_write_vid_callback(struct urb *u)
{
	struct s2226_urb *urb;
	int i;

	urb = (struct s2226_urb *)u->context;
	urb->ready = 1;
	urb->dev->d->write_vid_ready = 1;
	for (i = 0; urb != &urb->dev->d->write_vid[i]; i++)
		; /* find the index of the urb that completed */
	dprintk(4, "s2226_write_vid_callback %d\n", i);
	wake_up(&urb->dev->d->write_vid_wq);
	return;
}

static ssize_t s2226_write(struct file *file, const char *buffer, size_t nbytes,
			   loff_t *ppos)
{
	int retval;
	struct s2226_stream *strm = video_drvdata(file);
	size_t writesize;
	int i = 0;

	writesize = (nbytes > MAX_USB_SIZE) ? MAX_USB_SIZE : nbytes;

	if (writesize == 0)
		return 0;
	if (strm->type != S2226_STREAM_DECODE)
		return -EINVAL;
	if (strm->write_vid[0].buffer == NULL)
		return -EFAULT;
	/* find an available urb */
	while (!strm->write_vid[i].ready) {
		i++;
		if (i >= WR_URBS) {
			i = 0;
			strm->write_vid_ready = 0;
			retval = wait_event_interruptible_timeout(strm->write_vid_wq,
								  (strm->write_vid_ready != 0),
								  msecs_to_jiffies(5000));
			if (retval <= 0) {
				dprintk(2, "retval %x\n", retval);
				return retval;
			}
			if (!strm->write_vid_ready) {
				dprintk(2, "s2226 write fault\n");
				return -EFAULT;
			}
		}
	}
	dprintk(4, "s2226_write_vid: %d %p %d\n", i, strm->write_vid[i].buffer, (int)writesize);

	if (copy_from_user(strm->write_vid[i].buffer, buffer, writesize)) {
		printk(KERN_ERR "s2226 write video\n");
		return -EFAULT;
	}
	if (strm->dev->udev == NULL)
		return -ENODEV;
	strm->write_vid[i].ready = 0;
	usb_fill_bulk_urb(strm->write_vid[i].urb, strm->dev->udev,
			  usb_sndbulkpipe(strm->dev->udev,
					  strm->dev->ep[INDEX_EP_H264]),
			  (void *) strm->write_vid[i].buffer,
			  writesize,
			  s2226_write_vid_callback,
			  (void *) &strm->write_vid[i]);
	retval = usb_submit_urb(strm->write_vid[i].urb, GFP_KERNEL);
	if (retval) {
		printk(KERN_ERR "failed to submit URB[%p] %d\n",
		       strm->write_vid[i].urb,
		       retval);
		return retval;
	}
	return writesize;
}



static int read_interrupt_endpoint(struct s2226_dev *dev, int timeout)
{
	int interruptsize = MAX_USB_INT_SIZE;
	int retval;
	int actual_length;
	char *pdata;
	retval = usb_bulk_msg(dev->udev,
			      usb_rcvbulkpipe(
				      dev->udev, dev->ep[INDEX_EP_RESP]),
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
	if (!dev->udev)
		return -ENODEV;
	retval = usb_bulk_msg(dev->udev,
			      usb_sndbulkpipe(dev->udev,
					      dev->ep[INDEX_EP_CONTROL]),
			      dev->control_buffer,
			      controlsize,
			      &actual_length,
			      timeout);
	if (retval < 0)
		dprintk(1, "write control endpoint got %d\n", retval);
	return retval;
}



#define EXPECTED_IN_EPS  1 /* minimum number */
#define EXPECTED_OUT_EPS 2

static void s2226_init_audio(struct s2226_audio *aud)
{
	memset(aud, 0, sizeof(struct s2226_audio));
	aud->iRoute = -1;
	return;
}

static int s2226_probe(struct usb_interface *interface,
		       const struct usb_device_id *id)
{
	struct s2226_dev *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i;
	int in_count = 0; /* number of in endpoints */
	int out_count = 0; /* number of out endpoints */
	int rc = -ENOMEM;
	/* required sleep for device to boot.  do not remove */
	msleep(1150);
	pr_info("[s2226] %s\n", S2226_VERSION);

	/* allocate memory for our device state and initialize it to zero */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		pr_info("s2226: out of memory");
		goto error;
	}
	/* initialize locks */
	s2226_mutex_init(&dev->ioctl_lock);
	s2226_mutex_init(&dev->audlock);
	s2226_mutex_init(&dev->reslock);
	s2226_mutex_init(&dev->cmdlock);
	spin_lock_init(&dev->slock);
	s2226_mutex_init(&dev->lock);
	dev->m = &dev->strm[S2226_STREAM_MPEG];
	dev->p = &dev->strm[S2226_STREAM_PREVIEW];
	dev->d = &dev->strm[S2226_STREAM_DECODE];
	dev->m->dev = dev;
	dev->p->dev = dev;
	dev->d->dev = dev;
	dev->m->type = S2226_STREAM_MPEG;
	dev->p->type = S2226_STREAM_PREVIEW;
	dev->d->type = S2226_STREAM_DECODE;
    dev->p->f1 = vmalloc(1920*1280);
    if (dev->p->f1 == NULL) {
		pr_info("s2226: out of memory");
		goto error;
    }
	dev->fpga_ver = -1;
	dev->cfg_intf = -1;
	dev->alt_intf = -1;

	s2226_init_audio(&dev->aud);
	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;
	/* set up the endpoint information */
	iface_desc = interface->cur_altsetting;
	dprintk(4, "num endpoints %d\n", iface_desc->desc.bNumEndpoints);
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
		pr_info("Did not find correct number of endpoints\n");
		goto error;
	}
	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);
	/* let the user know what node this device is now attached to */
	dprintk(0, "USB 2226 device now attached to USBs2226-%d",
		interface->minor);
	dev->interrupt_buffer = kmalloc(MAX_USB_INT_SIZE, GFP_KERNEL);
	if (dev->interrupt_buffer == NULL) {
		rc = -ENOMEM;
		goto errorUR;
	}
	dev->control_buffer = kmalloc(MAX_USB_SIZE, GFP_KERNEL);
	if (dev->control_buffer == NULL) {
		rc = -ENOMEM;
		goto errorUR;
	}
	dev->interrupt_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (dev->interrupt_urb == NULL) {
		rc = -ENOMEM;
		goto errorUR;
	}
	dev->control_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (dev->control_urb == NULL) {
		rc = -ENOMEM;
		goto errorUR;
	}
	for (i = 0; i < S2226_MAX_STREAMS; i++) {
		rc = s2226_allocate_urbs(&dev->strm[i]);
		if (rc != 0) {
			rc = -ENOMEM;
			goto errorUR;
		}
	}

	init_waitqueue_head(&dev->d->write_vid_wq);
	dev->d->write_vid_ready = 1;
	dev_info(&dev->udev->dev, "s2226 successfully loaded\n");
	{
		char buf[64];
		int siz = 64; /* EP0 is 64 bytes max */
		rc = s2226_vendor_request(dev,
					      0x30,/* FIRMWARE_REV*/
					      0, 0, /* val, idx*/
					      buf,
					      siz,
					      0); /* bIn */
		if (rc > 0)
			dprintk(0, "USB firmware version %x.%02x\n",
				buf[1], buf[0]);
		else
			dprintk(0, "error getting USB firmware version\n");
		dev->usb_ver = buf[0] + (buf[1] << 8);
	}
	s2226_prime_fx2(dev);
	rc = s2226_set_attr(dev, ATTR_INT_EP_PKTEND, 0);
	{
		int fwver;
		rc = s2226_get_attr(dev, ATTR_ARM_FW_VERSION, &fwver);
		if (rc >= 0) {
			dev_info(&dev->udev->dev,
				 "s2226: ARM fw version: 0x%04x\n",
				 fwver);
			dev->arm_ver = fwver;
		} else
			dev_info(&dev->udev->dev,
			       "s2226: err getting ARM fw version %d\n",
			       rc);

		if (fwver & 0xff000000)
			dev_info(&dev->udev->dev,
				 "s2226: development firmware ONLY!\n");
		else if (fwver < 0x24)
			dev_info(&dev->udev->dev,
				 "s2226: firmware out of date!\n");
	}
	/* set default parameters */
	s2226_default_params(dev);
	dprintk(4, "before probe done %p\n", dev);
	dev->h51_mode.gop = 0;
	dev->h51_mode.vbr = 0;
	dev->h51_mode.vBitrate = S2226_DEF_VBITRATE;
	dev->h51_mode.aBitrate = S2226_DEF_ABITRATE;
	dev->h51_mode.aMode = AMODE_MP1L2;
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
	s2226_get_fpga_ver(dev);
	s2226_set_interface(dev, 0, 1, 1);
	dev_info(&dev->udev->dev, "s2226: probe success\n");
	return 0;
errorUR:
error:
	v4l2_device_unregister(&dev->v4l2_dev);
	kfree(dev);
	pr_info("s2226: probe failed\n");
	return rc;
}

/* disconnect routine.  when board is removed physically or with rmmod */
static void s2226_disconnect(struct usb_interface *interface)
{
	struct s2226_dev *dev = usb_get_intfdata(interface);
	int minor = interface->minor;
	int i;
	/* wake up the write queue */
	dprintk(2, "%s\n", __func__);
	wake_up(&dev->d->write_vid_wq);
	s2226_mutex_lock(&dev->lock);
	usb_set_intfdata(interface, NULL);
	for (i = 0; i < S2226_MAX_STREAMS; i++)
		video_unregister_device(&dev->strm[i].vdev);
	v4l2_device_disconnect(&dev->v4l2_dev);
	dev->udev = NULL;
	s2226_mutex_unlock(&dev->lock);
	v4l2_device_put(&dev->v4l2_dev);
	pr_info("USB s2226 #%d now disconnected", minor);
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
	/* register this driver with the USB subsystem */
	result = usb_register(&s2226_driver);
	if (result)
		pr_info("s2226drv: usb_register failed. Error number %d",
			result);
	dprintk(2, "s2226_init: done\n");
	return result;
}

static void __exit usb_s2226_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&s2226_driver);
}

static int s2226_vendor_request(void *pdev, unsigned char req,
				unsigned short idx, unsigned short val,
				void *pbuf, unsigned int len,
				int bOut)
{
	int r;
	struct s2226_dev *dev = (struct s2226_dev *) pdev;
	if (len > 64) {
		pr_info("%s : invalid VR len:trunc to EP0 size.\n", __func__);
		len = 64;
	}
	if (!bOut) {
		r = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
				    req,
				    USB_TYPE_VENDOR |
				    USB_RECIP_DEVICE | USB_DIR_IN,
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
				    usb_rcvbulkpipe(
					    dev->udev, dev->ep[INDEX_EP_H264]),
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
	/* does not apply to latest firmware */
	if (dev->usb_ver >= 0x20)
		return -EINVAL;
	/* send vendor cmd to assert ARM reset */
	ret = s2226_vendor_request(dev, S2226_VR_ARM_RESET_ASSERT,
				   0, 0, buf, siz, 0);
	msleep(20);
	/* send vendor cmd to de-assert ARM reset */
	ret = s2226_vendor_request(dev,
				   S2226_VR_ARM_RESET_DEASSERT,
				   0, 0, buf, siz, 0);
	/* wait for board to come up */
	msleep(1150);
	/* board reset, make sure board put back at default interface. */
	s2226_set_interface(dev, 0, 0, 1);
	return ret;
}

#define S2226_VR_LOAD_NEWFW   0x57  /* FX2SAM high */
#define S2226_VR_LOAD_ORIGFW  0x58  /* FX2SAM low */

static int s2226_fx2sam(struct s2226_dev *dev, int bNewFw)
{
	char buf[64];
	int siz = 64;
	int rc;
	rc = s2226_vendor_request(dev,
				  bNewFw ?
				  S2226_VR_LOAD_NEWFW : S2226_VR_LOAD_ORIGFW,
				  0, 0, buf, siz, 0);
	return rc;
}

static int s2226_default_params(struct s2226_dev *dev)
{
	int rc;
	/* switch to pktend mode for efficiency */
	rc = s2226_set_attr(dev, ATTR_INT_EP_PKTEND, 1);
	if (rc != 0)
		return rc;
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
		/* turn on AGC right */
		dprintk(2, "AGC R on, gain %d\n", gain);
		send_aic33_wr(dev, 29, 0x80);
		send_aic33_wr(dev, 30, (unsigned char) gain);
		send_aic33_wr(dev, 31, 0x3e);
	} else {
		/* turn off AGC right */
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
		/* turn on AGC left */
		dprintk(2, "AGC L on, gain %d\n", gain);
		send_aic33_wr(dev, 26, 0x80);
		send_aic33_wr(dev, 27, (unsigned char) gain);
		send_aic33_wr(dev, 28, 0x3e);
	} else {
		/* turn off AGC left */
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
		send_aic33_wr(dev, 19, bBal ? 0x84 : 0x04);
		/*LINE1L to Left ADC*/
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
		send_aic33_wr(dev, 22, bBal ? 0x84 : 0x04);
		/*LINE1R to Left ADC*/
		break;
	}
	return 0;
}

static int SetAudioIn(struct s2226_dev *dev)
{
	/* input must be set */
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


static int SetAudioVol(struct s2226_dev *dev, int gainL, int gainR,
		       int muteL, int muteR)
{
	unsigned char val;
	dev->aud.bVolMuteL = muteL;
	dev->aud.bVolMuteR = muteR;
	dev->aud.iVolGainL = gainL;
	dev->aud.iVolGainR = gainR;

	if (!dev->input_set)
		return -1;
	/* set audio out DAC settings (right, left) */
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

	/* set audio out Mono settings(extra gain, balanced) */
	if (dev->aud.iMonoGain > 9)
		dev->aud.iMonoGain = 9;
	if (!dev->input_set)
		return -1;
	val = dev->aud.iMonoGain << 4 | (dev->aud.bMonoMute ? 0x01 : 0x09);
	send_aic33_wr(dev, 79, val);
	return 0;
}

static int SetAudioHp(struct s2226_dev *dev, int gainL, int gainR,
		      int muteL, int muteR)
{
	dev->aud.iHpGainL = gainL;
	dev->aud.iHpGainR = gainL;
	dev->aud.bHpMuteR = muteR;
	dev->aud.bHpMuteL = muteL;

	/* set audio out Hp settings(extra gain, balanced) */
	if (dev->aud.iHpGainL > 9)
		dev->aud.iHpGainL = 9;
	if (dev->aud.iHpGainR > 9)
		dev->aud.iHpGainR = 9;
	if (!dev->input_set)
		return -1;
	send_aic33_wr(dev, 58, dev->aud.iHpGainL << 4 | 0x05);/* common left */
	send_aic33_wr(dev, 51, dev->aud.iHpGainL << 4 |
		      (dev->aud.bHpMuteL ? 0x05 : 0x0d));/*out left*/
	send_aic33_wr(dev, 65, dev->aud.iHpGainR << 4 |
		      (dev->aud.bHpMuteR ? 0x05 : 0x0d));/*out right*/
	/* power down the common on 2226, not connected */
	send_aic33_wr(dev, 72, dev->aud.iHpGainR << 4 | 0x05);/*right common*/
	return 0;
}


static int SetAudioStereo(struct s2226_dev *dev, int gainL, int gainR,
			  int muteL, int muteR)
{
	unsigned char val;
	dev->aud.iStereoGainL = gainL;
	dev->aud.iStereoGainR = gainL;
	dev->aud.bStereoMuteR = muteR;
	dev->aud.bStereoMuteL = muteL;

	/* set audio out Stereo settings(extra gain, balanced) */
	if (dev->aud.iStereoGainL > 9)
		dev->aud.iStereoGainL = 9;
	if (dev->aud.iStereoGainR > 9)
		dev->aud.iStereoGainR = 9;
	if (!dev->input_set)
		return -1;
	val = dev->aud.iStereoGainL << 4 |
		(dev->aud.bStereoMuteL ? 0x01 : 0x09);
	send_aic33_wr(dev, 86, val);
	val = dev->aud.iStereoGainR << 4 |
		(dev->aud.bStereoMuteR ? 0x01 : 0x09);
	send_aic33_wr(dev, 93, val);
	return 0;
}

static int SetAudioOut(struct s2226_dev *dev)
{
	if (!dev->input_set)
		return -1;

	SetAudioVol(dev, dev->aud.iVolGainR,
		    dev->aud.iVolGainL, dev->aud.bVolMuteL,
		    dev->aud.bVolMuteR);
	SetAudioMono(dev, dev->aud.iMonoGain, dev->aud.bMonoMute);
	SetAudioHp(dev, dev->aud.iHpGainL,
		   dev->aud.iHpGainR, dev->aud.bHpMuteL,
		   dev->aud.bHpMuteR);
	SetAudioStereo(dev, dev->aud.iStereoGainL,
		       dev->aud.iStereoGainR, dev->aud.bStereoMuteL,
		       dev->aud.bStereoMuteR);
	return 0;
}

/* also sets line input balanced or not */
static int SetAudioRoute(struct s2226_dev *dev, int route)
{
	if (route == -1)
		route = AUDIOROUTE_LINE1L;
	dev->aud.iRoute = route;
	if (!dev->input_set)
		return -1;
	dprintk(2, "audio route %d\n", route);
	/* MIC3L/R to Left ADC */
	send_aic33_wr(dev, 17, 0xff);
	/* MIC3L/R to Right ADC */
	send_aic33_wr(dev, 18, 0xff);
	if (dev->is_decode) {
		/* ADC is powered down */
		dprintk(0, "is_decode, turning AGC off\n");
		send_aic33_wr(dev, 15, 0x80);/*ADC PGA muted*/
		send_aic33_wr(dev, 16, 0x80);/*ADC PGA muted*/
		send_aic33_wr(dev, 19, 0x78);/*powered down*/
		send_aic33_wr(dev, 20, 0x7c);/*LINE2L to Left ADC (off)*/
		send_aic33_wr(dev, 21, 0x78);/*LINE1R to Left ADC (off)*/
		send_aic33_wr(dev, 22, 0x78);/*LINE1R to right ADC*/
		send_aic33_wr(dev, 23, 0x7c);/*LINE2R to right ADC (off)*/
		send_aic33_wr(dev, 24, 0x78);/*LINE1L to right ADC (off)*/
	} else {
		switch (route) {
		case AUDIOROUTE_LINE1L:
		case AUDIOROUTE_LINE1L_BYPASS:
		default:
			/*LINE1L to Left ADC*/
			send_aic33_wr(dev, 19, dev->aud.in_balL ? 0x84 : 0x04);
			/*LINE2L to Left ADC (off)*/
			send_aic33_wr(dev, 20, 0x7c);
			/*LINE1R to Left ADC (off)*/
			send_aic33_wr(dev, 21, 0x78);
			/*LINE1R to right ADC*/
			send_aic33_wr(dev, 22, dev->aud.in_balR ? 0x84 : 0x04);
			/*LINE2R to right ADC (off)*/
			send_aic33_wr(dev, 23, 0x7c);
			/*LINE1L to right ADC (off)*/
			send_aic33_wr(dev, 24, 0x78);
			break;
		case AUDIOROUTE_LINE2L:
		case AUDIOROUTE_LINE2L_BYPASS:
			/* LINE1L OFF (+ADC powered up)*/
			send_aic33_wr(dev, 19, 0x7c);
			/*LINE2L to Left ADC (on, differential)*/
			send_aic33_wr(dev, 20, 0x84);
			/*LINE1R to Left ADC (off)*/
			send_aic33_wr(dev, 21, 0x78);
			/* LINE1R OFF (+ADC powered up)*/
			send_aic33_wr(dev, 22, 0x7c);
			/*LINE2R to right ADC (on, differential)*/
			send_aic33_wr(dev, 23, 0x84);
			/*LINE1L to right ADC (off)*/
			send_aic33_wr(dev, 24, 0x78);
			break;
		}
	}

	switch (route) {
	case AUDIOROUTE_LINE1L: /* LINE1L routed */
	case AUDIOROUTE_LINE2L: /* LINE2L routed */
		/* to HPLOUT*/
		send_aic33_wr(dev, 45, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 46, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 47, 0x80); /*DAC_L1*/
		send_aic33_wr(dev, 48, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 49, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 50, 0x00); /*DAC_R1*/
		/* to HPLCOM*/
		send_aic33_wr(dev, 52, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 53, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 54, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 55, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 56, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 57, 0x00); /*DAC_R1*/
		/* to HPROUT*/
		send_aic33_wr(dev, 59, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 60, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 61, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 62, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 63, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 64, 0x80); /*DAC_R1*/
		/* to HPRCOM*/
		send_aic33_wr(dev, 66, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 67, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 68, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 69, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 70, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 71, 0x00); /*DAC_R1*/
		/* to mono settings*/
		send_aic33_wr(dev, 73, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 74, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 75, 0x80); /*DAC_L1*/
		send_aic33_wr(dev, 76, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 77, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 78, 0x80); /*DAC_R1*/
		/* to stereo left settings*/
		send_aic33_wr(dev, 80, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 81, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 82, 0x80); /*DAC_L1*/
		send_aic33_wr(dev, 83, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 84, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 85, 0x00); /*DAC_R1*/
		/* to stereo right*/
		send_aic33_wr(dev, 87, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 88, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 89, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 90, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 91, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 92, 0x80); /*DAC_R1*/
		break;
	case AUDIOROUTE_LINE1L_BYPASS:
		/* to HPLOUT*/
		send_aic33_wr(dev, 45, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 46, 0x80); /*PGA_L*/
		send_aic33_wr(dev, 47, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 48, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 49, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 50, 0x00); /*DAC_R1*/
		/* to HPLCOM*/
		send_aic33_wr(dev, 52, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 53, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 54, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 55, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 56, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 57, 0x00); /*DAC_R1*/
		/* to HPROUT*/
		send_aic33_wr(dev, 59, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 60, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 61, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 62, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 63, 0x80); /*PGA_R*/
		send_aic33_wr(dev, 64, 0x00); /*DAC_R1*/
		/* to HPRCOM*/
		send_aic33_wr(dev, 66, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 67, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 68, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 69, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 70, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 71, 0x00); /*DAC_R1*/
		/* to mono settings*/
		send_aic33_wr(dev, 73, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 74, 0x80); /*PGA_L*/
		send_aic33_wr(dev, 75, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 76, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 77, 0x80); /*PGA_R*/
		send_aic33_wr(dev, 78, 0x00); /*DAC_R1*/
		/* to stereo left settings*/
		send_aic33_wr(dev, 80, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 81, 0x80); /*PGA_L*/
		send_aic33_wr(dev, 82, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 83, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 84, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 85, 0x00); /*DAC_R1*/
		/* to stereo right*/
		send_aic33_wr(dev, 86, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 87, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 88, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 89, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 90, 0x80); /*PGA_R*/
		send_aic33_wr(dev, 91, 0x00); /*DAC_R1*/
		dprintk(2, "bypass line 1\n");
		break;
	case AUDIOROUTE_LINE2L_BYPASS:
		/* to HPLOUT*/
		send_aic33_wr(dev, 45, 0x80); /*LINE2L*/
		send_aic33_wr(dev, 46, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 47, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 48, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 49, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 50, 0x00); /*DAC_R1*/
		/* to HPLCOM*/
		send_aic33_wr(dev, 52, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 53, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 54, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 55, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 56, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 57, 0x00); /*DAC_R1*/
		/* to HPROUT*/
		send_aic33_wr(dev, 59, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 60, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 61, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 62, 0x80); /*LINE2R*/
		send_aic33_wr(dev, 63, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 64, 0x00); /*DAC_R1*/
		/* to HPRCOM*/
		send_aic33_wr(dev, 66, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 67, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 68, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 69, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 70, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 71, 0x00); /*DAC_R1*/
		/* to mono settings*/
		send_aic33_wr(dev, 73, 0x80); /*LINE2L*/
		send_aic33_wr(dev, 74, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 75, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 76, 0x80); /*LINE2R*/
		send_aic33_wr(dev, 77, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 78, 0x00); /*DAC_R1*/
		/* to stereo left settings*/
		send_aic33_wr(dev, 80, 0x80); /*LINE2L*/
		send_aic33_wr(dev, 81, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 82, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 83, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 84, 0x00); /*PGA_R*/
		send_aic33_wr(dev, 85, 0x00); /*DAC_R1*/
		/* to stereo right*/
		send_aic33_wr(dev, 80, 0x00); /*LINE2L*/
		send_aic33_wr(dev, 81, 0x00); /*PGA_L*/
		send_aic33_wr(dev, 82, 0x00); /*DAC_L1*/
		send_aic33_wr(dev, 83, 0x00); /*LINE2R*/
		send_aic33_wr(dev, 84, 0x80); /*PGA_R*/
		send_aic33_wr(dev, 85, 0x00); /*DAC_R1*/
		break;
	}
	dev->aud.iRoute = route;
	return 0;
}

static int s2226_get_fpga_ver(struct s2226_dev *dev)
{
	/* get fpga firmware version */
	int fwver = 0;
	int retval;
	retval = get_reg(dev, DEVID_FPGA, (0x21 << 1), &fwver);
	if (retval >= 0)
		dev_info(&dev->udev->dev,
			 "s2226: board_id %d\n"
			 "s2226: FPGA ver: 0x%04x\n",
			 (fwver >> 11), (fwver & 0x07ff));
	else {
		dev_info(&dev->udev->dev, "s2226: err FPGA ver %d\n", retval);
		return -1;
	}
	dev->fpga_ver = fwver & 0x07ff;
	dev->board_id = fwver >> 11;
	return 0;
}


#define S2226_NORMS		(V4L2_STD_PAL | V4L2_STD_NTSC)

static int s2226_open_v4l(struct file *file)
{
	struct s2226_fh *fh;
	struct s2226_stream *strm = video_drvdata(file);
	int rc;
	if (strm->type != S2226_STREAM_DECODE) {
		rc = v4l2_fh_open(file);
		dprintk(2, "%s: %d\n", __func__, rc);
		return rc;
	}
	/* decode stream type */
	fh = kmalloc(sizeof(struct s2226_fh), GFP_KERNEL);
	if (NULL == fh) {
		printk(KERN_INFO "s2226: out of memory\n");
		return -ENOMEM;
	}
	memset(fh, 0, sizeof(struct s2226_fh));
	file->private_data = fh;
	fh->dev = strm->dev;
	fh->strm = strm;
	return 0;
}


static int s2226_close(struct file *filp)
{
	int i;
	int res;
	struct s2226_fh *fh;
	struct s2226_stream *strm = video_drvdata(filp);
	struct s2226_dev *dev = strm->dev;

	if (ovl_check(dev, filp))
		ovl_free(dev);

	if (strm->type != S2226_STREAM_DECODE) {
		int rc;
		rc = vb2_fop_release(filp);
		return rc;
	}
	fh = filp->private_data;
	res = res_check(fh);
	if (res & RES_DECODE) {
		dprintk(2, "stop decode\n");
		for (i = 0; i < WR_URBS; i++) {
			usb_kill_urb(strm->write_vid[i].urb);
			strm->write_vid[i].ready = 1;
		}
		(void)  s2226_stop_decode(strm->dev, 0);
		res_free(strm->dev, fh, RES_DECODE);
	}
	fh = filp->private_data;
	kfree(fh);
	return 0;
}

static int s2226_got_data(struct s2226_stream *strm, unsigned char *tbuf, unsigned int tlen)
{
	struct s2226_buffer *buf;
	unsigned long flags = 0;
	char *vbuf;
	int rc = 0;
	dprintk(4, "%s\n", __func__);
	spin_lock_irqsave(&strm->qlock, flags);
	if (list_empty(&strm->buf_list)) {
		dprintk(0, "No active queue to serve\n");
		rc = -1;
		goto unlock;
	}
	buf = list_entry(strm->buf_list.next,
			 struct s2226_buffer, list);
	list_del(&buf->list);
	vbuf = vb2_plane_vaddr(&buf->vb, 0);
	if (vbuf == NULL) {
		printk(KERN_ERR "%s vbuf error\n", __func__);
		rc = -ENOMEM;
		goto unlock;
	}
	memcpy(vbuf, tbuf, tlen);
	buf->vb.v4l2_buf.length = tlen;
	v4l2_get_timestamp(&buf->vb.v4l2_buf.timestamp);
	buf->vb.v4l2_buf.sequence = strm->framecount++;
	vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE);

	dprintk(2, "wakeup [buf/i] [%p/%d]\n", buf, 0);
	rc = 0;
unlock:
	spin_unlock_irqrestore(&strm->qlock, flags);
	return rc;
}


static int s2226_got_preview_data(struct s2226_stream *strm, unsigned char *tbuf, unsigned int tlen)
{
	struct s2226_buffer *buf;
	unsigned long flags = 0;
	char *vbuf;
	int i;
	if (tlen == 2) {
		if ((tbuf[0] == INTTYPE_ASYNCEVENT) &&
		    (tbuf[1] == ASYNCEVENT_RAWDATA)) {
			strm->m_sync = 0;
			strm->m_copied = 0;
			strm->m_lines = 0;
			strm->m_pos = 0;
			return 0;
		}
		if ((tbuf[0] == INTTYPE_ASYNCEVENT) &&
		    (tbuf[1] == ASYNCEVENT_RAWDATA_END)) {
			strm->m_sync = 1;
			return 0;
		}
	}

	if (strm->m_sync)
		return 0;

	spin_lock_irqsave(&strm->qlock, flags);
	if (list_empty(&strm->buf_list)) {
		dprintk(3, "No active queue to serve\n");
		spin_unlock_irqrestore(&strm->qlock, flags);
		return -1;
	}
	buf = list_entry(strm->buf_list.next,
			 struct s2226_buffer, list);
	vbuf = vb2_plane_vaddr(&buf->vb, 0);

	if (vbuf == NULL) {
		printk(KERN_ERR "%s vbuf error\n", __func__);
		spin_unlock_irqrestore(&strm->qlock, flags);
		return -ENOMEM;
	}
	if (strm->m_sync) {
		for (i = 0; i < tlen; i++) {
			if (tbuf[i] == 0x22 && tbuf[i+1] == 0x26 && tbuf[i+2] == 0xda && tbuf[i+3] == 0x4a) {
				strm->m_sync = 0;
				break;
			}
		}
		strm->m_copied = 0;
		strm->m_lines = 0;
		strm->m_pos = 0;
		spin_unlock_irqrestore(&strm->qlock, flags);
		dprintk(2, "%x %x %x %X: msync %d, len %d\n", tbuf[0], tbuf[1], tbuf[2], tbuf[3], strm->m_sync,
		       tlen);
		return 0;
	}

	if (strm->m_pos + tlen >= vb2_plane_size(&buf->vb, 0))
		tlen = vb2_plane_size(&buf->vb, 0) - strm->m_pos;

	memcpy(vbuf + strm->m_pos, tbuf, tlen);
	strm->m_pos += tlen;
	if (strm->m_pos < vb2_get_plane_payload(&buf->vb, 0)) {
		spin_unlock_irqrestore(&strm->qlock, flags);
		return 0;
	}
	strm->m_sync = 1;
	list_del(&buf->list);
	spin_unlock_irqrestore(&strm->qlock, flags);

	if (!strm->single && (strm->type == S2226_STREAM_PREVIEW)) {
		int j;
		unsigned char *pf1, *pf2;
		unsigned char *pbuf;
		pf1 = vb2_plane_vaddr(&buf->vb, 0);
		pbuf = pf1;
		pf2 = pf1 + strm->height * strm->width;
		/* copy first field to temp store */
		memcpy(strm->f1, pf1, strm->height*strm->width);

		/* move second field into the frame */
		for (j = 0; j < strm->height / 2; j++) {
			pf1 += strm->width * 2;
			memcpy(pf1, pf2, strm->width * 2);
			pf2 += strm->width * 2;
			pf1 += strm->width * 2;
			if ((unsigned long) (pf1 - pbuf) >= (strm->width*strm->height*2))
				break;
		}
		/* move first field back into the frame */
		pf1 = strm->f1;
		pf2 = pbuf;
		for (j = 0; j < strm->height / 2; j++) {
			memcpy(pf2, pf1, strm->width * 2);
			pf2 += strm->width * 2;
			pf2 += strm->width * 2;
			pf1 += strm->width * 2;
			if ((unsigned long) (pf2 - pbuf) >= (strm->width*strm->height*2))
				break;
		}
	}

	buf->vb.v4l2_buf.length = vb2_get_plane_payload(&buf->vb, 0);
	v4l2_get_timestamp(&buf->vb.v4l2_buf.timestamp);
	strm->framecount++;
	buf->vb.v4l2_buf.sequence = strm->framecount;
	vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE);
	dprintk(2, "wakeup [buf/i] [%p/%d]\n", buf, 0);
	return 0;
}


/* videobuf2 info */
static int queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
		       unsigned int *nbuffers, unsigned int *nplanes,
		       unsigned int sizes[], void *alloc_ctxs[])
{
	struct s2226_stream *strm = vb2_get_drv_priv(vq);

	if (*nbuffers < S2226_MIN_BUFS)
		*nbuffers = S2226_MIN_BUFS;
	*nplanes = 1;
	switch (strm->type) {
	case S2226_STREAM_MPEG:
		sizes[0] = S2226_MPEG_BUFFER_SIZE;
		break;
	case S2226_STREAM_PREVIEW:
		sizes[0] = strm->width * strm->height * 2;
		break;
	}
	dprintk(2, "buffer setup size: %d\n", sizes[0]);
	return 0;
}


static int buffer_prepare(struct vb2_buffer *vb)
{
	struct s2226_buffer *buf = container_of(vb, struct s2226_buffer, vb);
	struct s2226_stream *strm = vb2_get_drv_priv(vb->vb2_queue);
	int w = strm->width;
	int h = strm->height;
	unsigned long size;
	dprintk(2, "%s: h %d, w %d\n", __func__, h, w);
	switch (strm->type) {
	case S2226_STREAM_MPEG:
		size = S2226_MPEG_BUFFER_SIZE;
		break;
	case S2226_STREAM_PREVIEW:
		size = w * h * 2;
		break;
	default:
		return -EINVAL;
	}
	if (vb2_plane_size(vb, 0) < size) {
		dprintk(2, "invalid buffer prepare\n");
		return -EINVAL;
	}
	vb2_set_plane_payload(&buf->vb, 0, size);
	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct s2226_buffer *buf = container_of(vb, struct s2226_buffer, vb);
	struct s2226_stream *strm = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long flags = 0;
	dprintk(4, "%s\n", __func__);
	spin_lock_irqsave(&strm->qlock, flags);
	list_add_tail(&buf->list, &strm->buf_list);
	spin_unlock_irqrestore(&strm->qlock, flags);
}


static int stop_streaming(struct vb2_queue *vq);
static int start_streaming(struct vb2_queue *vq, unsigned int count);

static struct vb2_ops s2226_vb2_ops = {
	.queue_setup = queue_setup,
	.buf_prepare = buffer_prepare,
	.buf_queue = buffer_queue,
	.start_streaming = start_streaming,
	.stop_streaming = stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;

	memset(cap, 0, sizeof(*cap));
	strlcpy(cap->driver, "s2226", sizeof(cap->driver));
	switch (strm->type) {
	case S2226_STREAM_MPEG:
		strlcpy(cap->card, "Sensoray Model 2226 H.264", sizeof(cap->card));
		break;
	case S2226_STREAM_PREVIEW:
		strlcpy(cap->card, "Sensoray Model 2226 Preview", sizeof(cap->card));
		break;
	case S2226_STREAM_DECODE:
		strlcpy(cap->card, "Sensoray Model 2226 Decode", sizeof(cap->card));
		break;
	}

	strlcpy(cap->bus_info, dev_name(&dev->udev->dev),
		sizeof(cap->bus_info));

	usb_make_path(dev->udev, cap->bus_info, sizeof(cap->bus_info));

    cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
    switch (strm->type) {
    default:
        cap->device_caps |= V4L2_CAP_VIDEO_CAPTURE;
        break;
    case S2226_STREAM_DECODE:
        cap->device_caps |= V4L2_CAP_VIDEO_OUTPUT;
        break;
    }
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}


struct s2226_fmt {
	char *name;
	u32 fourcc;
	int depth;
	u32 flags;
	int type;
};

/* formats */
static const struct s2226_fmt formats_mpeg[] = {
	{
		.name = "MPEGTS_H264",
		.fourcc = V4L2_PIX_FMT_MPEG,
		.depth = 8,
		.flags = V4L2_FMT_FLAG_COMPRESSED,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	}
};

static const struct s2226_fmt formats_preview[] = {
	{
		.name = "4:2:2, packed, UYVY",
		.fourcc = V4L2_PIX_FMT_UYVY,
		.depth = 16,
		.flags = 0,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	},
};

static void s2226_max_height_width(int input, int *h, int *w)
{
	switch (input) {
	case INPUT_COMP0_480I:
	case INPUT_COMP1_480I:
	case INPUT_SVIDEO0_480I:
	case INPUT_SVIDEO1_480I:
	case INPUT_H51_SD_480I:
	case INPUT_SDI_480I:
	case INPUT_SDI_480I_CB:
		*w = 640;
		*h = 480;
		return;
	case INPUT_SDI_576I:
	case INPUT_SDI_576I_CB:
	case INPUT_COMP0_576I:
	case INPUT_COMP1_576I:
	case INPUT_SVIDEO0_576I:
	case INPUT_SVIDEO1_576I:
	case INPUT_H51_SD_576I:
		*w = 704;
		*h = 576;
		return;
	case INPUT_SDI_720P_50:
	case INPUT_SDI_720P_50_CB:
	case INPUT_H51_HD_720P_50:
	case INPUT_SDI_720P_5994:
	case INPUT_SDI_720P_60:
	case INPUT_SDI_720P_60_CB:
	case INPUT_SDI_720P_5994_CB:
	case INPUT_H51_HD_720P_60:
	case INPUT_H51_HD_720P_5994:
	case INPUT_SDI_720P_24:
	case INPUT_SDI_720P_24_CB:
	case INPUT_H51_HD_720P_24:
	case INPUT_SDI_720P_2398:
	case INPUT_SDI_720P_2398_CB:
	case INPUT_H51_HD_720P_2398:
		*w = 1280;
		*h = 720;
		return;
	case INPUT_SDI_1080I_50:
	case INPUT_SDI_1080I_50_CB:
	case INPUT_H51_HD_1080I_50:
	case INPUT_SDI_1080I_5994:
	case INPUT_SDI_1080I_5994_CB:
	case INPUT_H51_HD_1080I_5994:
	case INPUT_SDI_1080I_60:
	case INPUT_SDI_1080I_60_CB:
	case INPUT_H51_HD_1080I_60:
	case INPUT_SDI_1080P_24:
	case INPUT_SDI_1080P_24_CB:
	case INPUT_H51_HD_1080P_24:
	case INPUT_SDI_1080P_2398:
	case INPUT_SDI_1080P_2398_CB:
	case INPUT_H51_HD_1080P_2398:
		*w = 1920;
		*h = 1080;
		return;
	default:
		*w = 640;
		*h = 480;
		return;
	}
}


static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	int index = 0;
	struct s2226_stream *strm = video_drvdata(file);
	const struct s2226_fmt *formats;
	dprintk(4, "%s 2226\n", __func__);
	if (f == NULL)
		return -EINVAL;

	index = f->index;

	if (index >= 1)
		return -EINVAL;

	formats = (strm->type != S2226_STREAM_PREVIEW) ? formats_mpeg : formats_preview;
	strlcpy(f->description, formats[index].name, sizeof(f->description));
	f->pixelformat = formats[index].fourcc;
	f->flags = formats[index].flags;
	f->type = formats[index].type;
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct s2226_stream *strm = video_drvdata(file);
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat = strm->fourcc;
	switch (strm->type) {
	case S2226_STREAM_PREVIEW:
		f->fmt.pix.field = strm->field;
		f->fmt.pix.width = strm->width;
		f->fmt.pix.height = strm->height;
		f->fmt.pix.bytesperline = strm->width*2;
		f->fmt.pix.sizeimage = strm->width*strm->height*2;
		f->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
		break;
	case S2226_STREAM_MPEG:
	case S2226_STREAM_DECODE:
		f->fmt.pix.bytesperline = 0;
		f->fmt.pix.sizeimage = S2226_MPEG_BUFFER_SIZE;
		f->fmt.pix.colorspace = 0;
		break;
	default:
		return -EINVAL;
	}
	f->fmt.pix.priv = 0;
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dprintk(4, "%s", __func__);
	return 0;
}



static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;
	int is_ntsc;
	int is_mpeg;
	int maxh;
	int maxw;
	const struct s2226_fmt *formats;

	formats = (strm->type != S2226_STREAM_PREVIEW) ? formats_mpeg : formats_preview;
	is_mpeg = (strm->type == S2226_STREAM_PREVIEW) ? 0 : 1;
	is_ntsc = !dev->v4l_is_pal;
	dprintk(4, "%s\n", __func__);

	if (f->fmt.pix.pixelformat != V4L2_PIX_FMT_MPEG &&
	    f->fmt.pix.pixelformat != V4L2_PIX_FMT_UYVY) {
		dprintk(1, "%s wrong format %x %x\n", __func__,
			f->fmt.pix.pixelformat, V4L2_PIX_FMT_UYVY);
		return -EINVAL;
	}
	f->fmt.pix.pixelformat = formats[0].fourcc;
	if (!is_mpeg) {
		int w, h;
		w = f->fmt.pix.width;
		h = f->fmt.pix.height;
		s2226_max_height_width(dev->cur_input, &maxh, &maxw);
		if (h >= maxh)
			h = maxh;
		if (w >= maxw)
			w = maxw;
		if (f->fmt.pix.field == V4L2_FIELD_ANY) {
			if (IS_PROG_INPUT(dev->cur_input))
				f->fmt.pix.field = V4L2_FIELD_TOP;
			else if (h <= 288)
				f->fmt.pix.field = V4L2_FIELD_TOP;
			else
				f->fmt.pix.field = V4L2_FIELD_INTERLACED;
		}
		f->fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
		f->fmt.pix.height = h;
		f->fmt.pix.width = w;
		f->fmt.pix.bytesperline = w * 2;
		f->fmt.pix.sizeimage = w * h * 2;
		f->fmt.pix.colorspace = V4L2_COLORSPACE_SMPTE170M;
		f->fmt.pix.priv = 0;
	} else {
		f->fmt.pix.field = V4L2_FIELD_NONE;
		f->fmt.pix.bytesperline = 0;
		f->fmt.pix.sizeimage = S2226_MPEG_BUFFER_SIZE;
		if (strm->type == S2226_STREAM_MPEG) {
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
			case S2226_INPUT_SDI_720P_24:
			case S2226_INPUT_SDI_720P_2398:
				f->fmt.pix.width = 1920;
				f->fmt.pix.height = 720;
				break;
			case S2226_INPUT_1080I_COLORBARS:
			case S2226_INPUT_SDI_1080I_5994:
			case S2226_INPUT_SDI_1080I_60:
			case S2226_INPUT_SDI_1080I_50:
			case S2226_INPUT_SDI_1080P_24:
			case S2226_INPUT_SDI_1080P_2398:
				f->fmt.pix.width = 1920;
				f->fmt.pix.height = 1080;
				break;
			}
		} else {
			/* decode */
			switch (dev->v4l_input) {
			case S2226_DECODE_480I:
			case S2226_DECODE_576I:
				f->fmt.pix.width = 720;
				f->fmt.pix.height = is_ntsc ? 480 : 576;
				break;
			case S2226_DECODE_720P_24:
			case S2226_DECODE_720P_5994:
			case S2226_DECODE_720P_2398:
			case S2226_DECODE_720P_60:
			case S2226_DECODE_720P_50:
				f->fmt.pix.width = 1920;
				f->fmt.pix.height = 720;
				break;
			case S2226_DECODE_1080P_24:
			case S2226_DECODE_1080P_2398:
			case S2226_DECODE_1080I_60:
			case S2226_DECODE_1080I_5994:
			case S2226_DECODE_1080I_50:
				f->fmt.pix.width = 1920;
				f->fmt.pix.height = 1080;
				break;
			}
		}
	}
	return 0;
}

/* MPEG capture device so this really has no meaning.
   There is no scalar.  accept format specified, 0but width and height
   of image will be fixed in the MPEG stream. */
static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;
	struct s2226_fh *fh = priv;
	int ret;
	int single = 0;

	ret = vidioc_try_fmt_vid_cap(file, fh, f);
	if (ret < 0)
		return ret;

	if (strm->type != S2226_STREAM_PREVIEW)
		return 0;

	strm->width = f->fmt.pix.width;
	strm->height = f->fmt.pix.height;
	strm->fourcc = f->fmt.pix.pixelformat;
    strm->field = f->fmt.pix.field;

	s2226_set_attr(dev, ATTR_SCALE_X, f->fmt.pix.width);

	
	if (IS_PROG_INPUT(dev->cur_input))
		single = 1;

	if (f->fmt.pix.field != V4L2_FIELD_INTERLACED)
		single = 1;

	strm->single = single;
	//printk("single %d, height %d\n", single, f->fmt.pix.height);
	s2226_set_attr(dev, ATTR_SCALE_Y, single ? f->fmt.pix.height : f->fmt.pix.height/2);
	s2226_set_attr(dev, ATTR_SCALE_SINGLE, single);
/*	s2226_set_attr(dev, ATTR_SCALED_OUTPUT, 0);*/
	s2226_set_attr(dev, ATTR_MPEG_SCALER, 1);
	dprintk(1, "setting x scale %d, yscale %d\n", f->fmt.pix.width, f->fmt.pix.height);
	return 0;
}


static int s2226_allocate_urbs(struct s2226_stream *strm)
{
	int i;
	int rsize;
	if (strm == NULL)
		return -ENOMEM;
	rsize = (strm->type != S2226_STREAM_PREVIEW) ? S2226_MPEG_URB_SIZE : S2226_PREVIEW_URB_SIZE;
	if (strm->type == S2226_STREAM_DECODE) {
		/* allocate MPEG stream write URBS */
		for (i = 0; i < WR_URBS; i++) {
			strm->write_vid[i].urb = usb_alloc_urb(0, GFP_KERNEL);
			if (strm->write_vid[i].urb == NULL)
				return -ENOMEM;
			strm->write_vid[i].ready = 1;
			strm->write_vid[i].dev = strm->dev;
			strm->write_vid[i].strm = strm;
			strm->write_vid[i].buffer = kmalloc(MAX_USB_SIZE, GFP_KERNEL);
			if (strm->write_vid[i].buffer == NULL)
				return -ENOMEM;
		}
		return 0;
	}
	for (i = 0; i < RD_URBS; i++) {
		strm->read_vid[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		if (strm->read_vid[i].urb == NULL)
			return -ENOMEM;
		strm->read_vid[i].ready = 0;
		strm->read_vid[i].dev = strm->dev;
		strm->read_vid[i].strm = strm;
		strm->read_vid[i].buffer = kmalloc(rsize, GFP_KERNEL);
		if (strm->read_vid[i].buffer == NULL)
			return -ENOMEM;
	}
	return 0;
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct s2226_stream *strm = vb2_get_drv_priv(vq);
	struct s2226_dev *dev = strm->dev;
	dprintk(0, "%s: type: %d\n", __func__, strm->type);
	switch (strm->type) {
	case S2226_STREAM_MPEG:
		s2226_start_encode(dev, 0, S2226_CONTEXT_V4L);
		break;
	case S2226_STREAM_PREVIEW:
		s2226_start_preview(dev, 0, S2226_CONTEXT_V4L);
		break;
	default:
		return -EINVAL;
	}
	strm->active = 1;
	strm->framecount = 0;
	strm->m_pos = 0;
	return 0;
}

static int s2226_stop_urbs(struct s2226_stream *strm, int bwrite)
{
	int i;
	strm->active = 0;
	if (!bwrite) {
		for (i = 0; i < RD_URBS; i++) {
			usb_kill_urb(strm->read_vid[i].urb);
			strm->read_vid[i].ready = 0;
		}
	} else {
		for (i = 0; i < WR_URBS; i++) {
			usb_kill_urb(strm->write_vid[i].urb);
			strm->write_vid[i].ready = 0;
		}
	}
	return 0;
}

static int stop_streaming(struct vb2_queue *vq)
{
	struct s2226_stream *strm = vb2_get_drv_priv(vq);
	struct s2226_dev *dev = strm->dev;
	unsigned long flags = 0;
	struct s2226_buffer *buf, *node;
	dprintk(5, "%s\n", __func__);
	switch (strm->type) {
	case S2226_STREAM_MPEG:
		s2226_stop_encode(dev, 0);
		s2226_stop_urbs(strm, 0);
		break;
	case S2226_STREAM_PREVIEW:
		s2226_stop_preview(dev, 0);
		s2226_stop_urbs(strm, 0);
		break;
	}
	spin_lock_irqsave(&strm->qlock, flags);
	list_for_each_entry_safe(buf, node, &strm->buf_list, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
		dprintk(2, "[%p/%d] done\n",
			buf, buf->vb.v4l2_buf.index);
	}
	spin_unlock_irqrestore(&strm->qlock, flags);
	return 0;
}

static int s2226_new_v4l_input(struct s2226_dev *dev, int inp)
{
	int rc = 0;
	switch (inp) {
	case S2226_INPUT_COMPOSITE_0:
		rc = s2226_new_input(dev, dev->v4l_is_pal ?
				     INPUT_COMP0_576I : INPUT_COMP0_480I);
		break;
	case S2226_INPUT_SVIDEO_0:
		rc = s2226_new_input(dev, dev->v4l_is_pal ?
				     INPUT_SVIDEO0_576I : INPUT_SVIDEO0_480I);
		break;
	case S2226_INPUT_COMPOSITE_1:
		rc = s2226_new_input(dev, dev->v4l_is_pal ?
				     INPUT_COMP1_576I : INPUT_COMP1_480I);
		break;
	case S2226_INPUT_SVIDEO_1:
		rc = s2226_new_input(dev, dev->v4l_is_pal ?
				     INPUT_SVIDEO1_576I : INPUT_SVIDEO1_480I);
		break;
	case S2226_INPUT_SDI_SD:
		rc = s2226_new_input(dev, dev->v4l_is_pal ?
				     INPUT_SDI_576I : INPUT_SDI_480I);
		break;
	case S2226_INPUT_SD_COLORBARS:
		rc = s2226_new_input(dev, dev->v4l_is_pal ?
				     INPUT_SDI_576I_CB : INPUT_SDI_480I_CB);
		break;
	case S2226_INPUT_720P_COLORBARS:
		rc = s2226_new_input(dev, dev->v4l_is_pal ?
				     INPUT_SDI_720P_50_CB : INPUT_SDI_720P_60_CB);
		break;
	case S2226_INPUT_1080I_COLORBARS:
		rc = s2226_new_input(dev, dev->v4l_is_pal ?
				     INPUT_SDI_1080I_50_CB : INPUT_SDI_1080I_60_CB);
		break;
	case S2226_INPUT_SDI_720P_5994:
		rc = s2226_new_input(dev, INPUT_SDI_720P_5994);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_720P_60:
		rc = s2226_new_input(dev, INPUT_SDI_720P_60);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_1080I_50:
		rc = s2226_new_input(dev, INPUT_SDI_1080I_50);
		dev->v4l_is_pal = 1;
		break;
	case S2226_INPUT_SDI_1080I_5994:
		rc = s2226_new_input(dev, INPUT_SDI_1080I_5994);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_1080I_60:
		rc = s2226_new_input(dev, INPUT_SDI_1080I_60);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_720P_50:
		rc = s2226_new_input(dev, INPUT_SDI_720P_50);
		dev->v4l_is_pal = 1;
		break;
	case S2226_INPUT_SDI_720P_24:
		rc = s2226_new_input(dev, INPUT_SDI_720P_24);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_720P_2398:
		rc = s2226_new_input(dev, INPUT_SDI_720P_2398);
		dev->v4l_is_pal = 1;
		break;
	case S2226_INPUT_SDI_1080P_24:
		rc = s2226_new_input(dev, INPUT_SDI_1080P_24);
		dev->v4l_is_pal = 0;
		break;
	case S2226_INPUT_SDI_1080P_2398:
		rc = s2226_new_input(dev, INPUT_SDI_1080P_2398);
		dev->v4l_is_pal = 1;
		break;
	default:
		return -EINVAL;
	}
	if (rc != 0)
		return rc;
	dev->v4l_input = inp;
	return rc;
}



static int s2226_new_v4l_input_decode(struct s2226_dev *dev, int inp)
{
	int rc = 0;
	dprintk(2, "%s\n", __func__);
	switch (inp) {
	case S2226_DECODE_480I:
		rc = s2226_new_input(dev, INPUT_H51_SD_480I);
		break;
	case S2226_DECODE_576I:
		rc = s2226_new_input(dev, INPUT_H51_SD_576I);
		break;
	case S2226_DECODE_1080I_60:
		rc = s2226_new_input(dev, INPUT_H51_HD_1080I_60);
		break;
	case S2226_DECODE_1080I_5994:
		rc = s2226_new_input(dev, INPUT_H51_HD_1080I_5994);
		break;
	case S2226_DECODE_1080I_50:
		rc = s2226_new_input(dev, INPUT_H51_HD_1080I_50);
		break;
	case S2226_DECODE_1080P_24:
		rc = s2226_new_input(dev, INPUT_H51_HD_1080P_24);
		break;
	case S2226_DECODE_1080P_2398:
		rc = s2226_new_input(dev, INPUT_H51_HD_1080P_2398);
		break;
	case S2226_DECODE_720P_60:
		rc = s2226_new_input(dev, INPUT_H51_HD_720P_60);
		break;
	case S2226_DECODE_720P_5994:
		rc = s2226_new_input(dev, INPUT_H51_HD_720P_5994);
		break;
	case S2226_DECODE_720P_50:
		rc = s2226_new_input(dev, INPUT_H51_HD_720P_50);
		break;
	case S2226_DECODE_720P_24:
		rc = s2226_new_input(dev, INPUT_H51_HD_720P_24);
		break;
	case S2226_DECODE_720P_2398:
		rc = s2226_new_input(dev, INPUT_H51_HD_720P_2398);
		break;
	default:
		return -EINVAL;
	}
	if (rc != 0)
		return rc;
	dev->v4l_input = inp;
	return rc;
}

static int vidioc_g_std(struct file *file, void *priv, v4l2_std_id *i)
{
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;
	*i = dev->current_norm;
	return 0;
}


static int s2226_anystream_busy(struct s2226_dev *dev)
{
	int i;
	for (i = 0; i < S2226_MAX_STREAMS; i++)
		if (vb2_is_streaming(&dev->strm[i].vb_vidq))
			return 1;
	return 0;
}



static int vidioc_s_std(struct file *file, void *priv, v4l2_std_id _i)
{
	v4l2_std_id *i = &_i;
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;
	int ret = 0;

	if (strm->type == S2226_STREAM_DECODE)
		return 0;

	/* if standard not being changed, just return */
	if ((*i & V4L2_STD_NTSC) && (dev->current_norm == V4L2_STD_NTSC))
		return 0;
	else if ((*i & V4L2_STD_PAL) && (dev->current_norm == V4L2_STD_PAL))
		return 0;

	if (s2226_anystream_busy(dev)) {
		dprintk(1, "streaming on this on other stream. busy\n");
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
	} else if (*i & V4L2_STD_PAL) {
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
	} else
		return -EINVAL;

	dev->v4l_is_pal = (*i & V4L2_STD_PAL) ? 1 : 0;
	/* change input based on new standard */
	ret = s2226_new_v4l_input(dev, dev->v4l_input);
	dev_info(&dev->udev->dev, "%s %d\n", __func__, ret);
	return ret;
}

static int vidioc_enum_input_decode(struct file *file, void *priv,
			     struct v4l2_input *inp)
{
	if (inp->index >= S2226_DECODE_MAX)
		return -EINVAL;
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std = S2226_NORMS;
	inp->status = 0;
	switch (inp->index) {
	case S2226_DECODE_480I:
		strlcpy(inp->name, "Decode 480I(NTSC)", sizeof(inp->name));
		break;
	case S2226_DECODE_576I:
		strlcpy(inp->name, "Decode 576I(PAL)", sizeof(inp->name));
		break;
	case S2226_DECODE_1080I_60:
		strlcpy(inp->name, "Decode 1080I(60Hz)", sizeof(inp->name));
		break;
	case S2226_DECODE_1080I_5994:
		strlcpy(inp->name, "Decode 1080I(59.94Hz)", sizeof(inp->name));
		break;
	case S2226_DECODE_1080I_50:
		strlcpy(inp->name, "Decode 1080I(50Hz)", sizeof(inp->name));
		break;
	case S2226_DECODE_1080P_24:
		strlcpy(inp->name, "Decode 1080P(24Hz)", sizeof(inp->name));
		break;
	case S2226_DECODE_1080P_2398:
		strlcpy(inp->name, "Decode 1080P(23.98Hz)", sizeof(inp->name));
		break;
	case S2226_DECODE_720P_60:
		strlcpy(inp->name, "Decode 720P(60Hz)", sizeof(inp->name));
		break;
	case S2226_DECODE_720P_5994:
		strlcpy(inp->name, "Decode 720P(59.94Hz)", sizeof(inp->name));
		break;
	case S2226_DECODE_720P_50:
		strlcpy(inp->name, "Decode 720P(50Hz)", sizeof(inp->name));
		break;
	case S2226_DECODE_720P_24:
		strlcpy(inp->name, "Decode 720P(24Hz)", sizeof(inp->name));
		break;
	case S2226_DECODE_720P_2398:
		strlcpy(inp->name, "Decode 720P(23.98Hz)", sizeof(inp->name));
		break;
	}
	/*inp->status =  (status & 0x01) ? 0 : V4L2_IN_ST_NO_SIGNAL; */
	return 0;
}

static int vidioc_enum_input(struct file *file, void *priv,
			     struct v4l2_input *inp)
{
	struct s2226_stream *strm = video_drvdata(file);

	if (strm->type == S2226_STREAM_DECODE)
		return vidioc_enum_input_decode(file, priv, inp);
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
		strlcpy(inp->name, "SDI Input(720p 50Hz PAL)",
			sizeof(inp->name));
		inp->std = V4L2_STD_PAL;
		break;
	case S2226_INPUT_SDI_720P_5994:
		strlcpy(inp->name, "SDI Input(720p 59.94Hz NTSC)",
			sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_720P_60:
		strlcpy(inp->name, "SDI Input(720p 60Hz NTSC)",
			sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_1080I_50:
		strlcpy(inp->name, "SDI Input(1080i 50Hz PAL)",
			sizeof(inp->name));
		inp->std = V4L2_STD_PAL;
		break;
	case S2226_INPUT_SDI_1080I_5994:
		strlcpy(inp->name, "SDI Input(1080i 59.94Hz NTSC)",
			sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_1080I_60:
		strlcpy(inp->name, "SDI Input(1080i 60Hz NTSC)",
			sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_720P_24:
		strlcpy(inp->name, "SDI Input(720p 24Hz NTSC)",
			sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_720P_2398:
		strlcpy(inp->name, "SDI Input(720p 23.97Hz PAL)",
			sizeof(inp->name));
		inp->std = V4L2_STD_PAL;
		break;
	case S2226_INPUT_SDI_1080P_24:
		strlcpy(inp->name, "SDI Input(1080p 24Hz NTSC)",
			sizeof(inp->name));
		inp->std = V4L2_STD_NTSC;
		break;
	case S2226_INPUT_SDI_1080P_2398:
		strlcpy(inp->name, "SDI Input(1080p 23.97Hz PAL)",
			sizeof(inp->name));
		inp->std = V4L2_STD_PAL;
		break;
	}
	/*inp->status =  (status & 0x01) ? 0 : V4L2_IN_ST_NO_SIGNAL; */
	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;

	*i = dev->v4l_input;
	dprintk(4, "g_input\n");
	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;
	int rc;

	dprintk(2, "%s: %d type %d\n", __func__, i, strm->type);
	if (i >= S2226_INPUT_MAX) {
		dprintk(2, "EINVAL\n");
		return -EINVAL;
	}
	/* we don't set the input if any device streams are running */
	if (s2226_anystream_busy(dev)) {
		if (i == dev->v4l_input && !dev->is_decode)
			return 0;
		dprintk(2, "eBUSY m\n");
		return -EBUSY;
	}
	dprintk(2, "new v4l input %d\n", i);
	if (strm->type == S2226_STREAM_DECODE)
		rc = s2226_new_v4l_input_decode(dev, i);
	else
		rc = s2226_new_v4l_input(dev, i);

	dprintk(2, "%s return %d\n", __func__, rc);
	return rc;
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
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;
	memset(aud, 0, sizeof(struct v4l2_audio));
	switch (dev->cur_audio_mpeg) {
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
	aud->index = dev->cur_audio_mpeg;
	return 0;
}


static int vidioc_s_audio(struct file *file, void *priv,
			  const struct v4l2_audio *aud)
{
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;

	if (aud->index >= S2226_AUDIOINPUT_MAX)
		return -EINVAL;
	dev->cur_audio_mpeg = aud->index;
	(void) s2226_set_audiomux_mpegin(dev, dev->cur_audio_mpeg);
	return 0;
}

/* --- controls ---------------------------------------------- */
static void ctrl_fill(struct v4l2_queryctrl *ctrl, enum v4l2_ctrl_type type,
	const char *name, __s32 min, __s32 max, __s32 step, __s32 def, __u32 flags)
{
	ctrl->type = type;
	strlcpy(ctrl->name, name, sizeof(ctrl->name));
	ctrl->minimum = min;
	ctrl->maximum = max;
	ctrl->step = step;
	ctrl->default_value = def;
	ctrl->flags = flags;
	ctrl->reserved[0] = 0;
	ctrl->reserved[1] = 0;
}

static int vidioc_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *ctrl)
{
	struct s2226_stream *strm = video_drvdata(file);

	static const u32 user_ctrls_full[] = {
		V4L2_CID_USER_CLASS, V4L2_CID_BRIGHTNESS, V4L2_CID_CONTRAST, V4L2_CID_SATURATION, V4L2_CID_HUE,
		S2226_CID_AUDIOROUTE,
		S2226_CID_AUDMUX_MPEGIN,
		S2226_CID_AUDMUX_LINEOUT,
		S2226_CID_AUDMUX_SDIOUT,
		0
	};

	static const u32 user_ctrls[] = {
		V4L2_CID_USER_CLASS, V4L2_CID_BRIGHTNESS, V4L2_CID_CONTRAST, V4L2_CID_SATURATION, V4L2_CID_HUE,
		S2226_CID_AUDMUX_MPEGIN,
		S2226_CID_AUDMUX_LINEOUT,
		S2226_CID_AUDMUX_SDIOUT,
		0
	};

	static const u32 audioin_ctrls_full[] = {
		S2226_CID_AUDIN_CLASS,
		S2226_CID_AUDIN_AGC_ON_L,
		S2226_CID_AUDIN_AGC_ON_R,
		S2226_CID_AUDIN_AGC_GAIN_L,
		S2226_CID_AUDIN_AGC_GAIN_R,
		S2226_CID_AUDIN_BAL_L,
		S2226_CID_AUDIN_BAL_R,
		S2226_CID_AUDIN_GAIN_L,
		S2226_CID_AUDIN_GAIN_R,
		0
	};

	static const u32 audioin_ctrls[] = {
		S2226_CID_AUDIN_CLASS,
		S2226_CID_AUDIN_AGC_ON_L,
		S2226_CID_AUDIN_AGC_ON_R,
		S2226_CID_AUDIN_AGC_GAIN_L,
		S2226_CID_AUDIN_AGC_GAIN_R,
		S2226_CID_AUDIN_GAIN_L,
		S2226_CID_AUDIN_GAIN_R,
		0
	};

	static const u32 audioout_ctrls_full[] = {
		S2226_CID_AUDOUT_CLASS,
		S2226_CID_AUDOUT_DACVOL_L,
		S2226_CID_AUDOUT_DACVOL_R,
		S2226_CID_AUDOUT_DACMUTE_L,
		S2226_CID_AUDOUT_DACMUTE_R,
		S2226_CID_AUDOUT_MONO_GAIN,
		S2226_CID_AUDOUT_MONO_MUTE,
		S2226_CID_AUDOUT_HP_GAIN_L,
		S2226_CID_AUDOUT_HP_GAIN_R,
		S2226_CID_AUDOUT_HP_MUTE_L,
		S2226_CID_AUDOUT_HP_MUTE_R,
		S2226_CID_AUDOUT_STEREO_GAIN_L,
		S2226_CID_AUDOUT_STEREO_GAIN_R,
		S2226_CID_AUDOUT_STEREO_MUTE_L,
		S2226_CID_AUDOUT_STEREO_MUTE_R,
		0
	};

	static const u32 audioout_ctrls[] = {
		S2226_CID_AUDOUT_CLASS,
		S2226_CID_AUDOUT_MONO_GAIN,
		S2226_CID_AUDOUT_MONO_MUTE,
		S2226_CID_AUDOUT_HP_GAIN_L,
		S2226_CID_AUDOUT_HP_GAIN_R,
		S2226_CID_AUDOUT_HP_MUTE_L,
		S2226_CID_AUDOUT_HP_MUTE_R,
		S2226_CID_AUDOUT_STEREO_GAIN_L,
		S2226_CID_AUDOUT_STEREO_GAIN_R,
		S2226_CID_AUDOUT_STEREO_MUTE_L,
		S2226_CID_AUDOUT_STEREO_MUTE_R,
		0
	};

	static const u32 audiometer_ctrls[] = {
		S2226_CID_AUDMTR_CLASS,
		S2226_CID_AUDMTR_CHANNEL,
		S2226_CID_AUDMTR_LEVEL_L,
		S2226_CID_AUDMTR_LEVEL_R,
		S2226_CID_AUDMTR_LEVELDB_L,
		S2226_CID_AUDMTR_LEVELDB_R,
		S2226_CID_AUDMTR_HOLDREL,
		S2226_CID_AUDMTR_HOLDTIME,
		S2226_CID_AUDMTR_HOLD_L,
		S2226_CID_AUDMTR_HOLD_R,
		S2226_CID_AUDMTR_CLIP_L,
		S2226_CID_AUDMTR_CLIP_R,		
		S2226_CID_AUDMTR_TEST,
		0
	};

	/* must be in increasing order */
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
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
	};
#define DEF_BRIGHT 0x80
#define DEF_CONTRAST 0x40
#define DEF_SATURATION 0x40
#define DEF_HUE 0

	int old_id;

	ctrl_classes[0] = altboard ? user_ctrls_full : user_ctrls;
	switch (strm->type) {
	case S2226_STREAM_DECODE:
		ctrl_classes[1] = audioout_ctrls_full;
		ctrl_classes[2] = audiometer_ctrls;
		ctrl_classes[3] = NULL;
		break;
	case S2226_STREAM_MPEG:
		ctrl_classes[1] = mpeg_ctrls;
		ctrl_classes[2] = altboard ? audioin_ctrls_full : audioin_ctrls;
		ctrl_classes[3] = audioout_ctrls;
		ctrl_classes[4] = audiometer_ctrls;
		break;
	case S2226_STREAM_PREVIEW:
		ctrl_classes[1] = altboard ? audioin_ctrls_full : audioin_ctrls;
		ctrl_classes[2] = audioout_ctrls;
		ctrl_classes[3] = audiometer_ctrls;
		ctrl_classes[4] = NULL;
		break;
	default:
		return -EINVAL;
	}

	old_id = ctrl->id;
	ctrl->id = v4l2_ctrl_next(ctrl_classes, old_id);
	switch (ctrl->id) {
	case V4L2_CID_USER_CLASS:
	case V4L2_CID_MPEG_CLASS:
		v4l2_ctrl_query_fill(ctrl, 0, 0, 0, 0);
		break;
	case S2226_CID_AUDMTR_CLASS:
		strlcpy(ctrl->name, "Audio Meter", sizeof(ctrl->name));
		ctrl->type = V4L2_CTRL_TYPE_CTRL_CLASS;
		break;
	case S2226_CID_AUDOUT_CLASS:
		strlcpy(ctrl->name, "Audio Output", sizeof(ctrl->name));
		ctrl->type = V4L2_CTRL_TYPE_CTRL_CLASS;
		break;
	case S2226_CID_AUDIN_CLASS:
		strlcpy(ctrl->name, "Audio Input", sizeof(ctrl->name));
		ctrl->type = V4L2_CTRL_TYPE_CTRL_CLASS;
		break;
	case S2226_CID_AUDIN_BAL_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio In Balanced Inputs", 0, 1, 1, 0, 0);
		break;
	case S2226_CID_AUDIN_BAL_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio In Balanced Inputs", 0, 1, 1, 0, 0);
		break;
	case S2226_CID_AUDIN_AGC_ON_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio In AGC On Right", 0, 1, 1, 0, 0);
		break;
	case S2226_CID_AUDIN_AGC_ON_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio In AGC On Left", 0, 1, 1, 0, 0);
		break;
	case S2226_CID_AUDIN_GAIN_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio In Gain Right", 0, 118, 1, 0, 0);
		break;
	case S2226_CID_AUDIN_GAIN_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio In Gain Left", 0, 118, 1, 0, 0);
		break;
	case S2226_CID_AUDIN_AGC_GAIN_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio In AGC Gain Right", 0, 118, 1, 0, 0);
		break;
	case S2226_CID_AUDIN_AGC_GAIN_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio In AGC Gain Left", 0, 118, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_DACVOL_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio DAC Out Volume Left (0=max, 127=-63.5dB)", 0, 127, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_DACVOL_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio DAC Out Volume Right (0=max, 127=-63.5dB)", 0, 127, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_MONO_GAIN:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio Mono Out Additional Gain (0=none to 9=9dB)", 0, 9, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_HP_GAIN_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio HP Out Gain Left (0=none to 9=9dB)", 0, 9, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_HP_GAIN_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio HP Out Gain Right (0=none to 9=9dB)", 0, 9, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_STEREO_GAIN_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio Stereo Out Gain Left (0=none to 9=9dB)", 0, 9, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_STEREO_GAIN_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio Stereo Out Gain Right (0=none to 9=9dB)", 0, 9, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_DACMUTE_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio DAC Out Mute Right", 0, 1, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_DACMUTE_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio DAC Out Mute Left", 0, 1, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_MONO_MUTE:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio Mono Out Mute", 0, 1, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_HP_MUTE_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio HP Out Mute Left", 0, 1, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_HP_MUTE_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio HP Out Mute Right", 0, 1, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_STEREO_MUTE_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio Stereo Out Mute Left", 0, 1, 1, 0, 0);
		break;
	case S2226_CID_AUDOUT_STEREO_MUTE_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio Stereo Out Mute Right", 0, 1, 1, 0, 0);
		break;
	case S2226_CID_AUDIOROUTE:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_MENU, "Audio Route", 0, 3, 1, 0, 0);
		break;
	case S2226_CID_AUDMUX_MPEGIN:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_MENU, "Audio Mux MPEG In", 0, 2, 1,
			  AMUX_MPEG_IN_LINE_IN, 0);
		break;
	case S2226_CID_AUDMUX_LINEOUT:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_MENU, "Audio Mux Line Out", 0, 2, 1,
			  AMUX_LINE_OUT_MPEG_OUT, 0);
		break;
	case S2226_CID_AUDMUX_SDIOUT:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_MENU, "Audio Mux SDI Out", 0, 3, 1,
			  AMUX_SDI_OUT_MPEG_OUT, 0);
		break;
	case S2226_CID_AUDMTR_LEVEL_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio Meter Peak Right", 0, 0xffffff, 1, 0, 
			  V4L2_CTRL_FLAG_VOLATILE |  V4L2_CTRL_FLAG_READ_ONLY);
		break;
	case S2226_CID_AUDMTR_LEVELDB_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio Meter Peak Right dB(x10)", -2048, 0, 1, -2048, 
			  V4L2_CTRL_FLAG_VOLATILE |  V4L2_CTRL_FLAG_READ_ONLY);
		break;
	case S2226_CID_AUDMTR_LEVEL_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio Meter Peak Left", 0, 0xffffff, 1, 0, 
			  V4L2_CTRL_FLAG_VOLATILE |  V4L2_CTRL_FLAG_READ_ONLY);
		break;
	case S2226_CID_AUDMTR_LEVELDB_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio Meter Peak Left dB(x10)", -2048, 0, 1, -2048,
			  V4L2_CTRL_FLAG_VOLATILE |  V4L2_CTRL_FLAG_READ_ONLY);
		break;
	case S2226_CID_AUDMTR_HOLD_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio Meter Hold Left(dBx10)", -2048, 0, 1, -2048,
			  V4L2_CTRL_FLAG_VOLATILE |  V4L2_CTRL_FLAG_READ_ONLY);
		break;
	case S2226_CID_AUDMTR_HOLD_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_INTEGER, "Audio Meter Hold Right(dBx10)", -2048, 0, 1, -2048,
			  V4L2_CTRL_FLAG_VOLATILE |  V4L2_CTRL_FLAG_READ_ONLY);
		break;
	case S2226_CID_AUDMTR_CLIP_L:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio Meter Clipped? Left", 0, 1, 1, 0, V4L2_CTRL_FLAG_VOLATILE |  V4L2_CTRL_FLAG_READ_ONLY);
		break;
	case S2226_CID_AUDMTR_CLIP_R:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio Meter Clipped? Right", 0, 1, 1, 0, V4L2_CTRL_FLAG_VOLATILE |  V4L2_CTRL_FLAG_READ_ONLY);
		break;
	case S2226_CID_AUDMTR_CHANNEL:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_MENU, "Audio Meter Channel", 0, 2, 1, 0, 0);
		break;
	case S2226_CID_AUDMTR_TEST:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_MENU, "Audio Meter Test", 0, 3, 1, 0, 0);
		break;
	case S2226_CID_AUDMTR_HOLDTIME:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_MENU, "Audio Meter Hold Time", 0, 7, 1, 0, 0);
		break;
	case S2226_CID_AUDMTR_HOLDREL:
		ctrl_fill(ctrl, V4L2_CTRL_TYPE_BOOLEAN, "Audio Meter Hold Release", 0, 1, 1, 0, 0);
		break;
	case V4L2_CID_BRIGHTNESS:
		v4l2_ctrl_query_fill(ctrl, 0, 255, 1, DEF_BRIGHT);
		break;
	case V4L2_CID_CONTRAST:
		v4l2_ctrl_query_fill(ctrl, 0, 0x7f, 1, DEF_CONTRAST);
		break;
	case V4L2_CID_SATURATION:
		v4l2_ctrl_query_fill(ctrl, 0, 0x7f, 1, DEF_SATURATION);
		break;
	case V4L2_CID_HUE:
		v4l2_ctrl_query_fill(ctrl, -0x40, 0x40, 1, DEF_HUE);
		break;
	case V4L2_CID_MPEG_STREAM_TYPE:
		v4l2_ctrl_query_fill(ctrl,
				     V4L2_MPEG_STREAM_TYPE_MPEG2_TS,
				     V4L2_MPEG_STREAM_TYPE_MPEG2_TS, 1,
				     V4L2_MPEG_STREAM_TYPE_MPEG2_TS);
		break;
	case V4L2_CID_MPEG_AUDIO_SAMPLING_FREQ:
		v4l2_ctrl_query_fill(ctrl,
				     V4L2_MPEG_AUDIO_SAMPLING_FREQ_48000,
				     V4L2_MPEG_AUDIO_SAMPLING_FREQ_48000, 1,
				     V4L2_MPEG_AUDIO_SAMPLING_FREQ_48000);
		break;
	case V4L2_CID_MPEG_AUDIO_ENCODING:
		v4l2_ctrl_query_fill(ctrl,
				     V4L2_MPEG_AUDIO_ENCODING_LAYER_2,
				     V4L2_MPEG_AUDIO_ENCODING_LAYER_2, 1,
				     V4L2_MPEG_AUDIO_ENCODING_LAYER_2);
		break;
	case V4L2_CID_MPEG_AUDIO_L2_BITRATE:
		v4l2_ctrl_query_fill(ctrl,
				     V4L2_MPEG_AUDIO_L2_BITRATE_128K,
				     V4L2_MPEG_AUDIO_L2_BITRATE_256K,
				     1,
				     V4L2_MPEG_AUDIO_L2_BITRATE_256K);
		break;
	case V4L2_CID_MPEG_VIDEO_ENCODING:
		v4l2_ctrl_query_fill(ctrl,
				     V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC,
				     V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC, 1,
				     V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC);
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		v4l2_ctrl_query_fill(ctrl,
				     S2226_MIN_VBITRATE * 1000,
				     S2226_MAX_VBITRATE * 1000,
				     100000, /* increment by 100k */
				     S2226_DEF_VBITRATE * 1000);
		break;
	case V4L2_CID_MPEG_VIDEO_GOP_CLOSURE:
		v4l2_ctrl_query_fill(ctrl,
				     0,
				     1, 1,
				     0);
		break;
	default:
		dprintk(0, "%s: v4l2_ctrl_next returned %x\n", __func__, ctrl->id);
		ctrl->id = old_id;
		return -EINVAL;
	}
	return 0;
}


static int vidioc_querymenu(struct file *file, void *priv,
			    struct v4l2_querymenu *qmenu)
{
	static const char *route[] = {
		"Audio Line1",
		"Audio Line2",
		"Audio Line1 Bypass",
		"Audio Line2 Bypass",
	};
	static const char *audmux_mpegin[] = {
		"Line In",
		"Test Tone",
		"SDI In",
	};
	static const char *audmux_lineout[] = {
		"Line In",
		"Mpeg Out",
		"SDI In",
	};
	static const char *audmux_sdiout[] = {
		"Line In",
		"Mpeg Out",
		"Test Tone",
		"SDI In",
	};
	static const char *audmtr_channel[] = { "Line In", "Mpeg Out", "SDI In"};
	static const char *audmtr_test[] = { "Off", "Force to Zero",
					     "Force to clip", "Force to -6dB"};
	static const char *audmtr_holdtime[] = {
		"No Hold (update 1ms)",
		"Hold High 0.5s",
		"Hold High 1.0s",
		"Hold High 1.5s",
		"Hold High 2.0s",
		"Hold High 2.5s",
		"Hold High 3.0s",
		"Hold High Always"
	};
				

	switch (qmenu->id) {
	case V4L2_CID_MPEG_VIDEO_ENCODING:
		if (qmenu->index == V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC) {
			strlcpy(qmenu->name, "H.264(AVC)", sizeof(qmenu->name));
			return 0;
		} else if (qmenu->index < V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC)
			memset(qmenu->name, 0, sizeof(qmenu->name));
		break;
	case V4L2_CID_MPEG_AUDIO_ENCODING:
		/* only support AAC or None */
		if (qmenu->index == V4L2_MPEG_AUDIO_ENCODING_LAYER_2)
			strlcpy(qmenu->name, "MP2", sizeof(qmenu->name));
		else
			memset(qmenu->name, 0, sizeof(qmenu->name));
		return 0;
	case V4L2_CID_MPEG_VIDEO_ASPECT:
		if (qmenu->index == V4L2_MPEG_VIDEO_ASPECT_1x1) {
			strlcpy(qmenu->name, "None", sizeof(qmenu->name));
			return 0;
		}
		break;
	case S2226_CID_AUDIOROUTE:
		if (qmenu->index > 3)
			return -EINVAL;
		strlcpy(qmenu->name, route[qmenu->index], sizeof(qmenu->name));
		return 0;
	case S2226_CID_AUDMUX_MPEGIN:
		if (qmenu->index > 2)
			return -EINVAL;
		strlcpy(qmenu->name, audmux_mpegin[qmenu->index], sizeof(qmenu->name));
		return 0;
	case S2226_CID_AUDMUX_LINEOUT:
		if (qmenu->index > 2)
			return -EINVAL;
		strlcpy(qmenu->name, audmux_lineout[qmenu->index], sizeof(qmenu->name));
		return 0;
	case S2226_CID_AUDMUX_SDIOUT:
		if (qmenu->index > 3)
			return -EINVAL;
		strlcpy(qmenu->name, audmux_sdiout[qmenu->index], sizeof(qmenu->name));
		return 0;
	case S2226_CID_AUDMTR_CHANNEL:
		if (qmenu->index > 2)
			return -EINVAL;
		strlcpy(qmenu->name, audmtr_channel[qmenu->index], sizeof(qmenu->name));
		return 0;
	case S2226_CID_AUDMTR_HOLDTIME:
		if (qmenu->index > 7)
			return -EINVAL;
		strlcpy(qmenu->name, audmtr_holdtime[qmenu->index], sizeof(qmenu->name));
		return 0;
	case S2226_CID_AUDMTR_TEST:
		if (qmenu->index > 3)
			return -EINVAL;
		strlcpy(qmenu->name, audmtr_test[qmenu->index], sizeof(qmenu->name));
		return 0;
	case V4L2_CID_MPEG_AUDIO_MODE:
		if ((qmenu->index != V4L2_MPEG_AUDIO_MODE_STEREO) &&
		    (qmenu->index != V4L2_MPEG_AUDIO_MODE_MONO)) {
			memset(qmenu->name, 0, sizeof(qmenu->name));
			return 0;
		}
	}
	return v4l2_ctrl_query_menu(qmenu, NULL, NULL);
}

static int vidioc_try_ext_ctrls(struct file *file, void *priv,
				struct v4l2_ext_controls *ctrls)
{
	struct v4l2_ext_control *ctrl = ctrls->controls;
	int i;
	/* for invalid controls, clamp or set them to default value */
	for (i = 0; i < ctrls->count; i++, ctrl++) {
		switch (ctrl->id) {
		case V4L2_CID_BRIGHTNESS:
			if (ctrl->value < 0)
				ctrl->value = 0;
			else if (ctrl->value > 255)
				ctrl->value = 255;
			break;
		case V4L2_CID_CONTRAST:
		case V4L2_CID_SATURATION:
			if (ctrl->value < 0)
				ctrl->value = 0;
			else if (ctrl->value > 0x7f)
				ctrl->value = 0x7f;
			break;
		case V4L2_CID_HUE:
			if (ctrl->value < -0x40)
				ctrl->value = -0x40;
			else if (ctrl->value > 0x40)
				ctrl->value = 0x40;
			break;
		case S2226_CID_AUDIN_BAL_R:
		case S2226_CID_AUDIN_BAL_L:
		case S2226_CID_AUDIN_AGC_ON_R:
		case S2226_CID_AUDIN_AGC_ON_L:
		case S2226_CID_AUDOUT_DACMUTE_R:
		case S2226_CID_AUDOUT_DACMUTE_L:
		case S2226_CID_AUDOUT_MONO_MUTE:
		case S2226_CID_AUDOUT_HP_MUTE_R:
		case S2226_CID_AUDOUT_HP_MUTE_L:
		case S2226_CID_AUDOUT_STEREO_MUTE_R:
		case S2226_CID_AUDOUT_STEREO_MUTE_L:
		case S2226_CID_AUDMTR_HOLDREL:
			if (ctrl->value < 0)
				ctrl->value = 0;
			else if (ctrl->value > 1)
				ctrl->value = 1;
			break;
		case S2226_CID_AUDMTR_HOLDTIME:
			if (ctrl->value < 0)
				ctrl->value = 0;
			else if (ctrl->value > 7)
				ctrl->value = 7;
			break;
		case S2226_CID_AUDOUT_MONO_GAIN:
		case S2226_CID_AUDOUT_HP_GAIN_L:
		case S2226_CID_AUDOUT_HP_GAIN_R:
		case S2226_CID_AUDOUT_STEREO_GAIN_L:
		case S2226_CID_AUDOUT_STEREO_GAIN_R:
			if (ctrl->value > 9)
				ctrl->value = 9;
			else if (ctrl->value < 0)
				ctrl->value = 0;
			break;
		case S2226_CID_AUDOUT_DACVOL_R:
		case S2226_CID_AUDOUT_DACVOL_L:
			if (ctrl->value > 127)
				ctrl->value = 127;
			else if (ctrl->value < 0)
				ctrl->value = 0;
			break;
		case S2226_CID_AUDIN_GAIN_R:
		case S2226_CID_AUDIN_GAIN_L:
		case S2226_CID_AUDIN_AGC_GAIN_R:
		case S2226_CID_AUDIN_AGC_GAIN_L:
			if (ctrl->value < 0)
				ctrl->value = 0;
			else if (ctrl->value > 118)
				ctrl->value = 118;
			break;
		case S2226_CID_AUDIOROUTE:
			if (ctrl->value < 0)
				ctrl->value = 0;
			else if (ctrl->value > AUDIOROUTE_LINE2L_BYPASS)
				ctrl->value = AUDIOROUTE_LINE2L_BYPASS;
			break;
		case S2226_CID_AUDMUX_LINEOUT:
		case S2226_CID_AUDMUX_MPEGIN:
		case S2226_CID_AUDMTR_CHANNEL:
			if (ctrl->value < 0)
				ctrl->value = 0;
			else if (ctrl->value > 2)
				ctrl->value = 2;
			break;
		case S2226_CID_AUDMUX_SDIOUT:
		case S2226_CID_AUDMTR_TEST:
			if (ctrl->value < 0)
				ctrl->value = 0;
			else if (ctrl->value > 3)
				ctrl->value = 3;
			break;
		case V4L2_CID_AUDIO_BALANCE:
			if (ctrl->value < -100)
				ctrl->value = -100;
			else if (ctrl->value > 100)
				ctrl->value = 100;
			break;
		case V4L2_CID_MPEG_STREAM_TYPE:
			ctrl->value = V4L2_MPEG_STREAM_TYPE_MPEG2_TS;
			break;
		case V4L2_CID_MPEG_AUDIO_ENCODING:
			ctrl->value = V4L2_MPEG_AUDIO_ENCODING_LAYER_2;
			break;
		case V4L2_CID_MPEG_AUDIO_MODE:
			ctrl->value = V4L2_MPEG_AUDIO_MODE_STEREO;
			break;
		case V4L2_CID_MPEG_VIDEO_GOP_CLOSURE:
			if (ctrl->value != 0)
				ctrl->value = 1;
			break;
		case V4L2_CID_MPEG_VIDEO_BITRATE:
			if (ctrl->value < S2226_MIN_VBITRATE * 1000)
				ctrl->value = S2226_MIN_VBITRATE * 1000;
			else if (ctrl->value > S2226_MAX_VBITRATE * 1000)
				ctrl->value = S2226_MAX_VBITRATE * 1000;
			else
				ctrl->value -= ctrl->value % 100000;
			break;
#if 0
		case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
			if (ctrl->value != V4L2_MPEG_VIDEO_BITRATE_MODE_CBR)
				ctrl->value = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR;
			break;
#endif
		case V4L2_CID_MPEG_VIDEO_ENCODING:
			ctrl->value = V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC;
			break;
		default:
			ctrls->error_idx = i;
			return -EINVAL;
		}
	}
	return 0;
}

static int vidioc_g_ext_ctrls(struct file *file, void *priv,
			      struct v4l2_ext_controls *ctrls)
{
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;
	struct v4l2_ext_control *ctrl = ctrls->controls;
	int i;
	int rc;
	int reg;

	for (i = 0; i < ctrls->count; i++, ctrl++) {
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
		case S2226_CID_AUDIOROUTE:
			ctrl->value = dev->aud.iRoute;
			break;
		case S2226_CID_AUDIN_GAIN_L:
			ctrl->value = dev->aud.iLeftGain;
			break;
		case S2226_CID_AUDIN_GAIN_R:
			ctrl->value = dev->aud.iRightGain;
			break;
		case S2226_CID_AUDIN_AGC_GAIN_R:
			ctrl->value = dev->aud.iAGCRightGain;
			break;
		case S2226_CID_AUDIN_AGC_GAIN_L:
			ctrl->value = dev->aud.iAGCLeftGain;
			break;
		case S2226_CID_AUDIN_AGC_ON_L:
			ctrl->value = dev->aud.bAGC_L;
			break;
		case S2226_CID_AUDIN_AGC_ON_R:
			ctrl->value = dev->aud.bAGC_R;
			break;
		case S2226_CID_AUDIN_BAL_L:
			ctrl->value = dev->aud.in_balL;
			break;
		case S2226_CID_AUDIN_BAL_R:
			ctrl->value = dev->aud.in_balR;
			break;
		case S2226_CID_AUDOUT_DACVOL_R:
			ctrl->value = dev->aud.iVolGainR;
			break;
		case S2226_CID_AUDOUT_DACVOL_L:
			ctrl->value = dev->aud.iVolGainL;
			break;
		case S2226_CID_AUDOUT_DACMUTE_R:
			ctrl->value = dev->aud.bVolMuteR;
			break;
		case S2226_CID_AUDOUT_DACMUTE_L:
			ctrl->value = dev->aud.bVolMuteL;
			break;
		case S2226_CID_AUDOUT_MONO_MUTE:
			ctrl->value = dev->aud.bMonoMute;
			break;
		case S2226_CID_AUDOUT_HP_MUTE_R:
			ctrl->value = dev->aud.bHpMuteR;
			break;
		case S2226_CID_AUDOUT_HP_MUTE_L:
			ctrl->value = dev->aud.bHpMuteL;
			break;
		case S2226_CID_AUDOUT_STEREO_MUTE_R:
			ctrl->value = dev->aud.bStereoMuteR;
			break;
		case S2226_CID_AUDOUT_STEREO_MUTE_L:
			ctrl->value = dev->aud.bStereoMuteL;
			break;
		case S2226_CID_AUDOUT_MONO_GAIN:
			ctrl->value = dev->aud.iMonoGain;
			break;
		case S2226_CID_AUDOUT_HP_GAIN_R:
			ctrl->value = dev->aud.iHpGainR;
			break;
		case S2226_CID_AUDOUT_HP_GAIN_L:
			ctrl->value = dev->aud.iHpGainL;
			break;
		case S2226_CID_AUDOUT_STEREO_GAIN_R:
			ctrl->value = dev->aud.iStereoGainR;
			break;
		case S2226_CID_AUDOUT_STEREO_GAIN_L:
			ctrl->value = dev->aud.iStereoGainL;
			break;
		case S2226_CID_AUDMUX_MPEGIN:
			ctrl->value = dev->cur_audio_mpeg;
			break;
		case S2226_CID_AUDMUX_LINEOUT:
			ctrl->value = dev->cur_audio_lineout;
			break;
		case S2226_CID_AUDMUX_SDIOUT:
			ctrl->value = dev->cur_audio_sdiout;
			break;
		case S2226_CID_AUDMTR_LEVEL_R:
		{
			rc = s2226_get_audiomtr_level(dev, NULL, &reg);
			if (rc != 0)
				return rc;
			ctrl->value = reg;
		}
		break;
		case S2226_CID_AUDMTR_LEVEL_L:
		{
			rc = s2226_get_audiomtr_level(dev, &reg, NULL);
			if (rc != 0)
				return rc;
			ctrl->value = reg;
		}
		break;
		case S2226_CID_AUDMTR_LEVELDB_R:
		{
			rc = s2226_get_audiomtr_leveldb(dev, NULL, &reg);
			if (rc != 0)
				return rc;
			ctrl->value = -reg;
		}
		break;
		case S2226_CID_AUDMTR_LEVELDB_L:
		{
			rc = s2226_get_audiomtr_leveldb(dev, &reg, NULL);
			if (rc != 0)
				return rc;
			ctrl->value = -reg;
		}
		break;
		case S2226_CID_AUDMTR_CHANNEL:
		{
			rc = s2226_get_audiomtr_channel(dev, &reg);
			if (rc != 0)
				return rc;
			ctrl->value = reg;
		}
		break;
		case S2226_CID_AUDMTR_TEST:
		{
			rc = s2226_get_audiomtr_test(dev, &reg);
			if (rc != 0)
				return rc;
			ctrl->value = reg;
		}
		break;
		case S2226_CID_AUDMTR_HOLDREL:
		{
			rc = s2226_get_audiomtr_holdrelease(dev, &reg);
			if (rc != 0)
				return rc;
			ctrl->value = reg;
		}
		break;
		case S2226_CID_AUDMTR_HOLDTIME:
		{
			rc = s2226_get_audiomtr_holdtime(dev, &reg);
			if (rc != 0)
				return rc;
			ctrl->value = reg;
		}
		break;
		case S2226_CID_AUDMTR_HOLD_L:
		{
			rc = s2226_get_audiomtr_holdclip(dev, &reg, NULL, NULL, NULL);
			if (rc != 0)
				return rc;
			ctrl->value = reg;
		}
		break;
		case S2226_CID_AUDMTR_HOLD_R:
		{
			rc = s2226_get_audiomtr_holdclip(dev, NULL, &reg, NULL, NULL);
			if (rc != 0)
				return rc;
			ctrl->value = reg;
		}
		break;
		case S2226_CID_AUDMTR_CLIP_L:
		{
			rc = s2226_get_audiomtr_holdclip(dev, NULL, NULL, &reg, NULL);
			if (rc != 0)
				return rc;
			ctrl->value = reg;
		}
		break;
		case S2226_CID_AUDMTR_CLIP_R:
		{
			rc = s2226_get_audiomtr_holdclip(dev, NULL, NULL, NULL, &reg);
			if (rc != 0)
				return rc;
			ctrl->value = reg;
		}
		break;
		default:
			return -EINVAL;
		}
	}
	return 0;
}

static int vidioc_s_ext_ctrls(struct file *file, void *priv,
			      struct v4l2_ext_controls *ctrls)
{
	int i;
	struct s2226_stream *strm = video_drvdata(file);
	struct s2226_dev *dev = strm->dev;
	struct v4l2_ext_control *ctrl = ctrls->controls;
	int rc = 0;

	dprintk(2, "%s: %d %d\n", __func__, ctrl->id, V4L2_CID_MPEG_VIDEO_BITRATE);
	/* verify control values */
	rc = vidioc_try_ext_ctrls(file, priv, ctrls);
	if (rc < 0)
		return rc;
	for (i = 0; i < ctrls->count; i++, ctrl++) {
		switch (ctrl->id) {
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
		case S2226_CID_AUDIN_BAL_R:
			SetAudioBalR(dev, ctrl->value);
			break;
		case S2226_CID_AUDIN_BAL_L:
			SetAudioBalL(dev, ctrl->value);
			break;
		case S2226_CID_AUDIN_AGC_ON_R:
			SetAudioRightAGC(dev, ctrl->value, dev->aud.iAGCRightGain);
			break;
		case S2226_CID_AUDIN_AGC_ON_L:
			SetAudioLeftAGC(dev, ctrl->value, dev->aud.iAGCLeftGain);
			break;
		case S2226_CID_AUDIN_GAIN_R:
			SetAudioRightGain(dev, ctrl->value);
			break;
		case S2226_CID_AUDIN_GAIN_L:
			SetAudioLeftGain(dev, ctrl->value);
			break;
		case S2226_CID_AUDIN_AGC_GAIN_R:
			SetAudioRightAGC(dev, dev->aud.bAGC_R,
					 ctrl->value);
			break;
		case S2226_CID_AUDIN_AGC_GAIN_L:
			SetAudioLeftAGC(dev, dev->aud.bAGC_L,
					ctrl->value);
			break;
		case S2226_CID_AUDIOROUTE:
			SetAudioRoute(dev, ctrl->value);
			break;
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
				dev_info(&dev->udev->dev,
					 "inv audio rate using 256k\n");
				dev->h51_mode.aBitrate = 256;
				break;
			}
			break;
		case S2226_CID_AUDOUT_DACVOL_L:
			SetAudioVol(dev, dev->aud.iVolGainR,
				    ctrl->value, dev->aud.bVolMuteL,
				    dev->aud.bVolMuteR);
			break;
		case S2226_CID_AUDOUT_DACVOL_R:
			SetAudioVol(dev, ctrl->value,
				    dev->aud.iVolGainL, dev->aud.bVolMuteL,
				    dev->aud.bVolMuteR);
			break;
		case S2226_CID_AUDOUT_DACMUTE_L:
			SetAudioVol(dev, dev->aud.iVolGainR,
				    dev->aud.iVolGainL, ctrl->value,
				    dev->aud.bVolMuteR);
			break;
		case S2226_CID_AUDOUT_DACMUTE_R:
			SetAudioVol(dev, dev->aud.iVolGainR,
				    dev->aud.iVolGainL, dev->aud.bVolMuteL,
				    ctrl->value);
			break;
		case S2226_CID_AUDOUT_MONO_MUTE:
			SetAudioMono(dev, dev->aud.iMonoGain, ctrl->value);
			break;
		case S2226_CID_AUDOUT_MONO_GAIN:
			SetAudioMono(dev, ctrl->value, dev->aud.bMonoMute);
			break;
			
		case S2226_CID_AUDOUT_HP_MUTE_R:
			SetAudioHp(dev, dev->aud.iHpGainL,
				   dev->aud.iHpGainR, dev->aud.bHpMuteL,
				   ctrl->value);
			break;
		case S2226_CID_AUDOUT_HP_MUTE_L:
			SetAudioHp(dev, dev->aud.iHpGainL,
				   dev->aud.iHpGainR, ctrl->value,
				   dev->aud.bHpMuteR);
			break;
		case S2226_CID_AUDOUT_HP_GAIN_L:
			SetAudioHp(dev, ctrl->value,
				   dev->aud.iHpGainR,
				   dev->aud.bHpMuteL,
				   dev->aud.bHpMuteR);
			break;
		case S2226_CID_AUDOUT_HP_GAIN_R:
			SetAudioHp(dev, dev->aud.iHpGainL,
				   ctrl->value,
				   dev->aud.bHpMuteL,
				   dev->aud.bHpMuteR);
			break;
		case S2226_CID_AUDOUT_STEREO_GAIN_L:
			SetAudioStereo(dev,
				       ctrl->value,
				       dev->aud.iStereoGainR,
				       dev->aud.bStereoMuteL,
				       dev->aud.bStereoMuteR);
			break;
		case S2226_CID_AUDOUT_STEREO_GAIN_R:
			SetAudioStereo(dev,
				       dev->aud.iStereoGainL,
				       ctrl->value,
				       dev->aud.bStereoMuteL,
				       dev->aud.bStereoMuteR);
			break;
		case S2226_CID_AUDOUT_STEREO_MUTE_R:
			SetAudioStereo(dev,
				       dev->aud.iStereoGainL,
				       dev->aud.iStereoGainR,
				       dev->aud.bStereoMuteL,
				       ctrl->value);
			break;
		case S2226_CID_AUDOUT_STEREO_MUTE_L:
			SetAudioStereo(dev,
				       dev->aud.iStereoGainL,
				       dev->aud.iStereoGainR,
				       ctrl->value,
				       dev->aud.bStereoMuteR);
			break;
		case S2226_CID_AUDMUX_MPEGIN:
			dev->cur_audio_mpeg = ctrl->value;
			(void) s2226_set_audiomux_mpegin(dev, dev->cur_audio_mpeg);
			break;
		case S2226_CID_AUDMUX_LINEOUT:
			(void) s2226_set_audiomux_lineout(dev, ctrl->value);
			break;
		case S2226_CID_AUDMUX_SDIOUT:
			(void) s2226_set_audiomux_sdiout(dev, ctrl->value);
			break;
		case S2226_CID_AUDMTR_CHANNEL:
			return s2226_set_audiomtr_channel(dev, ctrl->value);
		case S2226_CID_AUDMTR_TEST:
			return s2226_set_audiomtr_test(dev, ctrl->value);
		case S2226_CID_AUDMTR_HOLDREL:
			return s2226_set_audiomtr_holdrelease(dev, ctrl->value);
		case S2226_CID_AUDMTR_HOLDTIME:
			return s2226_set_audiomtr_holdtime(dev, ctrl->value);
		default:
			return -EINVAL;
		}
	}
	return 0;
}


long s2226_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static const struct v4l2_file_operations s2226_fops_v4l = {
	.owner = THIS_MODULE,
	.open = s2226_open_v4l,
	.release = s2226_close,
	.unlocked_ioctl = s2226_ioctl,
	.write = s2226_write,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.read = vb2_fop_read,
};

static const struct v4l2_ioctl_ops s2226_ioctl_ops = {
	.vidioc_querycap = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = vidioc_s_fmt_vid_cap,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_g_std = vidioc_g_std,
	.vidioc_s_std = vidioc_s_std,
	.vidioc_enum_input = vidioc_enum_input,
	.vidioc_g_input = vidioc_g_input,
	.vidioc_s_input = vidioc_s_input,
	.vidioc_queryctrl = vidioc_queryctrl,
	.vidioc_querymenu = vidioc_querymenu,
	.vidioc_g_ext_ctrls = vidioc_g_ext_ctrls,
	.vidioc_s_ext_ctrls = vidioc_s_ext_ctrls,
	.vidioc_try_ext_ctrls = vidioc_try_ext_ctrls,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_enumaudio = vidioc_enumaudio,
	.vidioc_s_audio = vidioc_s_audio,
	.vidioc_g_audio = vidioc_g_audio,
};

static struct video_device template = {
	.name = "s2226v4l",
	.fops = &s2226_fops_v4l,
	.ioctl_ops = &s2226_ioctl_ops,
	.minor = -1,
	.release = video_device_release_empty,
	.tvnorms = S2226_NORMS,
};


static int s2226_probe_v4l(struct s2226_dev *dev)
{
	int ret;
	int cur_nr = video_nr;
	struct vb2_queue *q;
	int i;

	dev->v4l2_dev.release = s2226_release;
	ret = v4l2_device_register(&dev->interface->dev, &dev->v4l2_dev);
	if (ret) {
		printk(KERN_ERR "s2226 v4l2 device register fail\n");
		return ret;
	}
	for (i = 0; i < S2226_MAX_STREAMS; i++) {
		struct s2226_stream *strm = &dev->strm[i];
		spin_lock_init(&strm->qlock);
		mutex_init(&strm->vb_lock);
		INIT_LIST_HEAD(&strm->buf_list);
		strm->width = S2226_DEF_PREVIEW_X;
		strm->height = S2226_DEF_PREVIEW_Y;
		if (i == 1)
			strm->fourcc = V4L2_PIX_FMT_UYVY;
		else
			strm->fourcc = V4L2_PIX_FMT_MPEG;
		if (i == 0) {
			s2226_set_attr(dev, ATTR_SCALE_X, strm->width);
			s2226_set_attr(dev, ATTR_SCALE_Y, strm->height);
			s2226_set_attr(dev, ATTR_SCALE_SINGLE, 1);
			s2226_set_attr(dev, ATTR_MPEG_SCALER, 1);
		}
		q = &strm->vb_vidq;
		strm->queue = &strm->vb_vidq;
		q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		q->io_modes = VB2_MMAP | VB2_READ | VB2_USERPTR;
		q->drv_priv = strm;
		q->lock = &strm->vb_lock;
		q->buf_struct_size = sizeof(struct s2226_buffer);
		q->mem_ops = &vb2_vmalloc_memops;
		q->ops = &s2226_vb2_ops;
		q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
		ret = vb2_queue_init(q);
		if (ret != 0) {
			dev_err(&dev->udev->dev,
				"%s vb2_queue_init 0x%x\n", __func__, ret);
			return ret;
		}

		dprintk(5, "queue init %d\n", i);
		/* register MPEG video device */
		strm->vdev = template;
        switch (strm->type) {
        case S2226_STREAM_MPEG:
            strcpy(strm->vdev.name, "s2226v4l_h264");
            break;
        case S2226_STREAM_PREVIEW:
            strcpy(strm->vdev.name, "s2226v4l_preview");
            break;
        case S2226_STREAM_DECODE:
            strcpy(strm->vdev.name, "s2226v4l_decode");
            break;
        }

		/* for locking purposes */
		strm->vdev.queue = q;
		strm->vdev.lock = &dev->lock;
		strm->vdev.v4l2_dev = &dev->v4l2_dev;
		set_bit(V4L2_FL_USE_FH_PRIO, &strm->vdev.flags);
		video_set_drvdata(&strm->vdev, strm);
		if (video_nr == -1)
			ret = video_register_device(&strm->vdev,
						    VFL_TYPE_GRABBER,
						    video_nr);
		else
			ret = video_register_device(&strm->vdev,
						    VFL_TYPE_GRABBER,
						    cur_nr);
		if (ret != 0) {
			dev_err(&dev->udev->dev,
				"failed to register video device!\n");
			return ret;
		}

	}
	dev->current_norm = V4L2_STD_NTSC_M;
	dev_info(&dev->udev->dev, "Sensoray 2226 V4L driver\n");
	dev->cur_input = -1;
	dev->v4l_input = -1;
	dev->is_decode = 0;
	s2226_new_v4l_input(dev, S2226_INPUT_COMPOSITE_0);
	return 0;
}


module_init(usb_s2226_init);
module_exit(usb_s2226_exit);

MODULE_DESCRIPTION("Sensoray 2226 Linux driver (Copyright 2008-2014)");
MODULE_LICENSE("GPL");
MODULE_VERSION(S2226_VERSION);
