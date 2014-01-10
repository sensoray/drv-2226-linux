/*
 *  Based on free capture.c From V4L2 website
 *  Modified by Sensoray Company Copyright 2011
 *  for Motion Jpeg and MPEG transfer from 2250/2251 board
 *  as well as raw & compressed transfer from 2253 board
 *
*/

/* Ensure 64-bit offsets are used for creating large files */
#define _LARGEFILE64_SOURCE
#define _FILE_OFFSET_BITS 64
// for SYNC_FILE_RANGE_* defines
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>
#include <sys/soundcard.h>
#include <pthread.h>
#include <time.h>
#include <signal.h>
//#include "s2253ioctl.h"
//#include "s2253.h"
#ifdef USE_MP4
#include "mp4.h"	/* mp4 remuxer */
#endif

#define CLEAR(x) memset (&(x), 0, sizeof (x))

typedef enum {
	IO_METHOD_READ,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR,
} io_method;

struct buffer {
	void *		start;
	size_t		length;
};

static char *		dev_name	= NULL;
static io_method	io		= IO_METHOD_MMAP;
static int		fd		= -1;
struct buffer *		buffers		= NULL;
static unsigned int	n_buffers	= 0;

typedef enum {
	TYPE_JPEG,
	TYPE_MJPEG,
	TYPE_MPEG1,
	TYPE_MPEG2,
	TYPE_MPEG4,
	TYPE_H264,
	TYPE_YUYV,
	TYPE_UYVY,
	TYPE_Y8,
	TYPE_NV12,
	TYPE_MP42,
	TYPE_MPEGTS,
	TYPE_MPEGPS,
	TYPE_BGR24,
	TYPE_RGB565,
} comp_type;

static int type = TYPE_MPEGTS;
static int mux_type = TYPE_H264;
static FILE *G_fvout = NULL;
static int G_size = 0; // full size
static int G_width = 0, G_height = 0;
static char G_osd_text[80]; // aka caption aka overlay
static int G_osd_on = 0; // aka caption aka overlay
static char *default_name = "output.mpg";
static char *out_name = NULL;
static int G_aud_cap = 0;
static int G_br = -1; // video bitrate
static int G_abr=  -1; // audio bitrate
static struct v4l2_jpegcompression G_jc;	/* jpeg compression */
static int G_svideo = 0;
static int G_ainput = -1;
static int G_agc = -1;
static int G_ach = -1; // audio channels -1=default, 0=stereo, 3=mono
static int G_pal = 0;
static int G_framerate = 30;
static int G_field = V4L2_FIELD_ANY;
static int G_frames = 250;
static int G_idr = -1; // h.264 IDR frames (0=first GOP only, otherwise every Nth GOP)
static int G_gop = -1; // GOP size in frames
static int G_profile = -1; // h.264 profile
static int G_level = -1; // h.264 level
static int G_paused = 0; // stream pause by SIGUSR1
#ifdef USE_MP4
static muxer_handle remux = NULL;
#endif
//static struct s2253_user_data userdata;
static int G_decimate = 1; // framerate divider 1/2/3/5
static int G_cc = -1; // closed captions in h.264
static int G_dropped = 0; // dropped frames, reset every second

static void
errno_exit (const char *s)
{
	fprintf (stderr, "%s error %d, %s\n",
		s, errno, strerror (errno));

	exit (EXIT_FAILURE);
}

static int
xioctl (int fd, int request, void *arg)
{
	int r;

	do r = ioctl (fd, request, arg);
	while (-1 == r && EINTR == errno);

	return r;
}


#define SYNC_BUFFER_SIZE (8192 * 1024)
/* sync fd 8MB buffers to disk, and drop write caches */
/* see: http://stackoverflow.com/a/3756466/965672 */
static off_t sync_and_drop_write_cache(int fd, off_t pos)
{
#ifdef SYNC_FILE_RANGE_WRITE
	off_t tmp = lseek(fd, 0, SEEK_CUR) & (~(SYNC_BUFFER_SIZE - 1));
	if (tmp == pos)
		return pos;

	// sync pos, async
	if (sync_file_range(fd,
		pos,
		SYNC_BUFFER_SIZE,
		SYNC_FILE_RANGE_WRITE) < 0)
		perror("sync_file_range");
	if (pos == 0)
		return tmp;

	pos -= SYNC_BUFFER_SIZE;
	// wait for previous sync to finish
	if (sync_file_range(fd,
		pos,
		SYNC_BUFFER_SIZE,
		SYNC_FILE_RANGE_WAIT_BEFORE
		| SYNC_FILE_RANGE_WRITE
		| SYNC_FILE_RANGE_WAIT_AFTER) < 0)
		perror("sync_file_range");
	// drop cache pages
	if (posix_fadvise(fd,
		pos,
		SYNC_BUFFER_SIZE,
		POSIX_FADV_DONTNEED) < 0)
		perror("posix_fadvise");
	return tmp;
#else
	return 0;
#endif
}


static void
process_image (const void *p, int len, unsigned int seq)
{
	static int i = 0;
	static int j = 0;
	char fname[260];
	static unsigned int next_seq = 0;

	FILE *fout;
	if (type == TYPE_JPEG) {
		/* capture every 5 frames */
		if (j % 5 == 1) {
			sprintf(fname, "out_%d.jpg",i++);
            
			fout = fopen(fname, "w+");
			if (fout == NULL) {
				fprintf(stderr, "could not open file!\n");
				return;
			}
			fprintf(stderr, "saving JPEG snapshot %s\n", fname);
			fwrite(p, 1, len, fout);
			fclose(fout);
		}
		j++;
#ifdef USE_MP4
	} else if (remux) {
		mp4_buf_put(remux, (char*) p, len);
		mp4_remux(remux);
		if (mp4_get_error(remux)) {
			fprintf(stderr, "\nremux failed: %s\n", mp4_get_error(remux));
			exit(1);
		}
		mp4_buf_get(remux);
#endif
	} else {
		if (G_fvout) {
			fwrite(p, 1, len, G_fvout);
			if (G_fvout == stdout)
				fflush(G_fvout);
		}
	}
	if (G_fvout != stdout) {
		static off_t pos = 0;
		pos = sync_and_drop_write_cache(fileno(G_fvout), pos);
	}
	/* print . if sequence ok, ! if frames were lost */
	if (seq != next_seq) {
		int n = (seq - next_seq) / G_decimate;
		// print number of frames missed 1-9,A-Z
		//fputc(n + (n < 10 ? '0' : 'A'-10), stderr);
		G_dropped++;
	}
	if (seq % G_framerate > (seq + G_decimate) % G_framerate) {
		fputc(G_dropped == 0 ? '.' : 
			G_dropped + (G_dropped < 10 ? '0' : 'A'-10)
			, stderr);
		G_dropped = 0;
	}
	next_seq = seq + G_decimate;
	fflush (stderr);
}

static int
read_frame			(void)
{
	struct v4l2_buffer buf;
	unsigned int i;

	switch (io) {
	case IO_METHOD_READ:
	case IO_METHOD_USERPTR:
	default:
		fprintf(stderr, "not supported, use mmap!\n");
		break;
	case IO_METHOD_MMAP:
		CLEAR (buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;

		if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
			switch (errno) {
			case EAGAIN:
				return 0;

			case EIO:
				/* Could ignore EIO, see spec. */

				/* fall through */

			default:
				errno_exit ("VIDIOC_DQBUF");
			}
		}
		//xioctl (fd, S2253_VIDIOC_ECHO_TS, &buf.timestamp);
		
		assert (buf.index < n_buffers);

		process_image (buffers[buf.index].start,
			       buf.bytesused,
			       buf.sequence);

		if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
			errno_exit ("VIDIOC_QBUF");

		break;
	}

	return 1;
}

static void
mainloop                        (void)
{
	unsigned int count;


	count = G_frames;

	while (G_frames == 0 || count-- > 0) {
		for (;;) {
			fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO (&fds);
			FD_SET (fd, &fds);

			/* Timeout. */
			tv.tv_sec = 2;
			tv.tv_usec = 0;

			r = select (fd + 1, &fds, NULL, NULL, &tv);

			if (-1 == r) {
				if (EINTR == errno)
					continue;

				errno_exit ("select");
			}

			if (0 == r) {
				//char msg[256];
				if (G_paused)
					continue;
				fprintf (stderr, "select timeout\n");
				//ioctl (fd, S2253_VIDIOC_DEBUG, msg);
				//fprintf (stderr, "debug: %s\n", msg);
				exit (EXIT_FAILURE);
			}

			if (read_frame ())
				break;
	
			/* EAGAIN - continue select loop. */
		}
	}
}

static void
stop_capturing                  (void)
{
	enum v4l2_buf_type type;

	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;

	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fprintf(stderr, "stream off sent\n");
		if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
			errno_exit ("VIDIOC_STREAMOFF");

		break;
	}
}

static void
start_capturing                 (void)
{
	unsigned int i;
	enum v4l2_buf_type type;

	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;

	case IO_METHOD_MMAP:
		for (i = 0; i < n_buffers; ++i) {
			struct v4l2_buffer buf;

			CLEAR (buf);

			buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory      = V4L2_MEMORY_MMAP;
			buf.index       = i;

			if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
				errno_exit ("VIDIOC_QBUF");
		}
		
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
			errno_exit ("VIDIOC_STREAMON");
        printf("mmap done\n");
		break;

	case IO_METHOD_USERPTR:
		for (i = 0; i < n_buffers; ++i) {
			struct v4l2_buffer buf;

			CLEAR (buf);

			buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory      = V4L2_MEMORY_USERPTR;
			buf.index       = i;
			buf.m.userptr	= (unsigned long) buffers[i].start;
			buf.length      = buffers[i].length;

			if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
				errno_exit ("VIDIOC_QBUF");
		}

		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
			errno_exit ("VIDIOC_STREAMON");

		break;
	}
}

static void
uninit_device (void)
{
	unsigned int i;

	switch (io) {
	case IO_METHOD_READ:
		free (buffers[0].start);
		break;

	case IO_METHOD_MMAP:
		for (i = 0; i < n_buffers; ++i)
			if (-1 == munmap (buffers[i].start, buffers[i].length))
				errno_exit ("munmap");
		break;

	case IO_METHOD_USERPTR:
		for (i = 0; i < n_buffers; ++i)
			free (buffers[i].start);
		break;
	}

	free (buffers);
}

static void
init_read (unsigned int		buffer_size)
{
	buffers = calloc (1, sizeof (*buffers));

	if (!buffers) {
		fprintf (stderr, "Out of memory\n");
		exit (EXIT_FAILURE);
	}

	buffers[0].length = buffer_size;
	buffers[0].start = malloc (buffer_size);

	if (!buffers[0].start) {
		fprintf (stderr, "Out of memory\n");
		exit (EXIT_FAILURE);
	}
}

static void
init_mmap (void)
{
	struct v4l2_requestbuffers req;

	CLEAR (req);

	req.count               = 4;
	req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory              = V4L2_MEMORY_MMAP;

	if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf (stderr, "%s does not support "
				"memory mapping\n", dev_name);
			exit (EXIT_FAILURE);
		} else {
			errno_exit ("VIDIOC_REQBUFS");
		}
	}

	if (req.count < 2) {
		fprintf (stderr, "Insufficient buffer memory on %s\n",
			dev_name);
		exit (EXIT_FAILURE);
	}

	buffers = calloc (req.count, sizeof (*buffers));

	if (!buffers) {
		fprintf (stderr, "Out of memory\n");
		exit (EXIT_FAILURE);
	}

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;

		CLEAR (buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = n_buffers;

		if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf))
			errno_exit ("VIDIOC_QUERYBUF");
		
		//fprintf(stderr, "buf.length = %d\n", buf.length);

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start =
			mmap (NULL /* start anywhere */,
			      buf.length,
			      PROT_READ | PROT_WRITE /* required */,
			      MAP_SHARED /* recommended */,
			      fd, buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start)
			errno_exit ("mmap");
	}
}

static void
init_userp			(unsigned int		buffer_size)
{
	struct v4l2_requestbuffers req;
	unsigned int page_size;

	page_size = getpagesize ();
	buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

	CLEAR (req);

	req.count               = 4;
	req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory              = V4L2_MEMORY_USERPTR;

	if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf (stderr, "%s does not support "
				"user pointer i/o\n", dev_name);
			exit (EXIT_FAILURE);
		} else {
			errno_exit ("VIDIOC_REQBUFS");
		}
	}

	buffers = calloc (4, sizeof (*buffers));

	if (!buffers) {
		fprintf (stderr, "Out of memory\n");
		exit (EXIT_FAILURE);
	}

	for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
		buffers[n_buffers].length = buffer_size;
		buffers[n_buffers].start = memalign (/* boundary */ page_size,
						buffer_size);

		if (!buffers[n_buffers].start) {
			fprintf (stderr, "Out of memory\n");
			exit (EXIT_FAILURE);
		}
	}
}

static void add_ctrl(struct v4l2_ext_controls *ctrls, __u32 id, __s32 value)
{
	ctrls->controls[ctrls->count].id = id;
	ctrls->controls[ctrls->count].value = value;
	ctrls->count++;
}

static void
init_device                     (void)
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	unsigned int min;
	
	if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf (stderr, "%s is no V4L2 device\n",
				dev_name);
			exit (EXIT_FAILURE);
		} else {
			errno_exit ("VIDIOC_QUERYCAP");
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf (stderr, "%s is no video capture device\n",
			dev_name);
		exit (EXIT_FAILURE);
	}
	
	fprintf(stderr, "Card: %s\n", cap.card);
#if 0
	if (strncmp(cap.card, "Sensoray Model 2253", 19)) {
		fprintf (stderr, "%s card is not supported\n",
			dev_name);
		exit (EXIT_FAILURE);
	}
#endif
	

	/* set osd or not*/
#if 0
	if (G_osd_on) {
		struct s2253_clock clk;
		struct timeval tv;
		struct s2253_osd osd;
		memset(&osd, 0, sizeof(osd));
		osd.osdOn = G_osd_text[0] ? 1 : 0;
		osd.osdBmp = 1;
		osd.transparent = 1;
		osd.positionTop = 1;
		osd.fraction = 0;
		osd.xOffset = 20;
		osd.yOffset = 20;
		strncpy(osd.line, G_osd_text, sizeof(osd.line));

		if (-1 == ioctl (fd, S2253_VIDIOC_OSD, &osd)) {
			perror ("S2253_VIDIOC_OSD");
			exit (EXIT_FAILURE);
		}


		/* set the clock on the device to our local time */
		gettimeofday(&tv, NULL);  /* get UTC time */
		tzset();		  /* get timezone and daylight info */
		clk.sec = tv.tv_sec - timezone + daylight * (60*60);
		clk.usec = tv.tv_usec;
		if (-1 == ioctl (fd, S2253_VIDIOC_CLOCK, &clk)) {
			perror ("S2253_VIDIOC_CLOCK");
			exit (EXIT_FAILURE);
		}

	}

	if (userdata.len) {
		if (-1 == ioctl (fd, S2253_VIDIOC_SET_USER_DATA, &userdata)) {
			perror ("S2253_VIDIOC_SET_USER_DATA");
			exit (EXIT_FAILURE);
		}
	}
#endif

	switch (io) {
	case IO_METHOD_READ:
		if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
			fprintf (stderr, "%s does not support read i/o\n",
				 dev_name);
			exit (EXIT_FAILURE);
		}

		break;

	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
			fprintf (stderr, "%s does not support streaming i/o\n",
				 dev_name);
			exit (EXIT_FAILURE);
		}

		break;
	}




	/* Select video input, video standard and tune here. */
#if 0	// not used for 2226
	{
		int index = G_svideo ? 1 : 0;

		if (-1 == ioctl (fd, VIDIOC_S_INPUT, &index)) {
			perror ("VIDIOC_S_INPUT");
			exit (EXIT_FAILURE);
		}
	}
	
	{	/* change default NTSC standard to PAL */
		v4l2_std_id type = (G_pal) ? V4L2_STD_PAL : V4L2_STD_NTSC;

		if (-1 == xioctl (fd, VIDIOC_S_STD, &type)) {
			perror("VIDIOC_S_STD");
			//fprintf(stderr, "could not set std\n");
		}

		if (-1 == xioctl (fd, VIDIOC_G_STD, &type)) {
			perror("VIDIOC_G_STD");
			//fprintf(stderr, "could not get std\n");
		}

		if (type == V4L2_STD_NTSC) {
			fprintf(stderr, "Using NTSC standard\n");
		} else if (type == V4L2_STD_PAL) {
			fprintf(stderr, "Using PAL standard\n");
		}
	}
#endif

	{
		struct v4l2_input input;
		__u32 current;

		if (-1 == ioctl (fd, VIDIOC_G_INPUT, &current)) {
		        perror ("VIDIOC_G_INPUT");
		}

		memset (&input, 0, sizeof (input));
		input.index = current;

		if (-1 == ioctl (fd, VIDIOC_ENUMINPUT, &input)) {
		        perror ("VIDIOC_ENUMINPUT");
		} else  {
			fprintf (stderr, "Current input: %s\n", input.name);
#if 0
			if (input.status & V4L2_IN_ST_NO_H_LOCK)
				fprintf (stderr, "Waiting for video lock... (press ctrl-c to exit)\n");
			while (input.status & V4L2_IN_ST_NO_H_LOCK) {
				sleep(1);
				if (-1 == ioctl (fd, VIDIOC_ENUMINPUT, &input))
				        perror ("VIDIOC_ENUMINPUT");	
			}
#else
			if (input.status & V4L2_IN_ST_NO_H_LOCK)
				fprintf (stderr, "Warning: no video lock detected\n");
#endif
		}
	}

	
	CLEAR (cropcap);

        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect; /* reset to default */

                if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
                        switch (errno) {
                        case EINVAL:
                                /* Cropping not supported. */
                                break;
                        default:
                                /* Errors ignored. */
                                break;
                        }
                }
        } else {	
                /* Errors ignored. */
        }


        CLEAR (fmt);
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	switch (G_size) {
	case 0:
		fmt.fmt.pix.width = G_pal ? 704 : 640;
		fmt.fmt.pix.height = G_pal ? 576 : 480;
		break;
	case 1:
		fmt.fmt.pix.width = G_pal ? 352 : 320;
		fmt.fmt.pix.height = G_pal ? 288 : 240;
		break;
	case 2:
		fmt.fmt.pix.width = G_width;
		fmt.fmt.pix.height = G_height;
		break;		
	}
	switch (type) {
	case TYPE_JPEG:
	case TYPE_MJPEG:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
		break;
	case TYPE_MPEG1:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MPEG;
		break;
#if 0
	case TYPE_MPEG2:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MP2V;
		break;

	case TYPE_MPEG4:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MP4V;
		break;
#endif
	case TYPE_H264:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
		break;
	case TYPE_YUYV:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
		break;
	case TYPE_UYVY:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
		break;
	case TYPE_Y8:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
		break;
	case TYPE_NV12:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
		break;
	case TYPE_BGR24:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
		break;
	case TYPE_RGB565:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
		break;
	case TYPE_MP42:
		//fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MP42;
		break;
	case TYPE_MPEGTS:
	case TYPE_MPEGPS:
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MPEG;
		break;
	}
        fmt.fmt.pix.field       = G_field;
	//fmt.fmt.pix.field       = V4L2_FIELD_NONE;  // deinterlacing
	//fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED; // interlaced
	

        if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt))
                errno_exit ("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */

	/* Buggy driver paranoia. */
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;
	#if 0
	{
		struct v4l2_streamparm parm;
		
		CLEAR(parm);
		parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (0 == xioctl(fd, VIDIOC_G_PARM, &parm)) {
			parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			parm.parm.capture.timeperframe.numerator = (G_pal ? 1000 : 1001) * G_decimate;
			parm.parm.capture.timeperframe.denominator = G_pal ? 25000 : 30000;
			fprintf(stderr, "Time per frame: %u/%u\n",
				parm.parm.capture.timeperframe.numerator, 
				parm.parm.capture.timeperframe.denominator);
			if (0 != xioctl(fd, VIDIOC_S_PARM, &parm))
				errno_exit ("VIDIOC_S_PARM");
		}
	}
#endif
	if (0) {
		struct v4l2_queryctrl ctrl;
		int id;
		for (id = V4L2_CID_BASE; id < V4L2_CID_LASTP1; id++) {
			ctrl.id = id;
			if (ioctl(fd, VIDIOC_QUERYCTRL, &ctrl))
				continue;
			if (ctrl.flags & V4L2_CTRL_FLAG_DISABLED)
				continue;
			
			fprintf(stderr, "Queried control: %s\n", ctrl.name);
		}
		for (id = V4L2_CID_PRIVATE_BASE; ; id++) {
			ctrl.id = id;
			if (ioctl(fd, VIDIOC_QUERYCTRL, &ctrl))
				break;
			if (ctrl.flags & V4L2_CTRL_FLAG_DISABLED)
				continue;
			
			fprintf(stderr, "Queried control: %s\n", ctrl.name);
		}
		
		ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
		while (0 == ioctl (fd, VIDIOC_QUERYCTRL, &ctrl)) {
        		fprintf(stderr, "Queried control: %s\n", ctrl.name);
			ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
		}
	}
		
	/* optional MPEG parameters */
	if(type == TYPE_MPEG4 ||
	   type == TYPE_H264 ||
	   type == TYPE_MP42 ||
	   type == TYPE_MPEGTS ||
	   type == TYPE_MPEGPS ||
	   type == TYPE_MPEG2) {
		struct v4l2_ext_control ctrl[12] = {};
		struct v4l2_ext_controls ctrls = {
			.ctrl_class = V4L2_CTRL_CLASS_MPEG,
			.count = 0,
			.controls = &ctrl[0],
		};
		
		if (type == TYPE_MPEGTS)
			add_ctrl(&ctrls, V4L2_CID_MPEG_STREAM_TYPE, V4L2_MPEG_STREAM_TYPE_MPEG2_TS);
		else if (type == TYPE_MPEGPS) {
			add_ctrl(&ctrls, V4L2_CID_MPEG_STREAM_TYPE, V4L2_MPEG_STREAM_TYPE_MPEG2_PS);
//			add_ctrl(&ctrls, V4L2_CID_MPEG_AUDIO_ENCODING, V4L2_MPEG_AUDIO_ENCODING_PCM);
		} else if (type != TYPE_MP42)
			mux_type = type;
		
		if (mux_type == TYPE_MPEG1)
			add_ctrl(&ctrls, V4L2_CID_MPEG_VIDEO_ENCODING, V4L2_MPEG_VIDEO_ENCODING_MPEG_1);
		else if (mux_type == TYPE_MPEG2)
			add_ctrl(&ctrls, V4L2_CID_MPEG_VIDEO_ENCODING, V4L2_MPEG_VIDEO_ENCODING_MPEG_2);
		else if (mux_type == TYPE_H264)
			add_ctrl(&ctrls, V4L2_CID_MPEG_VIDEO_ENCODING, V4L2_MPEG_VIDEO_ENCODING_MPEG_4_AVC);
//		else if (mux_type == TYPE_MPEG4)
//			add_ctrl(&ctrls, V4L2_CID_MPEG_VIDEO_ENCODING, V4L2_MPEG_VIDEO_ENCODING_MPEG_4);

		if (G_br != -1)
			add_ctrl(&ctrls, V4L2_CID_MPEG_VIDEO_BITRATE, G_br);
		if (G_abr != -1)
			add_ctrl(&ctrls, V4L2_CID_MPEG_AUDIO_AAC_BITRATE, G_abr);
		if (G_ach != -1)
			add_ctrl(&ctrls, V4L2_CID_MPEG_AUDIO_MODE, G_ach);
		if (G_gop != -1)
			add_ctrl(&ctrls, V4L2_CID_MPEG_VIDEO_GOP_SIZE, G_gop);
		if (G_idr != -1)
			add_ctrl(&ctrls, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, G_idr);
#if defined(V4L2_CID_MPEG_VIDEO_H264_PROFILE) && defined(V4L2_CID_MPEG_VIDEO_H264_LEVEL)
		if (G_profile != -1)
			add_ctrl(&ctrls, V4L2_CID_MPEG_VIDEO_H264_PROFILE, G_profile);
		if (G_level != -1)
			add_ctrl(&ctrls, V4L2_CID_MPEG_VIDEO_H264_LEVEL, G_level);
#endif
#if defined(V4L2_CID_MPEG_STREAM_VBI_FMT)
		if (G_cc != -1) {
			add_ctrl(&ctrls, V4L2_CID_MPEG_STREAM_VBI_FMT, G_cc ? 
				V4L2_MPEG_STREAM_VBI_FMT_IVTV :
				V4L2_MPEG_STREAM_VBI_FMT_NONE);
		}
#endif

#if 0
		if (0 != xioctl (fd, VIDIOC_S_EXT_CTRLS, &ctrls)) {
			fprintf(stderr, "could not set extended mpeg controls\n"
				"  error_id: %u/%u, %x -> %d\n",
				ctrls.error_idx, ctrls.count,
				ctrls.controls[ctrls.error_idx].id, ctrls.controls[ctrls.error_idx].value);
		}
#endif

	} else if (type == TYPE_MJPEG || type == TYPE_JPEG) {
		if (-1 == ioctl (fd, VIDIOC_S_JPEGCOMP, &G_jc))
			perror ("VIDIOC_S_JPEGCOMP");
	}
	
#if 0
	{
		struct v4l2_audio audio;

		memset (&audio, 0, sizeof (audio));

		if (0) while (-1 != ioctl (fd, VIDIOC_ENUMAUDIO, &audio)) {
			fprintf (stderr, "Enumerated audio: %s\n", audio.name);
		        audio.index++;
		}
		if (-1 != G_ainput || -1 != G_agc) {
			memset (&audio, 0, sizeof (audio));
			
			if (-1 != G_ainput)
				audio.index = G_ainput;
			if (-1 != G_agc) {
				if (-1 == G_ainput)
					if (-1 == ioctl(fd, VIDIOC_G_AUDIO, &audio))
						perror("VIDIOC_G_AUDIO");
				audio.mode = G_agc ? V4L2_AUDMODE_AVL : 0;
			}
			
			if (-1 == ioctl (fd, VIDIOC_S_AUDIO, &audio))
		        	perror ("VIDIOC_S_AUDIO");
		}

		if (-1 == ioctl (fd, VIDIOC_G_AUDIO, &audio)) {
		        perror ("VIDIOC_G_AUDIO");
		} else
			fprintf (stderr, "Current audio: %s\n", audio.name);
	}
#endif

#if 0
	{
		struct v4l2_tuner tuner;

		memset (&tuner, 0, sizeof (tuner));
		tuner.index = 0;

		if (-1 == ioctl (fd, VIDIOC_G_TUNER, &tuner)) {
		        perror ("VIDIOC_G_TUNER");
		} else
			fprintf (stderr, "Current tuner: %s\n", tuner.name);
	}
#endif


	switch (io) {
	case IO_METHOD_READ:
		init_read (fmt.fmt.pix.sizeimage);
		break;

	case IO_METHOD_MMAP:
		init_mmap ();
		break;

	case IO_METHOD_USERPTR:
		init_userp (fmt.fmt.pix.sizeimage);
		break;
	}
}

static void
close_device                    (void)
{
	if (-1 == close (fd))
		errno_exit ("close");

	fd = -1;
}

static void
open_device                     (void)
{
	struct stat st;

	if (isdigit(dev_name[0]) && strlen(dev_name) <= 6) {
		// find dev node with serial number
		char *saved = dev_name;
		int i = 0;
		char tmp[32];
		char card[32];
		struct v4l2_capability cap;
		sprintf(card, "Sensoray Model 2253 SN#%s", dev_name);
		dev_name = tmp;
		for (i = 0; i < 100; i++) {
			sprintf(tmp, "/dev/video%d", i);
			fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
			if (fd == -1)
				continue; // failed to open
			if (0 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
				if (strcmp(cap.card, card) == 0)
					return; // found!
			}
			close(fd);
			fd = -1;
		}
		dev_name = saved;
	}

	if (-1 == stat (dev_name, &st)) {
		fprintf (stderr, "Cannot identify '%s': %d, %s\n",
			dev_name, errno, strerror (errno));
		exit (EXIT_FAILURE);
	}

	if (!S_ISCHR (st.st_mode)) {
		fprintf (stderr, "%s is no device\n", dev_name);
		exit (EXIT_FAILURE);
	}

	fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

	if (-1 == fd) {
		fprintf (stderr, "Cannot open '%s': %d, %s\n",
			dev_name, errno, strerror (errno));
		exit (EXIT_FAILURE);
	}
	return;
}

static void
quit_handler(int sig)
{
	exit(EXIT_SUCCESS);
}

static void
pause_handler(int sig)
{
#ifdef VIDIOC_ENCODER_CMD
	struct v4l2_encoder_cmd cmd;

	CLEAR(cmd);
	G_paused ^= 1;
	cmd.cmd = G_paused ? V4L2_ENC_CMD_PAUSE : V4L2_ENC_CMD_RESUME;
	if (ioctl(fd, VIDIOC_ENCODER_CMD, &cmd))
		perror("VIDIOC_ENCODER_CMD");
#endif
}

static void cleanup(void)
{
	stop_capturing ();

	uninit_device ();

	close_device ();

	fprintf(stderr, "demo closing files... ");
	fflush(stderr);
#ifdef USE_MP4
	if (remux) {
		if (type == TYPE_MP42)
			mp4_write_movie(remux);
		mp4_delete(remux);
		remux = NULL;
	}
#endif
	if (G_fvout)
		fclose(G_fvout);
	fprintf(stderr, "done.\n");

}

static void
usage                           (FILE *                 fp,
                                 int                    argc,
                                 char **                argv)
{
	fprintf (fp,
		"Usage: %s [options]\n\n"
		"Options:\n"
		"-d | --device name   Video device name [/dev/video0] or serial no.\n"
		"-h | --help          Print this message\n"
		"-c | --caption       Caption for OSD\n"
		"-j | --jpeg          JPEG separate files\n"
		"-J | --mjpeg         Motion JPEG single file\n"
		"-q | --qual          Jpeg Quality factor(10-90, 75 default)\n"
		"-4 | --mpeg4         MPEG4 elementary stream\n"
		"-x | --h264          h.264 elementary stream\n"
		"-y | --yuyv          YUYV raw video\n"
		"-u | --uyvy          UYVY raw video\n"
		"-8 | --y8            Y8 raw video\n"
		"-B | --bgr24         BGR24 (24-bpp) raw video\n"
		"-5 | --rgb565        RGB565 (16-bpp) raw video\n"
		"-n | --nv12          NV12 raw (YUV420 semi-planar)\n"
		"-m | --mp42          MP4 mux (with -4 or -x)\n"
		"-z | --mp4f          MP4 fragmented mux (with -4 or -x)\n"
		"-t | --ts            MPEG-TS mux (with -4 or -x)\n"
		"-T | --ps            MPEG-PS mux (with -2 -4 or -x)\n"
		"-s | --size          Size 0=vga, 1=1/2 VGA, or NxN\n"
		"-o | --out name      Output file name (MPEG only)\n"
		"-b | --br bitrate    Set video bitrate(bps, 2mbit default)\n"
		"-p | --pal           Set Standard to PAL\n"
		"-i | --int n         Interpolate field mode\n"
		"-f | --frames n      Capture n frames [default 250]\n"
		"                     (0=unlimited, use ctrl-c to stop)\n"
		"-D | --decimate n    Frame rate divider 1/2/3/5 (default 1)\n"
		"-G | --gop n         Set GOP size (0=default,\n"
		"                     otherwise every Nth frame)\n"
		"-I | --idr n         Set h.264 IDR frames (0=first GOP only,\n"
		"                     otherwise every Nth GOP)\n"
#if defined(V4L2_CID_MPEG_VIDEO_H264_PROFILE) && defined(V4L2_CID_MPEG_VIDEO_H264_LEVEL)
		"-P | --profile n     Set h.264 profile 0=baseline 2=main 4=high(default)\n"
		"-L | --level n       Set h.264 level 0=1.0 1=1B 2=1.1 3=1.2 4=1.3 5=2.0 6=2.1\n"
		"                7=2.2 8=3.0 9=3.1 10=3.2 11=4.0(default) 12=4.1 13=4.2 14=5.0\n"
#endif
		//"-v | --svideo        Use S-Video input\n"
		"-a | --ainput n      Set audio input: 0=Mic 1=Line\n"
		"-r | --abr bitrate   Set audio bitrate(bps, 192k default)\n"
		"-g | --agc n         Set audio AGC 0=off 1=on (default)\n"
		"-O | --mono          Set mono audio\n"
		"-S | --stereo        Set stereo audio\n"
		"-U | --userdata s    Set user data in compressed stream.\n"
		"-N | --interval n    Set user data interval (0=one time,\n"
		"                     otherwise every Nth frame)\n"
		"-C | --cc            Enable closed captions in h.264 stream\n"
#ifdef VIDIOC_ENCODER_CMD
		"To pause/resume, send SIGUSR1 to this process."
#endif
		"",
		argv[0]);
}



static const char short_options [] = "d:hMREjJ1248mztTnxyupvo:s:a:w:b:c:q:i:f:r:g:I:G:U:N:D:OSP:L:CB5";

static const struct option
long_options [] = {
	{ "device",	required_argument,	NULL,		'd' },
	{ "help",	no_argument,		NULL,		'h' },
	{ "mmap",	no_argument,		NULL,		'M' },
	{ "read",	no_argument,		NULL,		'R' },
	{ "userp",	no_argument,		NULL,		'E' },
	{ "jpeg",	no_argument,		NULL,		'j' },
	{ "mjpeg",	no_argument,		NULL,		'J' },
	{ "qual",	no_argument,		NULL,		'q' },
//	{ "mpeg1",	no_argument,		NULL,		'1' },
//	{ "mpeg2",	no_argument,		NULL,		'2' },
	{ "mpeg4",	no_argument,		NULL,		'4' },
	{ "y8",		no_argument,		NULL,		'8' },
	{ "mp42",	no_argument,		NULL,		'm' },
	{ "mp4f",	no_argument,		NULL,		'z' },
	{ "ts",		no_argument,		NULL,		't' },
	{ "ps",		no_argument,		NULL,		'T' },
	{ "nv12",	no_argument,		NULL,		'n' },
	{ "h264",	no_argument,		NULL,		'x' },
	{ "yuyv",	no_argument,		NULL,		'y' },
	{ "uyvy",	no_argument,		NULL,		'u' },
	{ "bgr24",	no_argument,		NULL,		'B' },
	{ "rgb565",	no_argument,		NULL,		'5' },
	{ "pal",	no_argument,		NULL,		'p' },
//        { "svideo",	no_argument,		NULL,		'v' },
	{ "size",	required_argument,	NULL,		's' },
	{ "caption",	required_argument,	NULL,		'c' },
	{ "out",	required_argument,	NULL,		'o' },
	{ "br",		required_argument,	NULL,		'b' },
	{ "int",	required_argument,	NULL,		'i' },
	{ "frames",	required_argument,	NULL,		'f' },
	{ "ainput",	required_argument,	NULL,		'a' },
	{ "abr",	required_argument,	NULL,		'r' },
	{ "agc",	required_argument,	NULL,		'g' },
	{ "idr",	required_argument,	NULL,		'I' },
	{ "gop",	required_argument,	NULL,		'G' },
	{ "userdata",	required_argument,	NULL,		'U' },
	{ "interval",	required_argument,	NULL,		'N' },
	{ "decimate",	required_argument,	NULL,		'D' },
	{ "mono",	no_argument,		NULL,		'O' },
	{ "stereo",	no_argument,		NULL,		'S' },
	{ "profile",	required_argument,	NULL,		'P' },
	{ "level",	required_argument,	NULL,		'L' },
	{ "cc",		no_argument,		NULL,		'C' },
	{ 0, 0, 0, 0 },
};



int
main                            (int                    argc,
                                 char **                argv)
{
	signal(SIGINT, quit_handler);
	signal(SIGTERM, quit_handler);
	signal(SIGUSR1, pause_handler);

	dev_name = "/dev/video0";
	G_osd_text[0] = 0;
	G_jc.quality = 75;
#if 0
	userdata.data = NULL;
	userdata.len = 0;
	userdata.interval = 0;
#endif
	for (;;) {
		int index;
		int c;
		
		c = getopt_long (argc, argv,
				short_options, long_options,
				&index);

		if (-1 == c)
			break;

		switch (c) {
		case 0: /* getopt_long() flag */
			break;

		case 'd':
			dev_name = optarg;
			break;
		case 'a':
			G_ainput = atoi(optarg);
			break;
		case 'i':
			G_field = atoi(optarg) ? V4L2_FIELD_NONE : V4L2_FIELD_INTERLACED;
			break;
		case 'h':
			usage (stdout, argc, argv);
			exit (EXIT_SUCCESS);
		case 'x':
			if (type == TYPE_MP42 || type == TYPE_MPEGTS || type == TYPE_MPEGPS) {
				mux_type = TYPE_H264;
			} else {
				type = TYPE_H264;
				default_name = "output.264";
			}
			break;
		case 'M':
			io = IO_METHOD_MMAP;
			break;
		case 'R':
			io = IO_METHOD_READ;
			break;
		case 'E':
			io = IO_METHOD_USERPTR;
			break;
		case 'j':
			type = TYPE_JPEG;
			default_name = "output.jpg";
			break;
		case 'J':
			type = TYPE_MJPEG;
			default_name = "output.mjpg";
			break;
		case '1':
			type = TYPE_MPEG1;
			break;
		case '2':
			if (type == TYPE_MP42 || type == TYPE_MPEGTS || type == TYPE_MPEGPS) {
				mux_type = TYPE_MPEG2;
			} else {
				type = TYPE_MPEG2;
				default_name = "output.m2v";
			}
			break;
		case '4':
			if (type == TYPE_MP42 || type == TYPE_MPEGTS || type == TYPE_MPEGPS) {
				mux_type = TYPE_MPEG4;
			} else {
				type = TYPE_MPEG4;
				default_name = "output.m4v";
			}
			break;
		case '8':
			type = TYPE_Y8;
			default_name = "output.y8";
			break;
		case 'y':
			type = TYPE_YUYV;
			default_name = "output.yuyv";
			break;
		case 'u':
			type = TYPE_UYVY;
			default_name = "output.uyvy";
			break;
		case 'n':
			type = TYPE_NV12;
			default_name = "output.nv12";
			break;
		case 'B':
			type = TYPE_BGR24;
			default_name = "output.bgr";
			break;
		case '5':
			type = TYPE_RGB565;
			default_name = "output.rgb565";
			break;
		case 'm':
#ifdef USE_MP4
			remux = mp4_remuxer_create();
#endif
			/* fall thru */
		case 'z':
			if (type == TYPE_H264 || type == TYPE_MPEG4 || type == TYPE_MPEG2)
				mux_type = type;
			type = TYPE_MP42;
			default_name = c == 'z' ? "output.mp4f" : "output.mp4";
			break;
		case 't':
			if (type == TYPE_H264 || type == TYPE_MPEG4 || type == TYPE_MPEG2)
				mux_type = type;
			type = TYPE_MPEGTS;
			default_name = "output.ts";
			break;
		case 'T':
			if (type == TYPE_H264 || type == TYPE_MPEG4 || type == TYPE_MPEG2)
				mux_type = type;
			type = TYPE_MPEGPS;
			default_name = "output.mpg";
			break;
		case 'o':
			out_name = optarg;
			break;
		case 's': {
			char *p;
			p = strchr(optarg, 'x');
			if (p) {
				*p = '\0';
				G_width = atoi(optarg);
				G_height = atoi(p + 1);
				G_size = 2;
			} else
				G_size = atoi(optarg) ? 1 : 0;
			break;
		}
		case 'c':
			strncpy(G_osd_text, optarg, 80);
			G_osd_on = 1;
			break;
		case 'b':
			G_br = atoi(optarg);
			break;
		case 'r':
			G_abr = atoi(optarg);
			break;
		case 'g':
			G_agc = atoi(optarg);
			break;
		case 'q':
			G_jc.quality = atoi(optarg);
			if (G_jc.quality < 10 || G_jc.quality > 90) {
				fprintf(stderr, "values from 10-90 only\n");
				exit(EXIT_FAILURE);
			}
			break;
		case 'p':
			G_pal = 1;
			G_framerate = 25;
			break;
		case 'v':
			G_svideo = 1;
			break;
		case 'f':
			G_frames = atoi(optarg);
			break;
		case 'I':
			G_idr = atoi(optarg);
			break;
		case 'P':
			G_profile = atoi(optarg);
			break;
		case 'L':
			G_level = atoi(optarg);
			break;
		case 'G':
			G_gop = atoi(optarg);
			break;
		case 'U':
			//userdata.data = optarg;
			//userdata.len = strlen(optarg);
			break;
		case 'N':
			//userdata.interval = atoi(optarg);
			break;
		case 'D':
			switch (atoi(optarg)) {
			case 1: G_decimate = 1; break;
			case 2: G_decimate = 2; break;
			case 3: G_decimate = 3; break;
			case 5: G_decimate = 5; break;
			default: 
				fprintf(stderr, "Invalid decimate parameter\n");
				exit(EXIT_FAILURE);
			}
			break;
		case 'O':
			G_ach = V4L2_MPEG_AUDIO_MODE_MONO;
			break;
		case 'S':
			G_ach = V4L2_MPEG_AUDIO_MODE_STEREO;
			break;
		case 'C':
			G_cc = 1;
			break;
			
		default:
			usage (stderr, argc, argv);
			exit (EXIT_FAILURE);
		}
	}
	out_name = out_name ? out_name : default_name;

	if (strcmp(out_name, "-") == 0)
		G_fvout = stdout;
	else
		G_fvout = fopen(out_name, "w+");

	if (G_fvout == NULL) {
		fprintf(stderr, "invalid filename %s, exiting\n", out_name);
		exit (EXIT_FAILURE);
	}
#ifdef USE_MP4
	if (remux) {
		if (G_fvout == stdout) {
			fprintf(stderr, "MP4 cannot be used with stdout\n");
			exit (EXIT_FAILURE);
		}
		mp4_file_pointer(remux, G_fvout);
	}
#endif
	if (type != TYPE_JPEG) {
		fprintf(stderr, "Saving video to %s\n", out_name);
	} else {
		fprintf(stderr, "Saving JPG clips in this directory\n");
		fprintf(stderr, "Type ls -al *.jpg after demo stops\n");
	}


	open_device ();

	init_device ();
	
	atexit(cleanup);
	
        start_capturing ();
	mainloop ();
	
	G_aud_cap = 0;

        return 0;
}

// kate: space-indent off; indent-width 8; mixedindent off; indent-mode cstyle;
