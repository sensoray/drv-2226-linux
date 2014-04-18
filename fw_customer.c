// Copyright Sensoray 2011-2014
// Customer firmware update. Linux version


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
#include <pthread.h>
#include <time.h>
#include <signal.h>
#include "s2226ioctl.h"
#include "dh2226.h"

static FILE *fFW = NULL;
static int quit = 0;
static int G_arm, G_fpga;
static int fd;
static char *dev_name = NULL;

#define FNAME_ARM "f2226_228h.bin"
#define FNAME_FPGA "s2226_Ver.0.5.9.bin"

void control_c(int sig)
{
    printf("Please wait for firmware update to finish before exiting\n");
}


#define msleep(n) { struct timespec ts = { n/1000, (n*1000000)%1000000000 }; nanosleep(&ts, NULL); }

int update_fw(char *fwname, int addr, int len)
{
    int write_size = 128;
    int addr_start = 0;
    int len_start;
    size_t nr;   // number of bytes read
    int encsize; // encoded data inside packet
    int cursize; // current working size of frame
    int pktsize; // expected frame size (encoded data plus marker plus padding)
    struct flash_param p;
    int rv = 0;
    int arm_ver;
    unsigned char verify[512];

    p.addr = addr;
    p.len = len;
    addr_start = addr;
    len_start = p.len;
    fFW = fopen(fwname, "rb");

    if (fFW == NULL) {
        printf("could not open firmware file\n");
        return -EINVAL;
    }
    if (p.len = 510856) {
        p.len = 0x7e000;
    }
    if (ioctl(fd, S2226_VIDIOC_FLASH_ERASE, &p) != 0) {
	    printf("failed to erase flash\n");
	    return -1;
    }
    if (p.len == 0x7e000) {
        p.len = 510856;
    }

    if (ioctl(fd, S2226_VIDIOC_ARM_VER, &arm_ver) != 0) {
	    //printf("couldn't get arm version, assuming 0x33\n");
	    arm_ver = 0x33;
    }
    if (arm_ver >= 0x44) {
	    //printf("arm version: 0x%x. (supports fast update)\n", arm_ver);
	    write_size = 500;
    }

    while (1) {
	    nr = fread(p.data, 1, write_size, fFW);
	    if (nr <= 0) {
		    break;
	    }
	    p.len = nr;
	    p.addr = addr;

	    if (ioctl(fd, S2226_VIDIOC_FLASH_WR, &p) != 0) {
		    printf("failed write\n");
		    rv = -EAGAIN;
		    break;
	    }

	    // read back and verify
	    memcpy(verify, p.data, nr);
	    memset(p.data, 0xcd, write_size);
	    p.addr = addr;
	    if (arm_ver <= 0x33)
		    len = (nr >= 62) ? 62 : nr; // limit to read size on 0x33 and below
	    else
		    len = nr;

	    p.len = len;
	    if (ioctl(fd, S2226_VIDIOC_FLASH_RD, &p) != 0) {
		    printf("failed read\n");
		    rv = -EAGAIN;
		    break;
	    }
	    if (memcmp(verify, p.data, len) != 0) {
		    int j;
		    printf("verification error %d\n", len);
		    for (j = 0; j < 4; j++) {
			    printf("[%d]: %x %x\n", j, p.data[j], verify[j]);
		    }
		    rv = -EAGAIN;
		    break;
	    }
	    addr += nr;
	    printf(".");
	    fflush(NULL);
    }
    if (rv != 0) {
	    printf("\nfailed error code %d\n", rv);
	    if (addr_start == 0x1e0000) {
		    // if ARM fail, wipe out the overlay sector
		    // in case valid header exists and overlay
		    // tries to load corrupt firmware.
		    // TODO: this program will hold the device
		    // out of overlay if this happens.
		    p.addr = addr_start;
		    p.len = len_start;

		    if (ioctl(fd, S2226_VIDIOC_FLASH_ERASE, &p) != 0) {
			    printf("failed to erase flash\n");
			    return -1;
		    }
	    }
    }
    ioctl(fd, S2226_VIDIOC_FX2SAM_HI, NULL);

    if (fFW) {
        fclose(fFW);
        fFW = NULL;
    }
    return 0;
}

#define FPGA_REVA 33
#define FPGA_REVB 59
#define ARM_REVA 0x5c
#define ARM_REVB 0x228


static void errno_exit(const char *s)
{
	fprintf (stderr, "%s error %d, %s\n",
		s, errno, strerror (errno));

	exit (EXIT_FAILURE);
}

static int xioctl(int fd, int request, void *arg)
{
	int r;

	do r = ioctl (fd, request, arg);
	while (-1 == r && EINTR == errno);

	return r;
}


static void open_device(void)
{
	struct stat st;

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
    printf("opened\n");
	return;
}


static void init_device(void)
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
	if (strncmp(cap.card, "Sensoray Model 2226", 19)) {
		fprintf (stderr, "Sorry, \"%s\" is not a Sensoray 2226 device\n",
			dev_name);
		exit (EXIT_FAILURE);
	}
    return;
}


static void close_device(void)
{
	if (-1 == close (fd))
		errno_exit ("close");

	fd = -1;
}

// returns 0 on success
int main(int argc, char **argv)
{
    int k;
    int addr = 0;
    int len;
    int rv;
    int ch;
    int brevA = 1;
    int fail = 0;
    struct io_reg param;
    int rc;
    int bchange = 0;
    int bd;
    int arm_latest; 
    int fpga_latest;
    arm_latest = ARM_REVA;
    fpga_latest = FPGA_REVA;

    if (argc < 1) {
        printf("Usage: update_fw\n");
        return -1;
    }
    printf("Sensoray 2226 firmware updater March 19, 2014 (for RevA, RevB and Rev C boards)\n");
    dev_name = "/dev/video0";
    open_device();
    init_device();

    rc = ioctl(fd, S2226_VIDIOC_ARM_VER, &G_arm);
    if (rc >= 0) {
        printf("ARM version %x\n", G_arm);
    } else {
        printf("ARM version query failed %x\n", G_arm);
        return -1;
    }
    param.addr = (0x21 << 1);
    param.val = 0xff;
    rc = ioctl(fd, S2226_VIDIOC_FPGA_RD, &param);

    if (rc == 0) {
        bd = param.val >> 11;
        G_fpga = param.val & 0x7ff;

        if (G_fpga == 0x7ff) {
            (void) ioctl(fd, S2226_VIDIOC_SET_INPUT, 0);
            rc = ioctl(fd, S2226_VIDIOC_FPGA_RD, &param);
            if (rc != 0) {
                printf("error reading version\n");
                return -1;
            }
            bd = param.val >> 11;
            G_fpga = param.val & 0x7ff;            
        }
        printf("BOARD VERSION: %04x, fpga %d\n", bd, G_fpga);
        if (bd > 1) {
            //printf("Firmware updater not revised for these board revisions. Please use Windows or contact support\n");
            brevA = 0;
            arm_latest = ARM_REVB;
            fpga_latest = FPGA_REVB;
            printf("RevB or RevC board detected\n");
        } else {
            brevA = 1;
            printf("RevA board detected\n");
        }
    } else {
        printf("VERSION: error reading\n", param.addr);
        return 0;
    }

    if (G_arm >= arm_latest && G_fpga >= fpga_latest) {
        printf("ARM and FPGA firmware are already up to date!\n");
        close(fd);
        return 0;
    }

    printf("Uploading firmware.\n Do not cancel or turn off board during the update.\n");
    printf("Do you want to continue(Y/N)?");
    ch = getc(stdin);

    if (ch != 'Y' && ch != 'y') {
        return -1;
    }


    k=0;
    //fflush(fd);

    // send a NOP to test state of the ARM.  
    // in case a previous FW update attempt corrupted the overlay image
    // and that overlay image was attempted to load
    if (ioctl(fd, S2226_VIDIOC_NOP, NULL) != 0) {
	    printf("NOP failed, resetting H51 with OVERLAY signal(FX2SAM) deasserted\n");
	    ioctl(fd, S2226_VIDIOC_RESET_USB, NULL);
	    ioctl(fd, S2226_VIDIOC_FX2SAM_LO, NULL);
	    ioctl(fd, S2226_VIDIOC_RESET_BOARD, NULL);
	    // wait for ARM to start up
	    sleep(2);
	    ioctl(fd, S2226_VIDIOC_RESET_USB, NULL);
	    sleep(2);
	    ioctl(fd, S2226_VIDIOC_PRIME_FX2, NULL);
	    ioctl(fd, S2226_VIDIOC_DEFAULT_PARAMS, NULL);
    }

    len = 64*1024;
    printf("starting stage 1\n");
    signal(SIGINT, control_c);
    if (G_arm < arm_latest) {
        bchange = 1;
        addr = 0;
        printf("stage 1A\n");
        rv = update_fw(brevA ? "f2226_5Ch.bin" : FNAME_ARM, addr, len);
        if (rv != 0) {
            printf("failed to update CPU firmware!\n");
            fail = 1;
        }
        printf("stage 1B\n");
        addr = 0x01e0000;
        rv = update_fw(brevA ? "f2226_5Ch.bin" : FNAME_ARM, addr, len);
        if (rv != 0) {
            printf("failed to update CPU firmware!\n");
            fail = 1;
        }
    } else {
        printf("ARM firmware already up to date (or newer)\n");
    }
    addr = 0x010000;
    len = 510856;
    printf("\nstarting stage 2(please be patient, do not exit)\n");
    printf("Note: This may take up to 5 minutes\n");
    if (G_fpga < fpga_latest) {
        bchange = 1;
        rv = update_fw(brevA ? "s2226_Ver.0.3.3.bin" : FNAME_FPGA, addr, len);
        printf("\n");
        if (rv != 0) {
            printf("failed to update FPGA firmware!\n");
            fail = 1;
        }
        printf("\nstarting stage 3(please be patient, do not exit)\n");
        addr = brevA ? 0x90000 : 0x150000;
        len = 714000;
        rv = update_fw("mb86h51_mp2_dvb_20090525.bin", addr, len);

    } else {
        printf("FPGA firmware already up to date (or newer)\n");
    }

    if (!fail && bchange) {
        printf("Successfully updated firmware.  Please power cycle board to load new firmware\n");
    }

    close_device();
    return rv;
}

