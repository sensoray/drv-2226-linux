// Copyright Sensoray 2011
// Firmware version query program

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <linux/videodev2.h>
#include "s2226ioctl.h"
#include "dh2226.h"

static FILE *fdev = NULL;
char dev_name[100];
static void open_device(void)
{
    struct v4l2_capability cap;
    int i;
    int fails = 0;
    for (i = 0; i < 20; i++) {
        sprintf(dev_name, "/dev/video%d", i);
        printf("opening dev %s\n", dev_name);
        fdev = fopen(dev_name, "rb");
    	if (fdev == NULL) {
            fails++;
            if (fails < 4)
                continue;
            else {
                fprintf(stderr, "could not find device\n");
                return;
            }
        } 
        fails = 0;
        if (-1 == ioctl(fileno(fdev), VIDIOC_QUERYCAP, &cap)) {
            fprintf (stderr, "Cannot query name '%s': %d, %s\n",
                     dev_name, errno, strerror (errno));
            return;
        }
        printf("devname %s\n", cap.card);
        if (strncmp(cap.card, "2226", 4)) {
            fprintf (stderr, "not a 2226 device, searching other /dev/video devices\n");
            fclose(fdev);
            fdev = NULL;
            continue;
        } else
            break;
    }
    printf("opened\n");
	return;
}

// returns 0 on success
int main(int argc, char **argv)
{
    int k;

    int addr = 0;
    int len;
    int rv;
    int ch;
    int fail = 0;
    int ver;
    struct io_reg param;
    if (argc < 1) {
        printf("Usage: fwver. displays currently loaded firmware versions\n");
        return -1;
    }
    open_device();
    if (fdev == NULL) {
        printf("err: could not open s2226v0 device.\n");
        printf("if driver just loaded or board restarted, please wait and try again\n");
        return -ENODEV;
    }

    if (ioctl(fileno(fdev), S2226_VIDIOC_ARM_VER, &ver) != 0) {
	    printf("Could not get CPU firmware version\n");
    } else {
        printf("CPU version: 0x%02x\n", ver);
    }

    param.addr = (0x21 << 1);
    param.val = 0xff;
    rv = ioctl(fileno(fdev), S2226_VIDIOC_FPGA_RD, &param);
    if (rv == 0)
        printf("FPGA version: %d\n", (param.val & 0x7f));
    else
        printf("Error reading FPGA version %x\n", rv);
    fclose(fdev);
    return 0;
}

