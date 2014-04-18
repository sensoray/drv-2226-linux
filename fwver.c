// Copyright Sensoray 2011
// Firmware version query program

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <linux/videodev2.h>
#include "s2226ioctl.h"
#include "dh2226.h"


// returns 0 on success
int main(int argc, char **argv)
{
    int k;
    static FILE *fdev = NULL;
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
    fdev = fopen("/dev/video0", "wb");
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

