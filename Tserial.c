#include "lserial.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

int main(int argc, char** argv)
{
    int dev_fd;  
    if(argc < 2) {
        printf("Usage: ./program size.\n");
        exit(1);
    }
    int nbytes = atoi(argv[1]);

#ifdef recv
    dev_fd = serial_dev_open("/dev/ttyUSB0", false);
    serial_attr_set(dev_fd, B115200_E, BITS8_E, NONE_E, STOP1_E);
    char buff[nbytes];
    memset(buff, 0, nbytes);
    //sleep(10);
    //printf("after sleep 10s.\n");
    //int rlen = serial_read(dev_fd, buff, nbytes, 0);
    int rlen = serial_read(dev_fd, buff, nbytes);
    printf("rlen = %d\n", rlen);
#endif

#ifdef send 
    dev_fd = serial_dev_open("/dev/ttyUSB1", true);
    serial_attr_set(dev_fd, B115200_E, BITS8_E, NONE_E, STOP1_E);
    char buff[nbytes];
    memset(buff, 0, nbytes);
    serial_write(dev_fd, buff, nbytes);
    printf("wlen = %d\n", nbytes);
#endif

    return 0;
}
