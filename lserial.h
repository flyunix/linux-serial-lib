#ifndef _LSERIAL_H_
#define _LSERIAL_H__

#include "type.h"
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

typedef enum {
    B0_E = 0,
    B50_E = 50,
    B75_E = 75,
    B110_E = 110,
    B134_E = 134,
    B150_E = 150,
    B200_E = 200,
    B300_E = 300,
    B600_E = 600,
    B1200_E = 1200,
    B1800_E = 1800,
    B2400_E = 2400,
    B4800_E = 4800,
    B9600_E = 9600,
    B19200_E = 19200,
    B38400_E = 38400,
    B57600_E = 57600,
    B115200_E = 115200,
    B230400_E = 230400,
    BAUDRATE_NUM
}baudrate_e;

typedef enum {
    BITS7_E = 7,    
    BITS8_E = 8,
    BITS_NUM
}bits_e;

typedef enum {
    NONE_E = 0,
    ODD_E  = 1,
    EVEN_E = 2,
    PARITY_NUM
}parity_e;

typedef enum {
    STOP1_E = 0,    
    STOP2_E = 1,
    STOP_NUM
}stop_e;

int32 serial_dev_open(char *dev_name, bool is_block);
void  serial_dev_close(int dev_fd);
int32 serial_attr_set(int32 dev_fd, baudrate_e baudrate, bits_e bits, parity_e parity, stop_e stop);
void  serial_write(int32 dev_fd, int8* buff, int32 data_len);
int32 serial_read(int fd, void *ptr, size_t nbytes);
int32 serial_selec_read(int32 dev_fd, int8* buff, int32 data_len, uint32 timeout);

#endif
