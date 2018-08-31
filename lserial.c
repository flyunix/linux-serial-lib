#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "lserial.h"

static int
Select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
       struct timeval *timeout)
{
	int		n;

again:
    if ( (n = select(nfds, readfds, writefds, exceptfds, timeout)) < 0) {
		if (errno == EINTR)
			goto again;
		else
			printf("select error.\n");
	} else if (n == 0 && timeout == NULL)
        printf("select returned 0 with no timeout\n");
	return(n);		/* can return 0 on timeout */
}

int32 
serial_dev_open(char *dev_name, bool is_block)
{
    int fd, flags;

    flags = O_RDWR | O_NOCTTY;
    if(!is_block) {
        flags |= O_NDELAY;
    }

    fd = open(dev_name, flags);

    if(fd == -1)  {			
        printf("open serial %s failed, err = %s\n", dev_name, strerror(errno));
        exit(-1);
    }

#if 0
    //恢复串口为阻塞状态
    if((flags = fcntl(fd, F_GETFL, 0)) < 0) {
        printf("fcntl F_GETFL, serial %s failed, err = %s\n", dev_name, strerror(errno));
        exit(-1);
    }

    flags &= ~O_NONBLOCK;
    if(fcntl(fd, F_SETFL, flags) < 0) {
        printf("fcntl F_SETFL, serial %s failed, err = %s\n", dev_name, strerror(errno));
        exit(-1);
    }
#endif
    if(0 == isatty(STDIN_FILENO)) {
        printf("standard input is not a terminal device.\n");
        exit(-1);
    }

    printf("open %s succ.\n", dev_name);

    return fd;
}

int32 
serial_attr_set(int32 dev_fd, baudrate_e baudrate, bits_e bits, parity_e parity, stop_e stop)
{
	struct termios termios_old, termios_new;

	if(tcgetattr(dev_fd, &termios_old) != 0)
	{
		perror("tcgetattr failed.");
		return -1;
	}
	
	bzero(&termios_new, sizeof(termios_new));

	termios_new.c_cflag |= CLOCAL;
	termios_new.c_cflag |= CREAD;

	termios_new.c_cflag &= ~CSIZE;
	switch(bits)
	{
		case BITS7_E:
			termios_new.c_cflag |= CS7;
			break;
		case BITS8_E:
			termios_new.c_cflag |= CS8;
			break;
		default:
			termios_new.c_cflag |= CS8;
			break;
	}

	switch(parity)
	{
		case NONE_E://no parity check
			termios_new.c_cflag &= ~PARENB;
			break;
		case ODD_E://odd check
			termios_new.c_cflag |= PARENB;
			termios_new.c_cflag |= PARODD;
			termios_new.c_cflag |= (INPCK | ISTRIP);
			break;
		case EVEN_E://even check
			termios_new.c_cflag |= (INPCK | ISTRIP);
			termios_new.c_cflag |= PARENB;
			termios_new.c_cflag &= ~PARODD;
			break;
		default://no parity check
			termios_new.c_cflag &= ~PARENB;
			break;    
	}
	
	switch(baudrate)
	{
		case 1200:
			cfsetispeed(&termios_new, B1200);
			cfsetospeed(&termios_new, B1200);
			break;
		case 2400:
			cfsetispeed(&termios_new, B2400);
			cfsetospeed(&termios_new, B2400);
			break;
		case 4800:
			cfsetispeed(&termios_new, B4800);
			cfsetospeed(&termios_new, B4800);
			break;	
		case 9600:
			cfsetispeed(&termios_new, B9600);
			cfsetospeed(&termios_new, B9600);
			break;
		case 19200:
			cfsetispeed(&termios_new, B19200);
			cfsetospeed(&termios_new, B19200);
			break;
		case 38400:
			cfsetispeed(&termios_new, B38400);
			cfsetospeed(&termios_new, B38400);
			break;
		case 57600:
			cfsetispeed(&termios_new, B57600);
			cfsetospeed(&termios_new, B57600);
			break;
		case 115200:
			cfsetispeed(&termios_new, B115200);
			cfsetospeed(&termios_new, B115200);
			break;
		case 230400:
			cfsetispeed(&termios_new, B230400);
			cfsetospeed(&termios_new, B230400);
			break;
		default:
			cfsetispeed(&termios_new, B9600);
			cfsetospeed(&termios_new, B9600);
			break;
	}
	
    switch(stop) {
        case STOP1_E:
            termios_new.c_cflag &= ~CSTOPB;
            break;
        case STOP2_E:
            termios_new.c_cflag &= ~CSTOPB;
            break;
        default:
            termios_new.c_cflag &= ~CSTOPB;
            break;
    }

    //如果不是开发终端之类的，只是串口传输数据，而不需要串口来处理，那么使用原始模式(Raw Mode)方式来通讯，设置方式如下 
    //Raw mode 
    termios_new.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
            | INLCR | IGNCR | ICRNL | IXON);
    termios_new.c_oflag &= ~OPOST;
    termios_new.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    termios_new.c_cflag &= ~(CSIZE | PARENB);
    termios_new.c_cflag |= CS8;

	termios_new.c_cc[VMIN]  = 0;//控制字符, 所要读取字符的最小数量
	termios_new.c_cc[VTIME] = 0;//控制字符, 读取第一个字符的等待时间 unit:(1/10)second
	tcflush(dev_fd, TCIFLUSH);//溢出的数据可以接收，但不读

	if(tcsetattr(dev_fd, TCSANOW, &termios_new) != 0)
	{
		perror("com set error\n");
		return -1;
	}
	return 0;
}


int32 
serial_select_read(int32 fd, int8* buff, int32 data_len, uint32 timeout)
{
    int read_len= 0; 
    fd_set fs_read;

    struct timeval tv_timeout;
    tv_timeout.tv_sec = timeout;
    tv_timeout.tv_usec = 0;

    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

    Select(fd + 1, &fs_read, NULL, NULL, &tv_timeout);

    if(FD_ISSET(fd, &fs_read))  {
        read_len = serial_read(fd, buff, data_len);
    }

    return read_len;
}

void  
serial_write(int32 dev_fd, int8* buff, int32 data_len)
{
    size_t nbytes = 0;
    signed int i = 0;
    while(nbytes < data_len) {
        i = write(dev_fd, buff + nbytes, data_len - nbytes);
        if(i < 0) {
            continue;
        }
        nbytes += i;
    }
}

int32
serial_read(int fd, void *ptr, size_t nbytes)
{
    ssize_t		n;

    if ( (n = read(fd, ptr, nbytes)) == -1)
        printf("read error, errno = %d, %s\n", errno, strerror(errno));
    return(n);
}

void 
serial_dev_close(int dev_fd)
{
    if(dev_fd > 0) {
        close(dev_fd);
    }
}
