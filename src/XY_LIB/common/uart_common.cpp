#include "uart_common.h"



int uart_common_open(const char *port_str, int *fd)
{
	*fd = open(port_str, O_RDWR | O_NOCTTY);
	if(*fd < 0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
		return -1;
	}
	return 0;
}

int uart_common_close(int *fd)
{
	close(*fd);
	*fd = -1;
	return 0;
}


int uart_common_flush(int fd)
{
	if(fd < 0)
	{
		printf("%s, %d:ERROR\n", __func__, __LINE__);
		return -1;
	}
	else
	{
		tcflush(fd, TCIFLUSH);
	}

	return 0;
}

int uart_common_config(int fd, int baudrate, char data_bits, char parity_bits, char stop_bits)
{
	int st_baud[]=
	{
		B4800,
		B9600,
		B19200,
		B38400,
		B57600,
		B115200,
		B230400
	};
	int std_rate[]=
	{
		4800,
		9600,
		19200,
		38400,
		57600,
		115200,
		230400,
		1000000,
		1152000,
		3000000,
	};
	int i,j;
	struct termios newtio, oldtio;
	/* save current port parameter */
	if (tcgetattr(fd, &oldtio) != 0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
		return -1;
	}
	bzero(&newtio, sizeof(newtio));

	/* config the size of char */
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	/* config data bit */
	switch (data_bits)
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}
	/* config the parity bit */
	switch (parity_bits)
	{
		/* odd */
	case 'O':
	case 'o':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		break;
		/* even */
	case 'E':
	case 'e':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
		/* none */
	case 'N':
	case 'n':
		newtio.c_cflag &= ~PARENB;
		break;
	}

	/* config baudrate */
	j = sizeof(std_rate)/4;
	for(i = 0;i < j;i ++)
	{
		if(std_rate[i] == baudrate)
		{
			/* set standard baudrate */
			cfsetispeed(&newtio, st_baud[i]);
			cfsetospeed(&newtio, st_baud[i]);
			break;
		}
	}
	/* config stop bit */
    if( stop_bits == 1 )
       newtio.c_cflag &=  ~CSTOPB;
    else if ( stop_bits == 2 )
       newtio.c_cflag |=  CSTOPB;

    /* config waiting time & min number of char */
    newtio.c_cc[VTIME]  = 1;
    newtio.c_cc[VMIN] = 1;
	/* using the raw data mode */
	newtio.c_lflag	&= ~(ICANON | ECHO | ECHOE | ISIG);
	newtio.c_oflag	&= ~OPOST;

	/* flush the hardware fifo */
	tcflush(fd, TCIFLUSH);

	/* activite the configuration */
	if((tcsetattr(fd, TCSANOW,&newtio))!=0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
		return -1;
	}
	return 0;
}

int uart_common_start(const char *dev_name, int baud_rate, int *fd, fd_set *fdset)
{
	const char *ptemp;
	
	if(dev_name == NULL)
	{
		ptemp = DEFAULT_DEV_NAME;
	}
	else
	{
		ptemp = dev_name;
	}
	
	if(uart_common_open(ptemp, fd) == 0
			&& uart_common_config(*fd, baud_rate,8,'N',1) == 0)
	{
		FD_ZERO(fdset);
		FD_SET(*fd, fdset);
		return *fd;
	}
	return -1;
	
}

/* just used to test */
int uart_common_easy_send(int fd)
{
	char buf[40] = "If you see this, code is okay\n";
	
	return write(fd, buf, strlen((const char *)buf));
}


