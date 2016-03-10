#include "range.h"
#include "thread_common_op.h"
#include <semaphore.h>

static int ks103_fd = -1;
static fd_set ks103_fd_set;
struct ks103_adapter adapter;

pthread_mutex_t ultra_msg_lock = PTHREAD_MUTEX_INITIALIZER;
sem_t ultra_get_sem;

Ultra_Data ultra_data;

int tell_external_ultra_is_available(void)
{
	return sem_post(&ultra_get_sem);		//if error return -1
}

int check_ultra_data_if_available(int _get_id)
{
	int mask = 0;
	pthread_mutex_lock(&ultra_msg_lock);
	mask = ultra_data.gotten;
	pthread_mutex_unlock(&ultra_msg_lock);
	
	// no update
	if( mask == 0 )
	{
		return -1;
	}
	// ultra was update
	if( mask & 0x01 )
	{
		// ultra hasn't been gotten
		if( (mask & _get_id)  == 0)
		{
			return 0;
		}
		else	// ultra has been gotten
		{
			return -1;
		}
			
	}
	return 0;
}

void set_ultra_data(float _data)
{
	pthread_mutex_lock(&ultra_msg_lock);
	ultra_data._ultra = _data;
	ultra_data.gotten = 0x01;
	pthread_mutex_unlock(&ultra_msg_lock);
}

float get_ultra_data(int _get_id)
{
	float ret;
	pthread_mutex_lock(&ultra_msg_lock);
	ret = ultra_data._ultra;
	ultra_data.gotten |= _get_id;
	pthread_mutex_unlock(&ultra_msg_lock);

	return ret;
}

int XY_Get_Ultra_Data(float *_data, int _get_id)
{
	if(check_ultra_data_if_available(_get_id) == -1)
	{
		return -1;
	}

	*_data = get_ultra_data(_get_id);
	return 0;
}

int KS103_Open(const char *port_str)
{
	ks103_fd = open(port_str, O_RDWR | O_NOCTTY);  //block mode
	if(ks103_fd < 0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
		return -1;
	}
	return 0;
}

int KS103_Close()
{
	close(ks103_fd);
	ks103_fd = -1;
	return 0;
}

int KS103_Flush()
{
	if(ks103_fd < 0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
	}
	else
	{
		tcflush(ks103_fd, TCIFLUSH);	//清除正收到的数据
	}
	return 0;
}

int KS103_Config(int baudrate, char data_bits, char parity_bits, char stop_bits)
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
	if (tcgetattr(ks103_fd, &oldtio) != 0)
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
	tcflush(ks103_fd, TCIFLUSH);

	/* activite the configuration */
	if((tcsetattr(ks103_fd, TCSANOW,&newtio))!=0)
	{
		printf("%s,%d:ERROR\n",__func__,__LINE__);
		return -1;
	}
	return 0;
}


static int KS103_Start(const char *dev_name, int baud_rate)
{
	const char *ptemp;
	if(dev_name == NULL)
	{
		ptemp = "/dev/ttyTHS2";	//UART3
	}
	else
	{
		ptemp = dev_name;
	}
	if(KS103_Open(ptemp) == 0
			&& KS103_Config(baud_rate,8,'N',1) == 0)
	{
		FD_ZERO(&ks103_fd_set);
		FD_SET(ks103_fd, &ks103_fd_set);
		return ks103_fd;
	}
	return -1;
	
}

static int send_to_ks103(unsigned char *buf,int len)
{
	return write(ks103_fd, buf, len);
}

int KS103_Launch_Detect(unsigned char *buf)
{
	return send_to_ks103(buf, 3);
}

void update_ks103_cmd(unsigned char *cmd)
{
	*cmd = adapter.cmd;
}

int adapter_ctl(ks103_adapter *adapter, unsigned int cmd, int arg)
{
	switch(cmd){
	case KS103_RETRIES:
		adapter->retries = arg;
		break;
	case KS103_TIMEOUT:
		adapter->timeout = arg;
		break;
	case KS103_SCMD:
		if(arg > 0xCC)
			return -1;
		adapter->cmd = (unsigned char)arg;
		break;	
	}
	return 0;
}
#if 0
/* 如果由其他线程调用此函数修改adapter.cmd的话, 需要在这里加入互斥锁 */
static int change_adapter_cmd(unsigned char cmd_value)
{
	return adapter_ctl(&adapter, KS103_SCMD, cmd_value);
}
#endif
void check_adapter_cmd(void)
{
	if(!adapter.cmd || adapter.cmd > 0xCC)
		adapter_ctl(&adapter, KS103_SCMD, KS103_DEFAULT_CMD);
}


int adapter_setup(void)
{
	if(adapter_ctl(&adapter, KS103_SCMD, KS103_DEFAULT_CMD) < 0)
		return -1;
	adapter_ctl(&adapter, KS103_RETRIES, 1);
	adapter_ctl(&adapter, KS103_TIMEOUT, 0);	

	return 0;
}

float KS103_Cal_Detect_Result(unsigned char *buf, unsigned char cmd)
{
	unsigned char cal_dependence;
	float cal_result = 0;

	if(buf == NULL)
		return (float)-1;
	cal_dependence = cmd;

	switch(cal_dependence){
	case RANGE_0_2_5M_RMM_NTC:
	case RANGE_0_2_5M_RMM_TC:
	case RANGE_0_2_11M_RMM_NTC:
	case RANGE_0_2_11M_RMM_TC:
	case RANGE_12CM_2_11M_RMM_NTC:
	case RANGE_12CM_2_11M_RMM_TC:
		cal_result = (*buf << 8 | *(buf+1))/1000.0;
		break;
	default:
		break;
	}
	return cal_result;
}


static void *KS103_Thread_Func(void * arg)
{
	int ret;
	unsigned int depth = 0;
	unsigned char wbuf[3];
	unsigned char rbuf[2];
	int i = 0;
	
	adapter_setup();
	
	wbuf[0] = KS103_SLAVE_ADDRESS;
	wbuf[1] = KS103_REGISTER2_ADDRESS;
	
	//change_adapter_cmd(RANGE_12CM_2_11M_RMM_TC);
	
	while(1)
	{
#if 0
		gettimeofday(&tpstart,NULL); 
#endif
		update_ks103_cmd(&wbuf[2]);
		if(!wbuf[2] || wbuf[2] > 0xCC)
		{
			wbuf[2] = KS103_DEFAULT_CMD;
			check_adapter_cmd();
		}
		
		KS103_Launch_Detect(wbuf);
		ret = select(FD_SETSIZE, &ks103_fd_set, (fd_set *)0, (fd_set *)0, (struct timeval *)0);
		if (ret < 1)
		{
			printf("%s,%d,ERROR\n", __func__, __LINE__);
			FD_ZERO(&ks103_fd_set);
			FD_SET(ks103_fd, &ks103_fd_set);
			continue;
		}
		do
		{
			if(i > adapter.retries)
				break;
			ioctl(ks103_fd, FIONREAD, &depth);
			i++;
		}while(depth != 2);
		
		if(i > adapter.retries)
		{
			i = 0;
			continue;
		}
		i = 0;
		ret = read(ks103_fd, rbuf, depth);
		
		set_ultra_data( KS103_Cal_Detect_Result(rbuf, wbuf[2]) );
		//printf("ultra is: %f.\n", get_ultra_data());
		//tell_external_ultra_is_available();
		
		//usleep(100000);
#if 0
		gettimeofday(&tpend,NULL); 
		timeuse = 1000000 * (tpend.tv_sec - tpstart.tv_sec) + tpend.tv_usec - tpstartcd.tv_usec; 
		timeuse /= 1000000; 
		cout << "Each range cost " << timeuse << endl;
#endif
	}
	pthread_exit(NULL);
}


int KS103_Start_Thread(void)
{
	int ret;
	pthread_t A_ARR;

	ret = sem_init(&ultra_get_sem, 0, 0);
	if(ret == -1)
	{
		printf("semaphore initialization failed.\n");
		return -1;
	}
	
	ret = pthread_create(&A_ARR, 0, KS103_Thread_Func, NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}


int XY_Ultra_Setup(const char *device,int baudrate)
{
	int ret;
	
	if(KS103_Start(device,baudrate) < 0)
	{
		KS103_Close();
		return -1;
	}

	ret = sem_init(&ultra_get_sem, 0, 0);
	if(ret == -1)
	{
		printf("semaphore initialization failed.\n");
		return -1;
	}
	
	if(XY_Create_Thread(KS103_Thread_Func, NULL, THREADS_ULTRA, -1, SCHED_RR, 94) < 0)
	{
		printf("Create KS103 Thread Error.\n");
		return -1;
	}
	
#if 0
	if(KS103_Start_Thread() < 0)
	{
		return -1;
	}
#endif


	return 0;
}




