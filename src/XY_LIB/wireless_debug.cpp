#include "wireless_debug.h"
#include <stdarg.h>
#include "thread_common_op.h"

static int debug_fd = -1;
static fd_set debug_fd_set;

int debug_fresh_flag = 0;

struct debug_info debug_package = {0};

pthread_mutex_t debug_fresh_flag_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t debug_msg_lock = PTHREAD_MUTEX_INITIALIZER;


void XY_Update_Debug_Flag(int _offset)
{
	pthread_mutex_lock(&debug_fresh_flag_lock);
	debug_fresh_flag |= (1<<_offset);
	pthread_mutex_unlock(&debug_fresh_flag_lock);
}

int XY_Get_Debug_Flag(void)
{
	int ret;
	pthread_mutex_lock(&debug_fresh_flag_lock);
	ret = debug_fresh_flag;
	pthread_mutex_unlock(&debug_fresh_flag_lock);

	return ret;
}

void XY_Empty_Debug_Flag(int _offset)
{
	pthread_mutex_lock(&debug_fresh_flag_lock);
	debug_fresh_flag &= ~(1<<_offset);
	pthread_mutex_unlock(&debug_fresh_flag_lock);
}

int XY_Debug_Sprintf(int seq, char *fmt, ...)
{
	va_list args;
	int r;
	
	if( ((r = XY_Get_Debug_Flag()) & (1<<seq)) != 0  || seq == NULL)
	{
		seq = sdk_find_available_msg( strlen((const char *)fmt), r); 
		if(seq < 0)
			return 1;
	}
	
	va_start(args, fmt);	//使ap指向(fmt的基地址+fmt的长度)内存位置，即第二个参数位置
	pthread_mutex_lock(&debug_msg_lock);
	switch(seq)
	{
		case 0:
			memset(debug_package.msg_0, 0, strlen((const char *)debug_package.msg_0));
			vsprintf(debug_package.msg_0, fmt, args);
			break;
		case 1:
			memset(debug_package.msg_1, 0, strlen((const char *)debug_package.msg_1));
			vsprintf(debug_package.msg_1, fmt, args);
			break;
		case 2:
			memset(debug_package.msg_2, 0, strlen((const char *)debug_package.msg_2));
			vsprintf(debug_package.msg_2, fmt, args);
			break;
			break;
	}
	pthread_mutex_unlock(&debug_msg_lock);
	va_end(args);
	
	XY_Update_Debug_Flag(seq);

	return 0;
}

int XY_Debug_Send_At_Once(char *fmt, ...)
{
	char msg_at_once[512];
	
	va_list args;
	va_start(args, fmt);
	vsprintf(msg_at_once, fmt, args);
	va_end(args);
	sdk_send_debug_info(msg_at_once);
}


int XY_Debug_Setup(void)
{
	int wireless_baudrate = 9600;
	return sdk_wireless_debug_setup(DEBUG_DEV_NAME, wireless_baudrate);
}

int XY_Debug_Easy_Send(char *buf, int len)
{
	return write(debug_fd, buf, len);
}

int sdk_wireless_debug_setup(const char *device, int baudrate)
{
	if(uart_common_start(device, baudrate, &debug_fd, &debug_fd_set) < 0)
	{
		if(debug_fd > 0)
		{
			uart_common_close(&debug_fd);
		}
		return -1;
	}
	
	if(XY_Create_Thread(Debug_Check_And_Send_Thread_Func, NULL, THREADS_DEBUG, -1, SCHED_RR, 90) < 0)
	{
		printf("Create Debug And Send Thread Error.\n");
		return -1;
	}
	/*
	if(create_debug_check_and_send_thread() < 0)
	{
		return -1;
	}
	*/
	return 0;
}

int sdk_send_debug_info(char *str)
{
	return write(debug_fd, str, strlen((const char *)str));
}

int debug_msg_size[DECLARED_MSG] = {MSG_1_SIZE, MSG_2_SIZE, MSG_3_SIZE};
int sdk_find_available_msg(int need_len, int used_msg)
{
	int free_msg = -1;
	int i = 0;
	
	free_msg = (int)(pow(2, DECLARED_MSG)-1) - used_msg;

	if(free_msg == 0)
		return -1;

	while(i < DECLARED_MSG)
	{
		if(free_msg & 0x01)
		{
			if(need_len < debug_msg_size[i])
				return i;
		}
		free_msg = free_msg >> 1;	
		i++;
	}

	return -1;
}


static void *Debug_Check_And_Send_Thread_Func(void * arg)
{	
	//thread_binding_cpu(NULL, GENERAL_JOB_CPU);
	
	int _flag = 0;
	int i = 0;

	while(1)
	{	
		usleep(500000);
		if( (_flag = XY_Get_Debug_Flag()) == 0)
		{
			continue;
		}
		else
		{
			while(i < (sizeof(_flag) * 8) )
			{
				
				if(_flag & 0x01)
				{
					switch(i)
					{
						case 0:
							sdk_send_debug_info(debug_package.msg_0);
							break;
						case 1:
							sdk_send_debug_info(debug_package.msg_1);
							break;
					}
					XY_Empty_Debug_Flag(i);
				}
				_flag = _flag >> 1;	
				i++;
			}
			i = 0;
		}
		
	}
	pthread_exit(NULL);
}

int create_debug_check_and_send_thread(void)
{
	int ret;
	pthread_t A_ARR;
	ret = pthread_create(&A_ARR, 0, Debug_Check_And_Send_Thread_Func, NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}

