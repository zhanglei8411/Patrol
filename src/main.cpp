//============================================================================
// Name        : main.cpp
// Author      : wuyuwei
// Version     :
// Copyright   : DJI Inc
// Description : DJI Onboard API test in C++, Ansi-style
//============================================================================

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <sys/sysinfo.h>
#include <sched.h>
#include <execinfo.h>
#include "DJI_Pro_Sample.h"
#include "XY_LIB/wireless_debug.h"
#include "XY_LIB/route.h"
#include "XY_LIB/common/common.h"
#include "XY_LIB/control_steer.h"
#include "XY_LIB/range.h"
#include "XY_LIB/image_identify.h"
#include "XY_LIB/thread_common_op.h"
#include "XY_LIB/status_led.h"
#include "XY_LIB/http_chat_3g.h"
#include "XY_LIB/sd_store_log.h"
 

#define DEBUG_PRINTF 1

using namespace std;
extern Aircraft_Status aircraft_status;

void handle_signal(int signum)
{
	switch(signum)
	{
		case SIGINT:
			printf("SIGINT\n");		
			//raise(SIGKILL);
			exit(0);
			break;
		default:
			break;
	}
}

void handle_crash(int signum)
{
	void *array[30] = {0};
	size_t size;  
	char **strings = NULL;  
	size_t i;
	
	size = backtrace(array, 30);
	printf("Obtained %zd stack frames.\n", size);  
	strings = backtrace_symbols(array, size);  
	if (NULL == strings)  
	{  
		perror("backtrace_synbols");  
		exit(EXIT_FAILURE);  
	}  
	
	for (i = 0; i < size; i++)  
		printf("%s\n", strings[i]);  

	free(strings);  
	strings = NULL; 
	exit(0);
}



/* this function should be called by main first */
int setup_main_process_scheduler(int _policy, int _priority)
{
	struct sched_param param;
	int maxpri;
	
	maxpri = sched_get_priority_max(_policy);

	_priority = _priority < maxpri ? _priority : maxpri;
	
	param.sched_priority = _priority;		
	
	if(sched_setscheduler(getpid(), _policy, &param) == -1) //设置优先级
	{ 
		perror("sched_setscheduler() failed"); 
		return 1;
	} 

	return 0;
}

int process_binding_cpu(int cpu_seq)
{
	int cpu_core_num = get_nprocs();
	printf("CPU has %d core\n", cpu_core_num);
	cpu_set_t mask; 
 	cpu_set_t get; 

	cpu_seq = cpu_seq >= (cpu_core_num - 1) ? (cpu_core_num - 1) : cpu_seq;

	CPU_ZERO(&mask); 
	CPU_SET(cpu_seq, &mask); 
	if (sched_setaffinity(0, sizeof(mask), &mask) == -1) { 
		printf("warning: could not set CPU affinity, continuing\n"); 
		return 1;
	} 
	
	CPU_ZERO(&get); 
	if (sched_getaffinity(0, sizeof(get), &get) == -1) { 
		printf("warning: cound not get cpu affinity\n"); 
		return 1;
	} 
	for (int i = 0; i < cpu_core_num; i++) { 
		if (CPU_ISSET(i, &get)) { 
			printf("this process %d is running processor : %d\n",getpid(), i); 
		} 
	} 

	return 0;
	
}


int main(int argc,char **argv)
{
	activate_data_t user_act_data; 
	char temp_buf[65];
	char app_bundle_id[32] = "1234567890";
	int i = 0;
	int wait_time = 0;

	printf("\nXunyi Drone Test,Ver 1.1.0\n\n");

	setup_main_process_scheduler(SCHED_RR, 99);		//如无另行设置，后面创建的线程将继承这里的属性(调度策略+优先级)
	//process_binding_cpu(GENERAL_JOB_CPU);							
	
	signal(SIGINT, handle_signal);
	signal(SIGSEGV, handle_crash);
	
	if(XY_Debug_Setup() < 0)
	{
		printf("Debug Function Open ERROR...\n");
	}
	printf("Debug Function Open SUCCESS...\n");
	XY_Debug_Send_At_Once("Debug Function Open SUCCESS.\n");

  	if(XY_Status_Led_Setup() < 0)
	{
		printf("Led Function Open ERROR...\n");
		XY_Debug_Send_At_Once("Led Function Open ERROR.\n");
	}
	printf("Led Function Open SUCCESS...\n");
	XY_Debug_Send_At_Once("Led Function Open SUCCESS.\n");

	ioctl_led(2);

	if(XY_Capture_Setup() < 0)
	{
		printf("Capture function Open ERROR...\n");
		XY_Debug_Send_At_Once("Capture function Open ERROR.\n");
	}
	printf("Capture function Open SUCCESS...\n");
	XY_Debug_Send_At_Once("Capture function Open SUCCESS.\n");

#if 0
	printf("start capture\n");
	XY_Start_Capture();
	sleep(5);
	printf("stop capture\n");
	XY_Stop_Capture();
	sleep(5);
	printf("start capture\n");
	XY_Start_Capture();
	sleep(5);
	while(1)
	{
		sleep(5);
	}
#endif

#if 1
_relink:
	if(XY_Http_Chat_Setup() < 0)
	{
		printf("Http Chat Function Open ERROR...\n");
		XY_Debug_Send_At_Once("Http Chat Function Open ERROR.\n");
	}
	printf("Http Chat Function Open SUCCESS...\n");
	XY_Debug_Send_At_Once("Http Chat Function Open SUCCESS.\n");
	
	pthread_join(get_tid(THREADS_WAIT_ORDER), NULL );

	if( get_order_status() == 0)
	{
		goto _relink;
	}

	if( XY_Http_Reported_Setup() < 0)
	{
		printf("Http Reported Function Open ERROR...\n");
		XY_Debug_Send_At_Once("Http Reported Function Open ERROR.\n");
	}
	printf("Http Reported Function Open SUCCESS...\n");
	XY_Debug_Send_At_Once("Http Reported Function Open SUCCESS.\n");

#if 0
	/* just to test */
	//装载成功
	XY_Send_Http_Post_Request_Data(0, "msg_id=1&order_id=%s&load=1\r\n", get_order_id_from_json() );
	//标记定位中
	XY_Send_Http_Post_Request_Data(1, "msg_id=3&order_id=%s&landing=1\r\n", get_order_id_from_json() );
	//定位完成
	XY_Send_Http_Post_Request_Data(2, "msg_id=3&order_id=%s&landing=2\r\n", get_order_id_from_json() );
	//已送达, 准备返航
	XY_Send_Http_Post_Request_Data(3, "msg_id=4&order_id=%s&arrived=0\r\n", get_order_id_from_json() );
#endif

#if 0
	message_server_load_is_okay();
	sleep(70);		
	message_server_finding_mark();
	sleep(30);
#endif

#endif
	
	ioctl_led(2);
	if(argc == 2)
	{
		wait_time = atoi(argv[1]);

		wait_time = wait_time<5 ? 5 : wait_time;
		wait_time = wait_time>90 ? 90 : wait_time;
			
		printf("Please wait %ds\n", wait_time);
		XY_Debug_Send_At_Once("Please wait %ds\n", wait_time);
		
		while(i < wait_time)
		{
			sleep(1);
			i++;
			if(i % 2 != 0)
				continue;
			
			printf("Last: %ds\n", wait_time-i);
			XY_Debug_Send_At_Once("Last: %ds\n", wait_time-i);
		}
	}
	
	//ioctl_led(3);

	
#if 0
	XY_Start_Capture();
	while(1)
	{
		sleep(5);
	}
#endif
	
	if(DJI_Sample_Setup() < 0)
	{
		printf("Serial Port Open ERROR...\n");
		XY_Debug_Send_At_Once("Serial Port Open ERROR.\n");
		return 0;
	}
	printf("Serial Port Open SUCCESS...\n");
	XY_Debug_Send_At_Once("Serial Port Open SUCCESS.\n");	

	user_act_data.app_key = temp_buf;
	user_act_data.app_ver = SDK_VERSION;
	strcpy((char*)user_act_data.app_bundle_id, app_bundle_id);
    if(DJI_Pro_Get_Cfg(NULL,NULL,&user_act_data.app_id,&user_act_data.app_api_level,user_act_data.app_key) == 0)
	{
		/* user setting */
		printf("--------------------------\n");
		printf("app id=%d\n",user_act_data.app_id);
		printf("app api level=%d\n",user_act_data.app_api_level);
		printf("app key=%s\n",user_act_data.app_key);
		printf("--------------------------\n");
	}
	else
	{
		printf("ERROR:There is no user account\n");	
		return 0;
	}
	
	if(XY_Ultra_Setup("/dev/ttyTHS2", 9600) < 0)
	{
		printf("Ultra Function Open ERROR.\n");
		XY_Debug_Send_At_Once("Ultra Function Open ERROR.\n");
	}

	if(0 == XY_Load_Goods())
	{
		printf("Load signal send okay.\n");
		XY_Debug_Send_At_Once("Load signal send okay.\n");
		message_server_load_is_okay();
	}
	else
	{
		printf("Load signal send error.\n");
		XY_Debug_Send_At_Once("Load signal send error.\n");
	}
	
	sleep(5);
	
	init_aircraft_status();
	
	while(1)
	{
		DJI_Pro_Activate_API(&user_act_data,NULL);
		while(aircraft_status.activation == -1);
		if(aircraft_status.activation == 1)
			break;
		printf("Activation failed, return %d\n",aircraft_status.activation);
		XY_Debug_Send_At_Once("Activation failed, return %d\n",aircraft_status.activation);
		aircraft_status.activation = -1;
		usleep(50000);
	}
	printf("Activation succeed, start to request controller.\n");
	XY_Debug_Send_At_Once("Activation succeed, start to request controller.\n");

	int count = 0;
	while(1)
	{
		printf("obtaining.\n");
		DJI_Pro_Control_Management(1,NULL);
		while(aircraft_status.obtained_control == -1 && count < 20)
		{
			usleep(50000);	//50ms
			count++;
			printf("waitting.\n");
		}
		if(aircraft_status.obtained_control == 1)
			break;
		printf("Obtain control, return %d\n", aircraft_status.obtained_control);
		XY_Debug_Send_At_Once("Obtain control, return %d\n", aircraft_status.obtained_control);
		aircraft_status.obtained_control = -1;
		usleep(50000);
	}	
	printf("Obtained controller, start to flight.\n");
	XY_Debug_Send_At_Once("Obtained controller, start to flight.\n");

	enable_report_drone_pos();

	if(XY_SD_Log_Setup() < 0)
	{
		printf("Log Function Open ERROR.\n");
		XY_Debug_Send_At_Once("Log Function Open ERROR.\n");
	}
	XY_Start_Store_Log();
	
	XY_Drone_Execute_Task();
	
	XY_Stop_Store_Log();
	
	sleep(50);
	pause();
	
	return 0;
}
