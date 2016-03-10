#ifndef __WIRELESS_DEBUG_H__
#define __WIRELESS_DEBUG_H__

#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include "common/uart_common.h"
#include "common/common.h"
#include "image_identify.h"
#include <math.h>


#define DEBUG_DEV_NAME "/dev/ttyTHS0"


#define DEBUG_HEIGHT			0
#define DEBUG_XYZ				1

#define DECLARED_MSG	3
#define MSG_1_SIZE		50
#define MSG_2_SIZE		100
#define MSG_3_SIZE		100


struct debug_info{
	char msg_0[MSG_1_SIZE];
	char msg_1[MSG_2_SIZE];
	char msg_2[MSG_3_SIZE];
};

int sdk_send_debug_info(const char *str);
int sdk_find_available_msg(int need_len, int used_msg);
int XY_Debug_Setup(void);
int XY_Debug_Easy_Send(char *buf, int len);
int XY_Debug_Sprintf(int seq, const char *fmt, ...);
void XY_Debug_Send_At_Once(const char *fmt, ...);
int sdk_wireless_debug_setup(const char *device, int baudrate);
int create_debug_check_and_send_thread(void);



#endif
