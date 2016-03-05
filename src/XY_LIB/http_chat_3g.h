#ifndef __HTTP_CHAT_3G_H__
#define __HTTP_CHAT_3G_H__

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <time.h> 
#include <errno.h> 
#include <netdb.h>
#include <stdarg.h>
#include <semaphore.h>
#include "cJSON.h"




#define SERVER_NAME		"cs.sysmagic.com.cn"
#define SERVER_PORT		(80)

#define HTTP_REQ_SIZE		(512)
#define HTTP_RECV_BUF_SIZE	(1024)
#define HTTP_POST_DATA_SIZE	(1024)


#define DECLARED_POST_MSG	5
#define POST_MSG_0_SIZE		300
#define POST_MSG_1_SIZE		300
#define POST_MSG_2_SIZE		300
#define POST_MSG_3_SIZE		300
#define POST_MSG_4_SIZE		300

#define GD2GE_LONGTI_DIFF	(0.004740)
#define GD2GE_LATI_DIFF		(-0.002450)

struct post_info{
	char msg_0[POST_MSG_0_SIZE];
	char msg_1[POST_MSG_1_SIZE];
	char msg_2[POST_MSG_2_SIZE];
	char msg_3[POST_MSG_3_SIZE];
	char msg_4[POST_MSG_4_SIZE];
};



int XY_Http_Chat_Setup(void);
int XY_Http_Reported_Setup(void);
int create_link_in_http(void);
static void *reported_data_thread_func(void * arg);
static void *wait_order_thread_func(void * arg);
int XY_Send_Http_Post_Request_Data(int seq, char *fmt, ...);
char *get_order_id_from_json(void);
void set_order_status(void);
int get_order_status(void);

void message_server_keep_alive(void);
void message_server_load_is_okay(void);
void message_server_finding_mark(void);
void message_server_found_mark(void);
void message_server_deliver_is_okay(void);
void message_server_current_pos_of_drone(void);
void enable_report_drone_pos(void);

static void *keep_alive_thread_func(void * arg);
static void *report_drone_pos_at_intervals_thread_func(void *arg);





#endif
