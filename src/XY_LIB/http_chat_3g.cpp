/*
 * Description	: Chat by TCP/HTTP
 * Author		: Zibo Zhou
 * Time			: 2016/1/6
 */

#include "http_chat_3g.h"
#include "thread_common_op.h"
#include "wireless_debug.h"


#define HTTP_DEBUG_PRINT	1
#define HTTP_FUNC_ON		1

int sock_fd = -1;
char http_post_str[HTTP_POST_DATA_SIZE];
int http_post_head_length = 0;
int new_order = 0;

pthread_mutex_t post_req_flag_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t post_msg_lock = PTHREAD_MUTEX_INITIALIZER;
sem_t report_drone_pos_sem;


int post_req_flag = 0;

struct post_info post_package = {0};


void XY_Update_Post_Flag(int _offset)
{
	pthread_mutex_lock(&post_req_flag_lock);
	post_req_flag |= (1<<_offset);
	pthread_mutex_unlock(&post_req_flag_lock);
}

int XY_Get_Post_Flag(void)
{
	int ret;
	pthread_mutex_lock(&post_req_flag_lock);
	ret = post_req_flag;
	pthread_mutex_unlock(&post_req_flag_lock);

	return ret;
}

void XY_Empty_Post_Flag(int _offset)
{
	pthread_mutex_lock(&post_req_flag_lock);
	post_req_flag &= ~(1<<_offset);
	pthread_mutex_unlock(&post_req_flag_lock);
}


int XY_Http_Chat_Setup(void)
{

	if( create_link_in_http() < 0)
	{
		return -1;
	}
	
	if(XY_Create_Thread(wait_order_thread_func, NULL, THREADS_WAIT_ORDER, -1, SCHED_RR, 96) < 0)
	{
		printf("Create Wait Order Thread Error.\n");
		return -1;
	}
	
	return 0;

}

int XY_Http_Reported_Setup(void)
{
	if(sem_init(&report_drone_pos_sem, 0, 0) != 0)
		return -1;
	
	if(XY_Create_Thread(reported_data_thread_func, NULL, THREADS_HTTP_POST, -1, SCHED_RR, 94) < 0)
	{
		printf("Create Http Post Thread Error.\n");
		return -1;
	}
#if 0
	if(XY_Create_Thread(keep_alive_thread_func, NULL, THREADS_HTTP_KEEP_ALIVE, -1, SCHED_RR, 94) < 0)
	{
		printf("Create Http Keep-alive Thread Error.\n");
		return -1;
	}

#else
	if(XY_Create_Thread(report_drone_pos_at_intervals_thread_func, NULL, THREADS_HTTP_REPORT_POS, -1, SCHED_RR, 90) < 0)
	{
		printf("Create Http Report Drone Pos Thread Error.\n");
		return -1;
	}
#endif
	

	return 0;
}

int create_socket(int *_sock_fd)
{
	if( (*_sock_fd = socket(AF_INET, SOCK_STREAM, 0) ) < 0)
	{
		perror("socket error");
		return -1;
	}
	return 0;
}

int setup_addr(struct sockaddr_in *_addr, unsigned short _port, const char *hostname)
{
	struct hostent *host = NULL;
	
	_addr->sin_family = AF_INET;
	_addr->sin_port = htons(_port);
	host = gethostbyname(hostname);
	if(!host)
	{
		printf("get host error.\n");
		return -1;
	}
	_addr->sin_addr = *(struct in_addr *)host->h_addr_list[0];
	bzero(&(_addr->sin_zero), 8);

	return 0;
}

int connect_to_server(int _sock_fd)
{
	struct sockaddr_in server_addr;
	
	if( setup_addr(&server_addr, SERVER_PORT, SERVER_NAME) < 0)
	{
		perror("setup addr");
		return -1;
	}

	printf("* About to connect() to %s port %d.\n", SERVER_NAME, SERVER_PORT);
	if( connect(_sock_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr) ) == -1)
	{
		perror("connect error");
		return -1;
	}
	printf("*  Trying %s... connected\n", inet_ntoa(server_addr.sin_addr));
	
	return 0;
}

void setup_post_data_head(void)
{
	memset(http_post_str, 0, HTTP_POST_DATA_SIZE);
	strcat(http_post_str, "POST /api/order/set_express HTTP/1.1\r\n");
	strcat(http_post_str, "User-Agent: curl/7.22.0 (x86_64-pc-linux-gnu) libcurl/7.22.0 OpenSSL/1.0.1 zlib/1.2.3.4 libidn/1.23 librtmp/2.3\r\n");
	strcat(http_post_str, "Host: cs.sysmagic.com.cn\r\n");
	strcat(http_post_str, "Accept: */*\r\n");
	http_post_head_length = strlen((const char *)http_post_str);
}

int create_link_in_http(void)
{
	if(create_socket(&sock_fd) < 0)
	{
		printf("create socket error.\n");
		return -1;
	}

	if(connect_to_server(sock_fd) < 0)
	{
		printf("connect to server error.\n");
		close(sock_fd);
		return -1;
	}

	setup_post_data_head();

	return 0;
}


int send_http_post_data(char *_data)
{
	int index = 0;
	int ret = 0;
	int packet_length = 0;
	char temp_str[50];
	
	index = http_post_head_length;
	memset(http_post_str + index, 0, HTTP_POST_DATA_SIZE - index);

	packet_length = strlen((const char*) _data);
	sprintf(temp_str, "Content-Length: %d\r\n\r\n", (packet_length-2) );	//2为"\r\n"长度
	strcat(http_post_str, temp_str);
	strcat(http_post_str, _data);

#if HTTP_DEBUG_PRINT	
	printf("*  Send data in http...\n");
	printf("%s", http_post_str);
#endif

	ret = send(sock_fd, (void *)http_post_str, strlen((const char *)http_post_str), 0); 
	if (ret < 0)
	{ 
		printf("send error %d, Error message'%s'\n", errno, strerror(errno)); 
		return -1;
	}
	else
	{ 
		printf("send success, total send %d \n", ret); 
	} 

	return 0;
}

int post_msg_size[DECLARED_POST_MSG] = {POST_MSG_0_SIZE, POST_MSG_1_SIZE, POST_MSG_2_SIZE, POST_MSG_3_SIZE, POST_MSG_4_SIZE};
int find_available_post_msg(int need_len, int used_msg)
{
	int free_msg = -1;
	int i = 0;
	
	free_msg = (int)(pow(2, DECLARED_POST_MSG)-1) - used_msg;

	if(free_msg == 0)
		return -1;

	while(i < DECLARED_POST_MSG)
	{
		if(free_msg & 0x01)
		{
			if(need_len < post_msg_size[i])
				return i;
		}
		free_msg = free_msg >> 1;	
		i++;
	}

	return -1;
}

int XY_Send_Http_Post_Request_Data(int seq, char *fmt, ...)
{
	va_list args;
	int r;

	if( ((r = XY_Get_Post_Flag()) & (1<<seq)) != 0  || seq == NULL)
	{
		seq = find_available_post_msg( strlen((const char *)fmt), r); 
		if(seq < 0)
			return 1;
	}
	
	va_start(args, fmt);	//使ap指向(fmt的基地址+fmt的长度)内存位置，即第二个参数位置
	pthread_mutex_lock(&post_msg_lock);
	switch(seq)
	{
		case 0:
			memset(post_package.msg_0, 0, strlen((const char *)post_package.msg_0));
			vsprintf(post_package.msg_0, fmt, args);
			break;
		case 1:
			memset(post_package.msg_1, 0, strlen((const char *)post_package.msg_1));
			vsprintf(post_package.msg_1, fmt, args);
			break;
		case 2:
			memset(post_package.msg_2, 0, strlen((const char *)post_package.msg_2));
			vsprintf(post_package.msg_2, fmt, args);
			break;
		case 3:
			memset(post_package.msg_3, 0, strlen((const char *)post_package.msg_3));
			vsprintf(post_package.msg_3, fmt, args);
			break;
		case 4:
			memset(post_package.msg_4, 0, strlen((const char *)post_package.msg_4));
			vsprintf(post_package.msg_4, fmt, args);
			break;
	}
	pthread_mutex_unlock(&post_msg_lock);
	va_end(args);
	
	XY_Update_Post_Flag(seq);

	return 0;
}


static void *reported_data_thread_func(void * arg)
{
	int _flag = 0;
	int i = 0;
	int ret = 0;
	int err_cnt[10] = {0};
	int recv_bytes;
	char recv_buf[512];
	struct timeval tv_recv_timeo;
	
	tv_recv_timeo.tv_sec = 1; 	// set recv timeout is 1s
	tv_recv_timeo.tv_usec = 0; 
	ret = setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&tv_recv_timeo, sizeof(struct timeval));
	if(ret == -1)
	{
		printf("Set Recv TimeOut Error.\n");
		
	}
	
	while(1)
	{	
		usleep(200000);	// 200ms
		if( (_flag = XY_Get_Post_Flag()) == 0)	// no msg should be sent
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
							ret = send_http_post_data(post_package.msg_0);
							break;
						case 1:
							ret = send_http_post_data(post_package.msg_1);
							break;
						case 2:
							ret = send_http_post_data(post_package.msg_2);
							break;
						case 3:
							ret = send_http_post_data(post_package.msg_3);
							break;
						case 4:
							ret = send_http_post_data(post_package.msg_4);
							break;
					}
					if(ret == 0)	//send success
					{
#if 1
						memset(recv_buf, 0, 512);
						recv_bytes = recv(sock_fd, (void *)recv_buf, 512, 0);	// recv is blocking system calls
						if( recv_bytes == 0 )
						{
							close(sock_fd); 
							printf("Network errors, stop!\n"); 
							if( create_link_in_http() < 0 )
							{
								printf("create link in http error.\n");
								goto _exit;
							}
						}
						else if( recv_bytes == -1 )	//time out
						{
							goto _err_handle;
						}
						
						printf("recv_bytes = %d\n", recv_bytes); 
#if HTTP_DEBUG_PRINT
						printf("%s\n", recv_buf); 
#endif
						
						if( !strstr(recv_buf, "HTTP/1.1 200 OK") )
						{
							goto _err_handle;
						}
#endif

						err_cnt[i] = 0;
						XY_Empty_Post_Flag(i);
					}
					else
					{
_err_handle:
						err_cnt[i]++;
						if(err_cnt[i] > 2)
						{
							printf("Send bad\n");
							err_cnt[i] = 0;
							XY_Empty_Post_Flag(i);
						}
					}
				}
				_flag = _flag >> 1;	
				i++;
			}
			i = 0;
		}
		
	}
_exit:
	pthread_exit(NULL);
}

cJSON *json;
static void *wait_order_thread_func(void * arg)
{	
	char http_req[HTTP_REQ_SIZE], recv_buf[HTTP_RECV_BUF_SIZE];
	fd_set http_fds; 
	struct timeval tv;
	int ret, recv_cnt = 0;
	struct timeval tpstart, tpend;
	int timeuse, timesleep;
	int length = 0;
	char json_buf[500] = {0};

	memset(http_req, 0, HTTP_REQ_SIZE);
	strcat(http_req, "GET /api/order/get_json_order HTTP/1.1\r\n");		//request line
	strcat(http_req, "User-Agent: curl/7.22.0 (x86_64-pc-linux-gnu) libcurl/7.22.0 OpenSSL/1.0.1 zlib/1.2.3.4 libidn/1.23 librtmp/2.3\r\n");
	strcat(http_req, "Host: cs.sysmagic.com.cn\r\n");
	strcat(http_req, "Accept: */*\r\n\r\n");

#if HTTP_DEBUG_PRINT
	printf("%s", http_req);
#endif
	
	while(1)
	{
		ret = send(sock_fd, (void *)http_req, strlen(http_req), 0); 
		if (ret < 0)
		{ 
			printf("send error %d, Error message'%s'\n", errno, strerror(errno)); 
			goto pre_again;
		}
		else
		{ 
			printf("send success, total send %d \n", ret); 
		} 
		
		tv.tv_sec = 2; 
		tv.tv_usec = 0; 

		FD_ZERO(&http_fds); 
		FD_SET(sock_fd, &http_fds); 

		switch( select(sock_fd + 1, &http_fds, NULL, NULL, &tv) )
		{
			case -1:	// 错误
				close(sock_fd); 	//是否需要重新连接?
				printf("some thing read error!\n");		
				XY_Debug_Send_At_Once("some thing read error!\n");
				if( create_link_in_http() < 0 )
				{
					printf("create link in http error.\n");
					goto _exit;
				}

			case 0:		// 超时
				printf("request time out!\n");
				XY_Debug_Send_At_Once("request time out!\n");
				goto pre_again;

			default:
				memset(recv_buf, 0, HTTP_RECV_BUF_SIZE); 
				recv_cnt = recv(sock_fd, (void *)recv_buf, HTTP_RECV_BUF_SIZE, 0);		// recv会等到协议把数据接收完毕才会进行copy和返回
				
				printf("recv count = %d\n", recv_cnt); 
				/*
				 * recv return
				 * <0 : copy error
				 * =0 : 网络中断
				 * >0 : 接收到数据大小
				 */ 
				if (recv_cnt == 0)
				{ 
					close(sock_fd); 
					printf("Network errors, stop!\n"); 
					XY_Debug_Send_At_Once("Network errors, stop!\n");
					if( create_link_in_http() < 0 )
					{
						printf("create link in http error.\n");
						goto _exit;
					}
					
				}

#if HTTP_DEBUG_PRINT
				printf("%s\n", recv_buf); 
#endif
				break;
		}
		if( strstr(recv_buf, "Connection: close") )
		{
			printf("Connection: close\n");
			close(sock_fd); 
			if( create_link_in_http() < 0 )
			{
				printf("create link in http error.\n");
				goto _exit;
			}
		}
		length = strstr(recv_buf, "}") - strstr(recv_buf, "{") + 1;
		if(length == 2)
		{
			printf("empty packets\n");
			XY_Debug_Send_At_Once("empty packets\n");
			goto pre_again;
		}
		memcpy(json_buf, strstr(recv_buf, "{"), length);
		json = cJSON_Parse(json_buf);
		printf("json_buf is %s\n", json_buf);
		if (!json)
		{
			printf("Error before: [%s]\n", cJSON_GetErrorPtr());
			goto pre_again;
		}

		set_order_status();
		
		printf("Get: \n> order id is %s\n> num is %d\n> seq is %d\n> longti is %.8lf\n> lati is %.8lf\n",
														cJSON_GetObjectItem(json, "order_id")->valuestring, 
														cJSON_GetObjectItem(json, "num")->valueint, 
														cJSON_GetObjectItem(json, "seq")->valueint, 
														cJSON_GetObjectItem(json, "lng")->valuedouble, 
														cJSON_GetObjectItem(json, "lat")->valuedouble);

		
		XY_Debug_Send_At_Once("Get: \n> order id is %s\n> num is %d\n> seq is %d\n> longti is %.8lf\n> lati is %.8lf\n",
														cJSON_GetObjectItem(json, "order_id")->valuestring, 
														cJSON_GetObjectItem(json, "num")->valueint, 
														cJSON_GetObjectItem(json, "seq")->valueint, 
														cJSON_GetObjectItem(json, "lng")->valuedouble, 
														cJSON_GetObjectItem(json, "lat")->valuedouble);
		
		goto _exit;

		
pre_again:
		usleep(1000000);
	}

_exit:
	pthread_exit(NULL);
}

char *get_order_id_from_json(void)
{
	return cJSON_GetObjectItem(json, "order_id")->valuestring;
}


void set_order_status(void)
{
	new_order = 1;
}

int get_order_status(void)
{
	return new_order;
}
void message_server_keep_alive(void)
{
#if HTTP_FUNC_ON
	XY_Send_Http_Post_Request_Data(4, "\r\n", get_order_id_from_json() );
#endif
}

void message_server_load_is_okay(void)
{
#if HTTP_FUNC_ON
	XY_Send_Http_Post_Request_Data(0, "msg_id=1&order_id=%s&load=1\r\n", get_order_id_from_json() );
#endif
}

void message_server_finding_mark(void)
{
#if HTTP_FUNC_ON
	//标记定位中
	XY_Send_Http_Post_Request_Data(1, "msg_id=3&order_id=%s&landing=1\r\n", get_order_id_from_json() );
#endif
}

void message_server_found_mark(void)
{
#if HTTP_FUNC_ON
	//定位完成
	XY_Send_Http_Post_Request_Data(2, "msg_id=3&order_id=%s&landing=2\r\n", get_order_id_from_json() );
#endif
}

void message_server_deliver_is_okay(void)
{
#if HTTP_FUNC_ON
	//已送达, 准备返航
	XY_Send_Http_Post_Request_Data(3, "msg_id=4&order_id=%s&arrived=0\r\n", get_order_id_from_json() );
#endif
}

float double_to_6_decimal_places(double _a)
{
	uint64_t _b;

	_b = (int)( _a * 1000000 );

	return (float)(_b / 1000000.0);
}

void gps_radian_to_degree(double *_a, double *_b)
{
	double pi = 3.141592653;
	
	*_a = *_a * 180.0 / pi;
	*_b = *_b * 180.0 / pi;
}

void message_server_current_pos_of_drone(void)
{
#if HTTP_FUNC_ON
	api_pos_data_t _cpos;
	float f_longti, f_lati;
	
	DJI_Pro_Get_Pos(&_cpos);

	gps_radian_to_degree(&_cpos.longti, &_cpos.lati);
	
	f_longti 	= double_to_6_decimal_places(_cpos.longti) 	+ GD2GE_LONGTI_DIFF;
	f_lati		= double_to_6_decimal_places(_cpos.lati) 	+ GD2GE_LATI_DIFF;
	
	XY_Send_Http_Post_Request_Data(4, 	"msg_id=2&order_id=%s&longti_cur=%.6f&lati_cur=%.6f&time=50\r\n", 
										get_order_id_from_json(),
										f_longti, f_lati);
#endif
}


static void *keep_alive_thread_func(void * arg)
{
	while(1)
	{
		sleep(10);
		message_server_keep_alive();
	}
}


void enable_report_drone_pos(void)
{
#if HTTP_FUNC_ON
	sem_post(&report_drone_pos_sem);
#endif
}

static void *report_drone_pos_at_intervals_thread_func(void *arg)
{

	sem_wait(&report_drone_pos_sem);
	
	while(1)
	{
		sleep(1);
		message_server_current_pos_of_drone();
	}
}


