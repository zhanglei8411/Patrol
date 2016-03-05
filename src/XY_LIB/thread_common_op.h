#ifndef __THREAD_COMMON_OP_H__
#define __THREAD_COMMON_OP_H__


#define THREADS	20

typedef struct{
	int used;
	pthread_t tid;
}Thread_t;


#define GENERAL_JOB_CPU	0
#define CAPTURE_JOB_CPU 1
#define IMAGE_JOB_CPU	CAPTURE_JOB_CPU

#define THREADS_DEBUG				0
#define THREADS_CAPTURE				1
#define THREADS_IDENTIFY			2
#define THREADS_SERIALRECV			3
#define THREADS_POLL				4
#define THREADS_ULTRA				5
#define THREADS_ROUTE				6
#define THREADS_LED					7
#define THREADS_WAIT_ORDER			8
#define THREADS_HTTP_POST			9
#define THREADS_LOG					10
#define THREADS_DELIVER				11
#define THREADS_GOBACK				12
#define THREADS_HTTP_KEEP_ALIVE		13
#define THREADS_HTTP_REPORT_POS		14



int XY_Create_Thread(void *(* func)(void *), void *arg, int num, int _bind, int _policy, int _priority);
pthread_t get_tid(int num);
int thread_binding_cpu(pthread_t _tid, int cpu_seq);
int setup_thread_scheduler(pthread_attr_t *_attr, int _policy, int _priority);
int change_scheduler_in_thread(int _policy, int _priority);
void get_self_thread_priority(int *_priority);

#endif
