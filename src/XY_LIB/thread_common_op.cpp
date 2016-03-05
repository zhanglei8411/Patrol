#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <sched.h>
#include <sys/sysinfo.h>
#include "thread_common_op.h"

Thread_t threads[THREADS];

int XY_Create_Thread(void *(* func)(void *), void *arg, int num, int _bind, int _policy, int _priority)
{
	pthread_attr_t _attr;
		
	if(num >= THREADS)
		return -1;

#if 0
	if(threads[num].used == 1)
	{
		printf("%s, line %d: this thread_t not free.\n", __func__, __LINE__);
		return -1;
	}
#endif

	pthread_attr_init(&_attr);
	
	if(_policy != -1 && _priority != -1)
	{
		if(setup_thread_scheduler(&_attr, _policy, _priority) != 0)
		{
			printf("%s, line %d: setup thread scheduler error.\n", __func__, __LINE__);
			return -1;
		}
	}

	if(pthread_create(&threads[num].tid, &_attr, func, arg) != 0)
	{
		printf("%s, line %d: thread create error.\n", __func__, __LINE__);
		return -1;
	}
	else
	{
		if(_bind != -1)
		{
			if(thread_binding_cpu(threads[num].tid, _bind) != 0)
			{
				printf("%s, line %d: bind cpu failed after create thread.\n", __func__, __LINE__);
				return -1;
			}
		}
		threads[num].used = 1;
	}
	
	return 0;
}

pthread_t get_tid(int num)
{
	if(threads[num].used != 1)
		return 0;
	return threads[num].tid;
}

int thread_binding_cpu(pthread_t _tid, int cpu_seq)
{
	int cpu_core_num = get_nprocs();
	printf("CPU has %d core\n", cpu_core_num);
	cpu_set_t mask; 
 	cpu_set_t get; 

	_tid = (_tid == NULL ? pthread_self() : _tid);
	/* 绑定 */
	cpu_seq = (cpu_seq >= (cpu_core_num - 1) ? (cpu_core_num - 1) : cpu_seq);
	CPU_ZERO(&mask); 
	CPU_SET(cpu_seq, &mask); 
	if (pthread_setaffinity_np(_tid , sizeof(mask), &mask) == -1)
	{ 
		printf("warning: could not set CPU affinity, continuing\n"); 
		return 1;
	} 
	
	/* 查询绑定结果 */
	CPU_ZERO(&get); 
	if (pthread_getaffinity_np(_tid, sizeof(get), &get) == -1) 
	{ 
		printf("warning: cound not get cpu affinity\n"); 
		return 1;
	} 
	for(int i = 0; i < cpu_core_num; i++) 
	{ 
		if (CPU_ISSET(i, &get)) 
		{ 
			printf("this thread %d is running processor : %d\n", (int)(_tid), i); 
		} 
	}
	return 0;
}


int setup_thread_scheduler(pthread_attr_t *_attr, int _policy, int _priority)
{
	struct sched_param param;

	param.sched_priority = _priority;

	pthread_attr_init(_attr);

	if(pthread_attr_setinheritsched(_attr, PTHREAD_EXPLICIT_SCHED) != 0)			//必要!!!必要!!!PTHREAD_EXPLICIT_SCHED表示使用新设置的attr值，而不是继承调用者线程的值
		goto _error;
	if(pthread_attr_setschedpolicy(_attr, _policy) != 0)							//设置调度策略
		goto _error;
	if(pthread_attr_setschedparam(_attr, &param) != 0)								//设置优先级
		goto _error;

	return 0;

_error:
	return -1;
}

int change_scheduler_in_thread(int _policy, int _priority)
{
	struct sched_param param;
	if(_policy == -1)
	{
		pthread_getschedparam(pthread_self(), &_policy, &param);
	}
	param.sched_priority = _priority;
	if(pthread_setschedparam(pthread_self(), _policy, &param) != 0)
	{
        printf("%s, line %d: pthread_setschedparam failed\n", __func__, __LINE__);
		return -1;
	}
	/* get new setup */
	pthread_getschedparam(pthread_self(), &_policy, &param);
	printf("thread running at %s/%d\n", (_policy == SCHED_FIFO ? "FIFO" : (_policy == SCHED_RR ? "RR" : (_policy == SCHED_OTHER ? "OTHER" : "Unknown"))), 
										param.sched_priority);
										
	return 0;
}


void get_self_thread_priority(int *_priority)
{
	int policy;
	struct sched_param param;
	
	pthread_getschedparam(pthread_self(), &policy, &param);
	*_priority = param.sched_priority;
}


