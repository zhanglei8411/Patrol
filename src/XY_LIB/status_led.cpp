#include "status_led.h"
#include "thread_common_op.h"
#include <semaphore.h>


typedef struct{
	int cmd;
	int ing;
}LedJob;


LedJob led_jobs[10];

int led_fd = -1;
pthread_mutex_t led_job_status_lock = PTHREAD_MUTEX_INITIALIZER;
sem_t show_sem;


int XY_Status_Led_Setup(void)
{
	if(init_led(&led_fd) != 0)
	{
		led_fd = -1;
		return -1;
	}
	
	if(XY_Create_Thread(led_show_status_thread_func, NULL, THREADS_LED, -1, SCHED_RR, 88) < 0)
	{
		printf("Create Led Show Status Thread Error.\n");
		return -1;
	}

	return 0;
}


int init_led(int *fd)
{
	/* request the controller of gpio158 */
	*fd = open(SYSFS_LED_EXPORT, O_WRONLY);
	if(*fd == -1)
	{
		printf("Open export failed.\n");
		goto error;
	}
	write(*fd, SYSFS_LED_EXPORT_VAL, sizeof(SYSFS_LED_EXPORT_VAL));
	close(*fd);

	/* set gpio158 direction */
	*fd = open(SYSFS_LED_DIR, O_WRONLY);
	if(*fd == -1)
	{
		printf("Open led diretion failed.\n");
		goto release;
	}
	write(*fd, SYSFS_LED_DIR_VAL, sizeof(SYSFS_LED_DIR_VAL));
	close(*fd);

	/* open gpio158 value file to write */
	*fd = open(SYSFS_LED_VAL, O_WRONLY);
	if(*fd == -1)
	{
		printf("Open led value failed.\n");
		goto release;
	}

	turn_off_led(*fd);
	
	if(sem_init(&show_sem, 0, 0) != 0)
		goto release;
	
	return 0;
	
release:
	close(*fd);

error:
	return 1;
}

void set_led(int _fd, unsigned int _val)
{
	char ioval[1] = {0};
	int ret;

	_val = (_val > 1 ? 1 : _val);

	ioval[0] = (_val == 0 ? '0' : '1');
	ret = write(_fd, ioval, 1);
	lseek(_fd, 0, SEEK_SET);
}


void turn_on_led(int _fd)
{
	set_led(_fd, 1);
}

void turn_off_led(int _fd)
{
	set_led(_fd, 0);
}

void ioctl_led(int _new_cmd)
{
	int i = 0;
	
	pthread_mutex_lock(&led_job_status_lock);
	while(led_jobs[i].ing == 1)
		i++;
	led_jobs[i].ing = 1;
	led_jobs[i].cmd = _new_cmd;
	pthread_mutex_unlock(&led_job_status_lock);
	

	sem_post(&show_sem);
}

int get_led_cmd(void)
{
	int ret;	
	
	pthread_mutex_lock(&led_job_status_lock);
	ret = led_jobs[0].cmd;
	pthread_mutex_unlock(&led_job_status_lock);

	return ret;
}

void update_led_jobs(void)
{
	int i, j = 0;
	
	pthread_mutex_lock(&led_job_status_lock);

	for(j=0; j<(10-1); j++)
	{
		led_jobs[j].cmd = led_jobs[j+1].cmd;
		led_jobs[j].ing = led_jobs[j+1].ing;
	}
	
	led_jobs[j].ing = 0;
	pthread_mutex_unlock(&led_job_status_lock);
	
}

/* default */
int led_period = 2000;	//ms
int led_duty = 0;
void update_show_policy(int _cmd)
{
	switch(_cmd)
	{
		case 0:
			led_period = 1000;
			led_duty = 0;
			break;
		case 1:
			led_period = 1000;
			led_duty = 1000;
			break;
		case 2:
			led_period = 200;
			led_duty = 100;
			break;
		case 3:
			led_period = 2000;
			led_duty = 1000;
			break;
		default:
			break;
	}
}


static void *led_show_status_thread_func(void * arg)
{	
	int show_cnt = 0;

	sem_wait(&show_sem);
	
	while(1)
	{	
		if(show_cnt < 20)
		{
			update_show_policy(get_led_cmd());
			turn_on_led(led_fd);
			usleep(led_duty * 1000);
			turn_off_led(led_fd);
			usleep((led_period - led_duty) * 1000);
			show_cnt++;
		}
		else
		{
			update_led_jobs();
			sem_wait(&show_sem);	
			show_cnt = 0;
		}
	}
	pthread_exit(NULL);
}


