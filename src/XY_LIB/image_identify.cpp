#include "image_identify.h"
#include "thread_common_op.h"
#include "wireless_debug.h"
#include <semaphore.h>

using namespace cv;
using namespace std;

xyVision::GetTarget sample("config.ini");


Offset_Data offset_data;

pthread_mutex_t capture_on_off_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t offset_msg_lock = PTHREAD_MUTEX_INITIALIZER;
int capture_on_flag = 0;
sem_t capture_start_sem;

sem_t image_get_sem;
sem_t image_handle_sem;
sem_t offset_get_sem;

int tell_external_offset_is_available(void)
{
	return sem_post(&offset_get_sem);		//if error return -1
}

int check_offset_data_if_available(int _get_id)
{

	int mask = 0;
	pthread_mutex_lock(&offset_msg_lock);
	mask = offset_data.gotten;
	pthread_mutex_unlock(&offset_msg_lock);
	
	// no update
	if( mask == 0 )
	{
		return -1;
	}
	// offset was update
	if( mask & 0x01 )
	{
		// offset hasn't been gotten
		if( (mask & _get_id)  == 0)
		{
			return 0;
		}
		else	// offset has been gotten
		{
			return -1;
		}
			
	}
	return 0;
}

void set_offset_data(Point3f _point)
{
	pthread_mutex_lock(&offset_msg_lock);
	offset_data._offset.x = _point.x;
	offset_data._offset.y = _point.y;
	offset_data._offset.z = _point.z;
	offset_data.gotten = 0x01;
	pthread_mutex_unlock(&offset_msg_lock);
}

Offset get_offset_data(int _get_id)
{
	Offset ret;
	pthread_mutex_lock(&offset_msg_lock);
	ret = offset_data._offset;
	offset_data.gotten |= _get_id;
	pthread_mutex_unlock(&offset_msg_lock);

	return ret;
}

/* Instructions : 
 * Offset offset;
 * XY_Get_Offset_Data(&offset, OFFSET_GET_ID_A);
 */
int XY_Get_Offset_Data(Offset *_data, int _get_id)
{
	if(check_offset_data_if_available(_get_id) == -1)
	{
		return -1;
	}

	*_data = get_offset_data(_get_id);
	return 0;
}



void set_capture_on_flag(int _val)
{
	_val = _val > 1 ? 1 : _val;
	pthread_mutex_lock(&capture_on_off_lock);
	capture_on_flag = _val;
	pthread_mutex_unlock(&capture_on_off_lock);
}

void clear_capture_on_flag(void)
{
	set_capture_on_flag(0);
}

int get_capture_on_flag(void)
{
	int ret;
	pthread_mutex_lock(&capture_on_off_lock);
	ret = capture_on_flag;
	pthread_mutex_unlock(&capture_on_off_lock);

	return ret;
}

int if_capture_on(void)
{
	return get_capture_on_flag();
}

void wait_capture_on(void)
{
	sem_wait(&capture_start_sem);
}

int XY_Start_Capture(void)
{
	clear_sample_states();
	set_capture_on_flag(1);
	sem_post(&capture_start_sem);
	
	return 0;
}


int XY_Stop_Capture(void)
{
	Offset _offset;
	set_capture_on_flag(0);
	//clear_sample_states();
	XY_Get_Offset_Data(&_offset, OFFSET_GET_ID_A);
	return 0;
}

stringstream stream;
string dirname = "/mnt/sdcard/image/";
char second_level_dirname[10] = {0};
string total_filename;
float cur_height;

int get_current_cnt_in_profile(void)
{
	char cnt[3] = {0};
	int ret, fd;
	
	fd = open("/home/ubuntu/Work/Test/test-cnt-log", O_RDONLY);
	if(fd == -1)
	{
		printf("test-cnt-log open error.\n");
	}
	memset(cnt, 0, 3);
	ret = read(fd, cnt, 3);
	if(ret == -1)
	{
		perror("write");
	}
	
	return atoi(cnt);
}

#if 1
int mk_image_store_dir(void)
{
	int status = 0;
	int cur_cnt =0;
	char mkdirname[50] = {0};
	
	cur_cnt = get_current_cnt_in_profile();
	sprintf(mkdirname, "%simage-%d", "/mnt/sdcard/image/", cur_cnt);
	sprintf(second_level_dirname, "image-%d/", cur_cnt);
	
	status = mkdir(mkdirname, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

	if( status < 0 )
	{
		return -1;
	}
	//printf("mkdirname: %s\n", mkdirname);
	printf("second_level_dirname: %s\n", second_level_dirname);

	return 0;
}
#endif

int save_image_into_sdcard(Mat _image)
{
	struct timeval	  tv;  
	struct tm		  *tmlocal; 
	char name_head[50] = {0};

	XY_Pro_Get_Pos_Height(&cur_height);
	
	if( gettimeofday(&tv, NULL) != 0)
	{
		printf("ERROR get timestamp for image name.\n");
		
	}
	tmlocal = localtime(&tv.tv_sec); 
	sprintf(name_head, "%d_%d_%ld", tmlocal->tm_min,
									tmlocal->tm_sec,
									tv.tv_usec); 

	stream.str("");
	stream << cur_height;
	total_filename = dirname + second_level_dirname + name_head + "H" + stream.str() + ".jpg";
	cout << total_filename << endl;
	const char *p = total_filename.c_str();
	imwrite(p, _image);
	return 0;
}

Mat img;
static void *capture_thread_func(void * arg)
{	
	//thread_binding_cpu(NULL, CAPTURE_JOB_CPU);

	int time_ = 0;
	int reopen_cnt = 0;
	int cache_cnt = 0;
	//xyVision::GetTarget sample("config.ini");

	//reopen has no effect

_reopen:
	cv::VideoCapture cap(0);
	//or cv::VideoCapture cap(200);
	if(!cap.isOpened())
	{
		printf("Cap not open\n");
		XY_Debug_Sprintf(1, "Cap not open\n");
		sleep(1);
		reopen_cnt++;
		if(reopen_cnt > 3)
			goto _exit;
		goto _reopen;
	}
	
	cap.set(CV_CAP_PROP_FRAME_WIDTH, (double)1280);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, (double)720);
	cap.set(CV_CAP_PROP_SATURATION, (double)0.5);
	
	cout << "CV_CAP_PROP_FRAME_WIDTH " << cap.get(CV_CAP_PROP_FRAME_WIDTH) << endl;
	cout << "CV_CAP_PROP_FRAME_HEIGHT " << cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
	cout << "CV_CAP_PROP_SATURATION " << cap.get(CV_CAP_PROP_SATURATION) << endl;
	
	while(1)
	{	
		if(if_capture_on())
		{
			sem_wait(&image_handle_sem);	//除第一次外，等待image_identify_thread_func处理完图像
			cap >> img;
			if(!img.data)
			{
				printf("No image!\n");
				sleep(2);
				sem_post(&image_handle_sem);
				continue;
			} 
			time_++;
			if(time_ == 1)
			{
				save_image_into_sdcard(img);
				time_ = 0;
			}
			sem_post(&image_get_sem);		
			usleep(100000);			//100ms时CPU占用率为7.7%左右
		}
		else
		{
			wait_capture_on();
			for(cache_cnt = 0; cache_cnt<5; cache_cnt++)
			{
				cap >> img;
				usleep(30000);
			}
		}
	}
_exit:
	pthread_exit(NULL);
}


void clear_sample_states(void)
{
	sample.clearStates();
}

static void *image_identify_thread_func(void * arg)
{
	//thread_binding_cpu(NULL, IMAGE_JOB_CPU);
	
	//xyVision::GetTarget sample("config.ini");
	while(1)
	{	
		sem_wait(&image_get_sem);						//等待capture获取到图像
#if 1
		sample << img;									//identify
		if(!sample.isDetected )
			goto pre_restart;

		cout << sample.getCurrentLoc() << endl;

		set_offset_data(sample.getCurrentLoc());
		//tell_external_offset_is_available();
#endif
pre_restart:
		sem_post(&image_handle_sem);
	}
	pthread_exit(NULL);
}

int setup_sem(void)
{
	int ret;

	ret = sem_init(&capture_start_sem, 0, 0);
	ret += sem_init(&image_get_sem, 0, 0);
	ret += sem_init(&image_handle_sem, 0, 1);
	//ret += sem_init(&offset_get_sem, 0, 0);
	return ret;
}

int ctreate_capture_and_identify_thread(void)
{
	int ret;

	ret = setup_sem();
	if(ret != 0)
	{
		printf("semaphore initialization failed.\n");
		return -1;
	}
#if 1
	if( mk_image_store_dir() < 0)
	{
		printf("Create Image Dir Error.\n");
		//return -1;
	}
#endif

	if(XY_Create_Thread(capture_thread_func, NULL, THREADS_CAPTURE, -1, SCHED_RR, 92) < 0)
	{
		printf("Create Capture Thread Error.\n");
		return -1;
	}

	if(XY_Create_Thread(image_identify_thread_func, NULL, THREADS_IDENTIFY, -1, SCHED_RR, 91) < 0)
	{
		printf("Create Image Identify Thread Error.\n");
		return -1;
	}

	return 0;

#if 0
	pthread_t tid;
	ret = pthread_create(&tid, 0,capture_thread_func, NULL);
	if(ret != 0)
	{
		return -1;
	}
	ret = pthread_create(&tid, 0,image_identify_thread_func, NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
#endif 
}

int XY_Capture_Setup(void)
{
	return ctreate_capture_and_identify_thread();
}

