#ifndef __IMAGE_IDENTIFY_H__
#define __IMAGE_IDENTIFY_H__

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "getTargetLoc.hpp"
#include "opencv.hpp"
#include "../DJI_LIB/DJI_Pro_App.h"

#define OFFSET_GET_ID_A	0x02
#define OFFSET_GET_ID_B	0x04
/* #define OFFSET_GET_ID_*	0x** */

typedef struct
{
	float x;
	float y;
	float z;
}Offset;

typedef struct
{
	Offset 	_offset;
	int 	gotten;
}Offset_Data;

int mk_image_store_dir(void);
int get_current_cnt_in_profile(void);
int XY_Get_Offset_Data(Offset *_data, int _get_id);
int XY_Start_Capture(void);
int XY_Stop_Capture(void);
int XY_Capture_Setup(void);
void clear_sample_states(void);

#endif
