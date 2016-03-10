#ifndef __SD_STORE_H__
#define __SD_STORE_H__


#include <time.h>
#include <sys/time.h> 
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <math.h>
#include "../DJI_LIB/DJI_Pro_App.h"
#include "image_identify.h"
#include "ultra_filter.h"

int XY_SD_Log_Setup(void);
void set_no_gps_z_data(float _no_gps_z);
void set_ctrl_data(attitude_data_t _ctrl_data);
int XY_Start_Store_Log(void);
int XY_Stop_Store_Log(void);
void set_log_offset_adjust(Offset _src_offset);


#endif
