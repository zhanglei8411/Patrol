#ifndef __CONTROL_LAW_H__
#define __CONTROL_LAW_H__

#include <math.h>
#include <stdio.h>
#include "../DJI_LIB/DJI_Pro_App.h"
#include "wireless_debug.h"
#include "route.h"
#include "common/common.h"
#include "image_identify.h"


#define DT 								(0.02)	//the period, for integration
#define PI								(3.1415926)

/*move xy para*/
#define HOVER_POINT_RANGE 				(0.1)	//FP use
#define HOVER_VELOCITY_MIN 				(0.1)	//FP use
#define TRANS_TO_HOVER_DIS 				(13.0) 	// for P2P trans to FP
#define P2P_MAX_ATTITUDE				(10.0)
#define P2P_MAX_VEL_N_E					(3.0)

/*image para*/
#define DIS_DIFF_WITH_MARK				(0.10)	//image FP & image down use
#define DIS_DIFF_WITH_MARK_HOVER		(1.5)	//hover to get close to the image, due to the high height, the dis is large
#define MIN_VEL_TO_GET_IMAGE			(0.3)	//the limit for image to use
#define MIN_ANGLE_TO_GET_IMAGE  		(4.0)	
#define MAX_EACH_DIS_IMAGE_GET_CLOSE	(3.0)	//each time the image target dis limit
#define MAX_CTRL_VEL_UPDOWN_WITH_IMAGE	(0.35)	//change from 0.5 by zhanglei night 0114
#define CAM_INSTALL_DELTA_X				(0.0)	//m,add to offset camera x, down to see drone, x is right 
#define CAM_INSTALL_DELTA_Y				(0.198)	//m,add to offset camera y, down to see drone, y is down, the drone center is y+
#define GPS_HEALTH_GOOD					(6)

/*updown para*/
#define UPDOWN_LIMIT_VEL_HIGH			(1.5)	//for up to height without image at high height
#define UPDOWN_LIMIT_VEL_LOW			(0.35)	//for up to height without image at low height
#define	DOWN_LIMIT_VEL_WITH_IMAGE		(0.35)	//for image get down
#define UPDOWN_CTRL_KP					(0.2)	//for all height ctrl
#define HEIGHT_CTRL_DELTA_FOR_HIGH		(1.0)	//for high height use
#define HEIGHT_HIGH_ABOVE				(20.0)	//>20m is as high
#define HEIGHT_CTRL_DELTA_FOR_LOW		(0.25)	//for low height use



typedef XYZ Center_xyz;

typedef struct{
	double roll_deg;
	double pitch_deg;
	double yaw_deg;
}Body_Angle;



int XY_Cal_Attitude_Ctrl_Data_Down_To_Height_WithVel_IMAGE(	api_vel_data_t cvel, 
														api_pos_data_t cpos,
														api_pos_data_t _focus_point,
														float target_vel, 
														float t_height,
														attitude_data_t *puser_ctrl_data,
														int *flag);

int XY_Cal_Attitude_Ctrl_Data_Up_To_Height_WithVel(	api_vel_data_t cvel, 
														api_pos_data_t cpos,
														api_pos_data_t _focus_point,
														float target_vel, 
														float t_height,
														attitude_data_t *puser_ctrl_data,
														int *flag);

int XY_Cal_Attitude_Ctrl_Data_UpDown_TO_Height(	api_vel_data_t cvel,
												api_pos_data_t cpos,
												api_pos_data_t _focus_point,
												float t_height,
												attitude_data_t *puser_ctrl_data,
												int *flag);

int XY_Cal_Attitude_Ctrl_Data_P2P(	api_vel_data_t cvel, 
									api_pos_data_t cpos, 
									float height, 
									Leg_Node *p_legn,
									attitude_data_t *puser_ctrl_data,
									int *flag);

int XY_Cal_Vel_Ctrl_Data_FP(api_vel_data_t cvel, api_pos_data_t cpos, api_pos_data_t spos, api_pos_data_t tpos, float height, attitude_data_t *puser_ctrl_data);
int XY_Cal_Vel_Ctrl_Data_FP_JUST_P2P(api_vel_data_t cvel, api_pos_data_t cpos, api_pos_data_t spos, api_pos_data_t tpos, float height, attitude_data_t *puser_ctrl_data);
void geo2XYZ(api_pos_data_t pos, XYZ *pXYZ);
void init_g_origin_pos(api_pos_data_t *_g_origin_pos);
void XYZ2xyz(api_pos_data_t s_pos, XYZ pXYZ, Center_xyz*pxyz);
void QUA2ANGLE(api_quaternion_data_t cur_quaternion, Body_Angle *body_angle) ;

int XY_Get_and_Print_Image_Data(void);
void XY_Cal_Vel_Ctrl_Data_FP_With_IMAGE(api_vel_data_t cvel, api_pos_data_t cpos, float height, attitude_data_t *puser_ctrl_data,int *flag);
void XY_Cal_Vel_Ctrl_Data_Get_Down_FP_With_IMAGE(api_vel_data_t cvel, api_pos_data_t cpos, attitude_data_t *puser_ctrl_data);




#endif

