#ifndef __ROUTE_H__
#define __ROUTE_H__

#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <time.h>
#include "../DJI_LIB/DJI_Pro_App.h"
#include "control_law.h"
#include "wireless_debug.h"
#include "common/common.h"
#include "control_steer.h"

/*-----velocity for up down------*/
/*deliver up*/
#define DELIVER_MAX_VEL_UP_TO_H2								(1.25)	
#define DELIVER_MIN_VEL_UP_TO_H2								(1.25)	//0220 (1.0 to 1.25)

#define DELIVER_MAX_VEL_UP_TO_H3								(1.75)
#define DELIVER_MIN_VEL_UP_TO_H3								(0.5)	//0220 (1.0 to 0.5)

/*deliver down*/
#define DELIVER_MAX_VEL_DOWN_TO_H1								(1.75)
#define DELIVER_MIN_VEL_DOWN_TO_H1								(0.2)	//0220 (0.4 to 0.2); ready to hover find image

/*down and drop*/
#define DELIVER_MAX_VEL_DOWN_TO_H2								(0.35)	//0220 (0.35 to 0.4)
#define DELIVER_MIN_VEL_DOWN_TO_H2								(0.35)	//0220 (0.35 to 0.4);01-27 (0.2 to 0.35)

/*delete*/
#define DELIVER_MAX_VEL_DOWN_TO_H3								(0.25)	// dele, no use
#define DELIVER_MIN_VEL_DOWN_TO_H3								(0.2)	//deke, no use

/*go back up */
#define GOBACK_MAX_VEL_UP_TO_H2									(1.0)
#define GOBACK_MIN_VEL_UP_TO_H2									(1.0)

#define GOBACK_MAX_VEL_UP_TO_H3									(1.75)
#define GOBACK_MIN_VEL_UP_TO_H3									(0.5)	//0220 (1.0 to 0.5)


/*go back down */
#define GOBACK_MAX_VEL_DOWN_TO_H1								(1.75)
#define GOBACK_MIN_VEL_DOWN_TO_H1								(0.2)	//0220 (0.5 to 0.2); ready to hover find image

#define GOBACK_MAX_VEL_DOWN_TO_H2								(0.5)	//0220 set o.5
#define GOBACK_MIN_VEL_DOWN_TO_H2								(0.5)	//0220 set o.5

#define GOBACK_MAX_VEL_DOWN_TO_H3								DELIVER_MAX_VEL_DOWN_TO_H3
#define GOBACK_MIN_VEL_DOWN_TO_H3								DELIVER_MIN_VEL_DOWN_TO_H3
/*-----velocity for up down------*/




/*----------Height define---------*/
/*Take off height correct*/
#define DIFF_HEIGHT_OF_MANUAL_PACK								(0.1)	
#define DIFF_HEIGHT_OF_AUTO_PACK								(0.52)
#define DIFF_HEIGHT_WHEN_TAKEOFF								(0)		// = TAKEOFF_HEIGHT - TARGET_HEIGHT				

#define DELIVER_HEIGHT_OF_UPH2										(10.0)	//m
#define DELIVER_HEIGHT_OF_UPH3										(35.0)
#define DELIVER_HEIGHT_OF_DOWNH1									(25.0 	- DIFF_HEIGHT_WHEN_TAKEOFF)

#define DELIVER_HEIGHT_OF_DOWNH2								(0.25)	//0218 (0.5 to 0.25); 01-23 (0.8 to 0.5)	01-25 (use image height no diff)
#define DELIVER_THRESHOLD_OF_DOWN_TO_H2_OUT						(0.15)	//01-23 (0.25 to 0.15)


#define DELIVER_HEIGHT_OF_DOWNH3									(0.5)

#define GOBACK_HEIGHT_OF_UPH2										(10.0	- DIFF_HEIGHT_WHEN_TAKEOFF)
#define GOBACK_HEIGHT_OF_UPH3										(35.0)
#define GOBACK_HEIGHT_OF_DOWNH1										(25.0)
#define GOBACK_HEIGHT_OF_DOWNH2										(0.3)	// 01-27 (1.5 to 0.3)
#define GOBACK_HEIGHT_OF_DOWNH3										(0.3)	// 01-27 delete the process of down to h3 

/*height diff*/
#define DELIVER_THRESHOLD_OF_UP_TO_H2_OUT						(1.0)		
#define DELIVER_THRESHOLD_OF_UP_TO_H3_OUT						(1.0)	
#define DELIVER_THRESHOLD_OF_DOWN_TO_H1_OUT						(1.0)	

#define DELIVER_THRESHOLD_OF_DOWN_TO_H3_OUT						(0.15)

#define GOBACK_THRESHOLD_OF_UP_TO_H2_OUT						(1.0)
#define GOBACK_THRESHOLD_OF_UP_TO_H3_OUT						(1.0)
#define GOBACK_THRESHOLD_OF_DOWN_TO_H1_OUT						(1.0)
#define GOBACK_THRESHOLD_OF_DOWN_TO_H2_OUT						(0.25)
#define GOBACK_THRESHOLD_OF_DOWN_TO_H3_OUT						(0.15)
/*----------Height define---------*/



/*----------Height control para define---------*/
#define DELIVER_UP_TO_H2_KPZ									(0.2)
#define DELIVER_UP_TO_H3_KPZ									(0.2)
#define DELIVER_DOWN_TO_H1_KPZ									(0.2)
#define DELIVER_DOWN_TO_H2_KPZ									(0.2)
#define DELIVER_DOWN_TO_H3_KPZ									(0.2)
	

#define GOBACK_UP_TO_H2_KPZ										(0.2)
#define GOBACK_UP_TO_H3_KPZ										(0.2)
#define GOBACK_DOWN_TO_H1_KPZ									(0.2)
#define GOBACK_DOWN_TO_H2_KPZ									(0.2)
#define GOBACK_DOWN_TO_H3_KPZ									(0.2)
/*----------Height control para define---------*/



/*-----Coordinations------*/
#define XYI_TERRACE_LONGTI						(2.09443030)//Google Earth
#define XYI_TERRACE_LATI						(0.52849836)//Google Earth
#define XYI_TERRACE_ALTI						(0.0)

#define ORIGIN_IN_HENGSHENG_LONGTI				(2.094421665) //Google EarthÌØÕ÷µã×ø±ê²ÎÊý,»¡¶È.9
#define ORIGIN_IN_HENGSHENG_LATI				(0.528485654) //Google EarthÌØÕ÷µã×ø±ê²ÎÊý,»¡¶È.9
#define ORIGIN_IN_HENGSHENG_ALTI				(0.0)

/* 1-15 test */
#define DELTA_X_M_GOOGLEEARTH					(-6.302767)   //m base the data in test on 0113-2
#define DELTA_Y_M_GOOGLEEARTH					(-0.504967)   //m	
#define DELTA_Z_M_GOOGLEEARTH					(-110.209)   //m	

/*-----Coordinations------*/




inline void set_leg_seq(struct Leg *_pleg, int _leg_seq)
{
	_pleg->leg_seq = _leg_seq;
}

inline void set_leg_num(struct Leg *_pleg, int _leg_num)
{
	_pleg->leg_num = _leg_num;
}

inline void set_pos(position *_pos, double _longti, double _lati, double _alti)
{
	_pos->_longti = _longti;
	_pos->_lati = _lati;
	_pos->_alti = _alti;
}

inline void set_xyz(XYZ *_xyz, double _x, double _y, double _z)
{
	_xyz->x = _x;
	_xyz->y = _y;
	_xyz->z = _z;
}

inline void set_leg_start_pos(struct Leg *_pleg, double _longti, double _lati, double _alti)
{
	set_pos(&_pleg->start, _longti, _lati, _alti);
}

inline void set_leg_end_pos(struct Leg *_pleg, double _longti, double _lati, double _alti)
{
	set_pos(&_pleg->end, _longti, _lati, _alti);
}

inline void set_leg_cur_pos(struct Leg *_pleg, double _longti, double _lati, double _alti)
{
	set_pos(&_pleg->current, _longti, _lati, _alti);
}

inline void set_leg_start_xyz(struct Leg *_pleg, double _x, double _y, double _z)
{
	set_xyz(&_pleg->_start, _x, _y, _z);
}

inline void set_leg_end_xyz(struct Leg *_pleg, double _x, double _y, double _z)
{
	set_xyz(&_pleg->_end, _x, _y, _z);
}

inline void set_leg_cur_xyz(struct Leg *_pleg, double _x, double _y, double _z)
{
	set_xyz(&_pleg->_current, _x, _y, _z);
}


int setup_route_list_head_node(Leg_Node *head);
int insert_new_leg_into_route_list(Leg_Node *head, struct Leg leg);
Leg_Node *create_head_node(void);
int add_leg_node(Leg_Node *_head, struct Leg _leg);
void delete_leg_list(Leg_Node *_head);
void XY_Release_List_Resource(Leg_Node *_head);
int update_cur_legn_data(double _longti, double _lati);
int temporary_init_deliver_route_list(void);
int temporary_init_goback_route_list(void);


int XY_Drone_Execute_Task(void);
int XY_Drone_Deliver_Task(void);
int XY_Drone_Goback_Task(void);
static void *drone_deliver_task_thread_func(void * arg);
static void *drone_deliver_up_thread_func(void * arg);
static void *drone_deliver_p2p_thread_func(void * arg);
static void *drone_deliver_down_thread_func(void * arg);


static void *drone_goback_task_thread_func(void * arg);
static void *drone_goback_up_thread_func(void * arg);
static void *drone_goback_p2p_thread_func(void * arg);
static void *drone_goback_down_thread_func(void * arg);



#endif
