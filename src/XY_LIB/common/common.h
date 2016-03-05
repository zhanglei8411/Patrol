#ifndef __COMMON_H__
#define __COMMON_H__

typedef struct{
	double _longti;
	double _lati;
	double _alti;
	float _height;
}position;

typedef struct{
	double x;
	double y;
	double z;
}XYZ;

struct Leg
{
	unsigned int leg_seq;
	unsigned char leg_num;
	position start;
	position end;
	position current;
	position origin;
	XYZ _start;
	XYZ _end;
	XYZ _current;
};

typedef struct _Leg_Node
{
	struct Leg leg;
	_Leg_Node *next;
	_Leg_Node *prev;
}Leg_Node;

typedef struct
{
	volatile int obtained_control;
	volatile int activation;
	volatile int take_off;
	
}Aircraft_Status;



#endif
