#ifndef __CONTROL_STEER_H__
#define __CONTROL_STEER_H__


#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>


/*
 * gpio158 - GPIO2_OUT(14)
 */
#define SYSFS_IO_EXPORT			"/sys/class/gpio/export"
#define SYSFS_IO_EXPORT_VAL		"158"
#define SYSFS_IO_DIR				"/sys/class/gpio/gpio158/direction"
#define SYSFS_IO_DIR_VAL			"out"
#define SYSFS_IO_VAL				"/sys/class/gpio/gpio158/value"
#define SYSFS_IO_EDGE				"/sys/class/gpio/gpio158/edge"
#define SYSFS_IO_EDGE_VAL			"rising"


int XY_Load_Goods(void);
int XY_Unload_Goods(void);


#endif
