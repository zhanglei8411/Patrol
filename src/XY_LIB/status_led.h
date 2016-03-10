#ifndef __STATUS_LED_H__
#define __STATUS_LED_H__


#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdlib.h>


#define SYSFS_LED_EXPORT			"/sys/class/gpio/export"
#define SYSFS_LED_EXPORT_VAL		"157"
#define SYSFS_LED_DIR				"/sys/class/gpio/gpio157/direction"
#define SYSFS_LED_DIR_VAL			"out"
#define SYSFS_LED_VAL				"/sys/class/gpio/gpio157/value"
#define SYSFS_LED_EDGE				"/sys/class/gpio/gpio157/edge"
#define SYSFS_LED_EDGE_VAL			"rising"

int XY_Status_Led_Setup(void);
int init_led(int *fd);
void turn_on_led(int _fd);
void turn_off_led(int _fd);
void ioctl_led(int _new_cmd);

#endif
