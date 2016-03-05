#ifndef __UART_COMMON_H__
#define __UART_COMMON_H__

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/types.h>
#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>


#define DEFAULT_DEV_NAME "/dev/ttyTHS0"

int uart_common_open(const char *port_str, int *fd);
int uart_common_close(int *fd);
int uart_common_flush(int fd);
int uart_common_config(int fd, int baudrate, char data_bits, char parity_bits, char stop_bits);
int uart_common_start(const char *dev_name, int baud_rate, int *fd, fd_set *fdset);
int uart_common_easy_send(int fd);


#endif
