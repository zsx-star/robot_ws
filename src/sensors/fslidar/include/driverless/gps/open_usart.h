#ifndef OPEN_USART_H
#define OPEN_USART_H

#include <fcntl.h>
#include<stdio.h>
#include<unistd.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#define TRUE 1
#define FALSE 0
#endif


int open_usart(const char *dev);
void set_speed(int fd, int speed);//decleration
int set_Parity(int fd, int databits, int stopbits, int parity);
