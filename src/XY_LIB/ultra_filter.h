#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifndef ULTRA_FILTER_H
#define ULTRA_FILTER_H
//定义队列数据类型
typedef struct
{
	float* arr;//首地址
	int len; //数组大小
	int front;//首元素下标
	int cnt;//记录元素个数
}Queue;

Queue* create(int len);//创建队列
void destory(Queue* pq);//销毁队列
int empty(Queue* pq);//判断队列是否为空
int  ultra_queue_full(Queue* pq);//判断队列是否满
void push_queue(Queue* pq,float data);//入队
void travel(Queue* pq);//遍历
float queue_pop(Queue* pq);//出队
float get_head(Queue* pq);//获取首元素
float get_tail(Queue* pq);//获取队尾元素
int size(Queue* pq);//元素个数
int Get_calced_Ultra(Queue*pq,float* ultra);
void ultra_calc(Queue *pq);
void ultra_height_filter(Queue *pq,float data,float *filter_data);


#endif

