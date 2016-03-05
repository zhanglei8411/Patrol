#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#ifndef ULTRA_FILTER_H
#define ULTRA_FILTER_H
//���������������
typedef struct
{
	float* arr;//�׵�ַ
	int len; //�����С
	int front;//��Ԫ���±�
	int cnt;//��¼Ԫ�ظ���
}Queue;

Queue* create(int len);//��������
void destory(Queue* pq);//���ٶ���
int empty(Queue* pq);//�ж϶����Ƿ�Ϊ��
int  ultra_queue_full(Queue* pq);//�ж϶����Ƿ���
void push_queue(Queue* pq,float data);//���
void travel(Queue* pq);//����
float queue_pop(Queue* pq);//����
float get_head(Queue* pq);//��ȡ��Ԫ��
float get_tail(Queue* pq);//��ȡ��βԪ��
int size(Queue* pq);//Ԫ�ظ���
int Get_calced_Ultra(Queue*pq,float* ultra);
void ultra_calc(Queue *pq);
void ultra_height_filter(Queue *pq,float data,float *filter_data);


#endif

