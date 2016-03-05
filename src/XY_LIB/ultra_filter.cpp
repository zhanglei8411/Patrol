
#include "ultra_filter.h"

Queue* create(int len)//��������
{
	Queue* pq=(Queue*)malloc(sizeof(Queue));
	if(NULL==pq)
	{
		printf("creat fail\n");
		return NULL;
	}
	pq->arr=(float*)malloc(sizeof(float)*len);
	if(NULL==pq->arr)
	{
		printf("malloc fail\n");
		return NULL;
	}
	pq->len=len;
	pq->front=0;
	pq->cnt=0;
	return pq;
}
void destory(Queue* pq)//���ٶ���
{
	free(pq->arr);
	pq->arr=NULL;
	free(pq);   
	pq=NULL;
}
int empty(Queue* pq)
{
	return 0==pq->cnt;
}
int ultra_queue_full(Queue* pq)
{
	return pq->len==pq->cnt;
}
void push_queue(Queue* pq,float data)//���
{
	if(ultra_queue_full(pq))
	{
		printf("queue is full,push %f fail\n",data);
		return;
	}
	pq->arr[(pq->front+pq->cnt)%pq->len]=data;
	++pq->cnt;
}
void travel(Queue* pq)//����
{
	printf("�����е�Ԫ����");
	int i=0;
	for(i=pq->front;i<pq->front+pq->cnt;i++)
	{
		printf("%f ",pq->arr[i%pq->len]);//��ֹ�±�Խ��
	}
	printf("\n");

}
float queue_pop(Queue* pq)//����
{
	if(empty(pq))
	{
		printf("queue is empty ,pop fail\n");
		return -1;
	}
	--pq->cnt;
       //printf("ultra_pop %.4f",pq->arr[(pq->front++)%pq->len]);
	return pq->arr[(pq->front++)%pq->len];
	
}
int Get_calced_Ultra(Queue*pq,float* ultra)
{
    float ultra_tmp=get_head(pq);
    if(ultra_tmp!=-1)
    {
        *ultra=ultra_tmp;
    }
    return 0;
}

float get_head(Queue* pq)//��ȡ��Ԫ��
{
	if(empty(pq))
	return -1;
	return pq->arr[(pq->front)%pq->len];
}

float get_tail(Queue* pq)//��ȡ��βԪ��
{
	if(empty(pq))
	return -1;
	return pq->arr[(pq->front+pq->cnt-1)%pq->len];
}
int size(Queue* pq)//Ԫ�ظ���
{
	return pq->cnt;
}

//�����׼��
static int STD_calc(float avg, Queue *pq)
{
     float variance=0;
     float stdev;
     variance=(pow((avg-pq->arr[(pq->front)%pq->len]),2)+ pow((avg-pq->arr[(pq->front+1)%pq->len]),2)+pow((avg-pq->arr[(pq->front+2)%pq->len]),2)+pow((avg-pq->arr[(pq->front+3)%pq->len]),2) )/4;
     stdev=sqrt(variance);
     if(stdev > 0.3)
     {
        return -1;
     }
     else
     { 
     	return 0;
     }
}
static void filter_hop(Queue *pq)
{
      
     // else if(ultra_index>0&&ultra_index<3)
      //{ 
              if( ((pq->arr[(pq->front)%pq->len]<=pq->arr[(pq->front+1)%pq->len])&&(pq->arr[(pq->front+1)%pq->len]<=pq->arr[(pq->front+2)%pq->len]))||
                  ((pq->arr[(pq->front)%pq->len]>=pq->arr[(pq->front+1)%pq->len])&&(pq->arr[(pq->front+1)%pq->len]>=pq->arr[(pq->front+2)%pq->len])))
              {
                     //ʲôҲ����
              }
              else
              {
                  pq->arr[(pq->front)%pq->len]=0;
              }
      //} 
}


void ultra_calc(Queue *pq)
{  
    float avg = (pq->arr[(pq->front)%pq->len] + pq->arr[(pq->front+1)%pq->len] + pq->arr[(pq->front+2)%pq->len] + pq->arr[(pq->front+3)%pq->len] ) / 4;
	if(STD_calc(avg,pq) == 0)
    {		
		float tmp = 0;
		tmp = abs(pq->arr[(pq->front)%pq->len] - avg);	
		if(tmp <= 0.2 && tmp >= 0)
		{
			//���������   
        	filter_hop(pq);
		}
		else
		{               
			pq->arr[(pq->front)%pq->len] = 0;
		}
    }
    else
	{
       pq->arr[(pq->front)%pq->len] = 0;
    }
    return;
}

void ultra_height_filter(Queue *pq,float data,float *filter_data)
{
	//printf("data is %.4f\n",data);
	if(!ultra_queue_full(pq))
   	{
        push_queue(pq,data);
        //printf("_log_ultra_data is %.4f\n",_log_ultra_data);
  	 }
   	if(ultra_queue_full(pq))
   	{
        ultra_calc(pq);
		Get_calced_Ultra(pq,filter_data);
		//printf("filter_data is %.4f\n",*filter_data);
        queue_pop(pq);                            
        //printf("_calc_ultra_data is %.4f\n",_calc_ultra_data);
   	}
}

