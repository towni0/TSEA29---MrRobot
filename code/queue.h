//quick implementation of queue for robot project (linked list)
#include <stdlib.h>

struct node{
	uint8_t orderdata;
	struct node *next;
};

struct queue{
		unsigned int size;
		node *front;
		node *back;

}orderQueue;

void queue_init(struct queue *toBeInit){
	toBeInit->front = 0;
	toBeInit->back = 0;
}

void enqueue(uint8_t order, struct queue *queue){
	
	//first item to be added
	if(queue->front == 0 && queue->back == 0){
		queue->back = (struct node *)malloc(1*sizeof(struct node));
		queue->back->next = 0;
		queue->back->orderdata = order;
		queue->front = queue->back;
	}
	//else add it to the back
	else{
		node *temp = (struct node *)malloc(1*sizeof(struct node));
		queue->back->next = temp;
		temp->orderdata = order;
		temp->next = 0;
		//update back
 		queue->back = temp;
	}
	//update back
	queue->size++;
}

void dequeue(struct queue *queue){
	node *temp = queue->front;
	if(queue->front == 0){
		//if it is empty do nothing
		return;
	}
	else{
		if (queue->front == queue->back){
			queue->front=0;
			queue->back=0;
		}
		else{
			queue->front = queue->front->next;
		}
		free(temp);
		queue->size--;
	}
}

unsigned int queuesize(struct queue *queue){
	return queue->size;
}

bool isempty(struct queue *queue){
	return queue->size;
}
