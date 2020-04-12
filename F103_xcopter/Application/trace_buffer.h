/*
 * trace_buffer.h
 *
 *  Created on: 10 kwi 2020
 *      Author: mate
 */

#ifndef TRACE_BUFFER_H_
#define TRACE_BUFFER_H_

//#define TRACE_BUFFER_LEN  7936 //256 * 31
#define TRACE_BUFFER_LEN  248 //8 * 31

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

typedef unsigned char bool;
#define true 1
#define false 0

typedef struct _TraceBuffer
{
	volatile uint8_t lock;
    uint16_t begin;
    uint16_t length;
    uint8_t payload[TRACE_BUFFER_LEN];
} TraceBuffer;

typedef struct _TraceRadioFrame
{
    uint8_t len; //how many bytes left
    uint8_t payload[31];

}TraceRadioFrame;

#define TRACE_BUFFER_OK 0
#define TRACE_BUFFER_FULL 1
#define TRACE_BUFFER_EMPTY 2
#define TRACE_BUFFER_LOCKED 3


int trace_buffer_push(char* buffer, int len);
int trace_buffer_pop(TraceRadioFrame *frame);

int trace_buffer_lock();
bool trace_buffer_is_locked();
void trace_buffer_unlock();



#endif /* TRACE_BUFFER_H_ */
