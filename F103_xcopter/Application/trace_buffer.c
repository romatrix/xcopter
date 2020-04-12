/*
 * trace_buffer.c
 *
 *  Created on: 10 kwi 2020
 *      Author: mate
 */

#include "trace_buffer.h"

TraceBuffer tb = {0};

uint16_t begin_pos(uint16_t offset)
{
    offset += tb.begin;

    if(offset >= sizeof(tb.payload))
        return offset - sizeof(tb.payload);
    else {
        return offset;
    }
}


uint16_t end_pos(uint16_t offset)
{
    offset += tb.begin + tb.length;

    if(offset >= sizeof(tb.payload))
        return offset - sizeof(tb.payload);
    else {
        return offset;
    }
}

int trace_buffer_push(char* buffer, int len)
{
    if(tb.length + len >= TRACE_BUFFER_LEN){
		return TRACE_BUFFER_FULL;
	}

	for(int i = 0; i < len; ++i){
        tb.payload[end_pos(i)] = buffer[i];
        if(buffer[i] == 0){
        	--len;
        }
	}

    tb.length += len;

	return TRACE_BUFFER_OK;
}


int trace_buffer_pop(TraceRadioFrame *frame)
{
    if(tb.length == 0){
		return TRACE_BUFFER_EMPTY;
	}

    int len = tb.length > sizeof(frame->payload) ? sizeof(frame->payload) : tb.length;

    for(int i = 0; i < len; ++i){
        frame->payload[i] = tb.payload[begin_pos(i)];
	}


    tb.begin = begin_pos(len);
    frame->len = tb.length;
    tb.length -= len;

	return TRACE_BUFFER_OK;
}

int trace_buffer_lock()
{
	if(1 == tb.lock){
		return TRACE_BUFFER_LOCKED;
	}

	tb.lock = 1;

	return TRACE_BUFFER_OK;
}

bool trace_buffer_is_locked()
{
	return tb.lock;
}

void trace_buffer_unlock()
{
	tb.lock = 0;
}

