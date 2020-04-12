/*
 * radio_comm.c
 *
 *  Created on: 28 mar 2020
 *      Author: mate
 */


#include "settings.h"
#include "RFM70.h"
#include "radio_comm.h"
#include "trace_buffer.h"

#define true 1
#define false 0

uint8_t trace_via_radio = 1;

void radio_comm_master_loop();


void radio_comm_init(SpiPort *port){
	RFM70_Initialize(port);
}

static void print_remote_traces()
{
	char print_buf[256] = {0};
	char recv_buf[256] = {0};
	uint16_t len = sizeof(recv_buf);
	int pos = 0;

	if(!Receive_Packet((u8*)recv_buf, &len, 100) || len > 0){
		for(int j = 0; j < len; ++j){
			char c = recv_buf[j + 1];
			if(c >= 31 || c == '\n'){
				print_buf[pos++] = c;
			}
		}
	} else {
		printf("[1] failed 1\n");
	}

	printf("RT: len=%d, data=<%s>\n", pos, print_buf);
}


void radio_comm_send_frame(RadioFrame *frame)
{
	Send_Packet(W_TX_PAYLOAD_NOACK_CMD, (u8*)frame, sizeof(RadioFrame));

	if(1 == trace_via_radio){
		RFM70_SwitchToRxMode();
		print_remote_traces();
		RFM70_SwitchToTxMode();
	}
}

static void send_traces_to_remote()
{
	TraceRadioFrame frame = {0};
	trace_buffer_lock();

	if(TRACE_BUFFER_OK != trace_buffer_pop(&frame)){
		frame.len = 1;
		Send_Packet(W_TX_PAYLOAD_NOACK_CMD, (u8*)&frame, 1);
	} else {
		do{
			int len = frame.len > sizeof(frame.payload) ? sizeof(TraceRadioFrame) : frame.len;
			Send_Packet(W_TX_PAYLOAD_NOACK_CMD, (u8*)&frame, len);

			char buf[33] = {0};
			snprintf(buf, len, "len=%d <%s>\n", frame.len, frame.payload);
			print(buf);
		}
		while(!trace_buffer_pop(&frame));
	}

	trace_buffer_unlock();
}

void radio_comm_recv_frame(RadioFrame *frame)
{
	uint16_t len = sizeof(RadioFrame);
	Receive_Packet((u8*)frame, &len, WAIT_INFINITE);

	if(1 == trace_via_radio){
		RFM70_SwitchToTxMode();
		send_traces_to_remote();
		RFM70_SwitchToRxMode();
	}
}


void radio_comm_set_trace_via_radio(uint8_t s)
{
	trace_via_radio = s;
}

uint8_t radio_comm_get_trace_via_radio()
{
	return trace_via_radio;
}
