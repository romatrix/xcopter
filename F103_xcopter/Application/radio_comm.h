# include "RFM70.h"

typedef struct _RadioFrame
{
	uint8_t len;
	uint8_t header; //x
	uint16_t payload[4];
} RadioFrame;


//typedef struct _TraceRadioFrame
//{
//	uint8_t counter; //how many messages left
//	uint16_t payload[31];
//
//}TraceRadioFrame;

#define WAIT_INFINITE 0xffffffff


void radio_comm_init(SpiPort *port);
//void radio_comm_master_loop();
//void radio_comm_slave_loop();
void radio_comm_send_frame(RadioFrame *frame);
void radio_comm_recv_frame(RadioFrame *frame);

void radio_comm_set_trace_via_radio(uint8_t s);
uint8_t radio_comm_get_trace_via_radio();
