#ifndef _SOLENOID_VALVES_H_
#define _SOLENOID_VALVES_H_

#include "main.h"
#include "spi.h"
#include "mcp2515.h"

#define YV1(dat) \
	{ \
			if (dat==1)		 {can_can_send_data[0] |= 0x01;}  \
			else if(dat==0){can_can_send_data[0] &= ~0x01;} \
			else				 	 {can_can_send_data[0] ^= 0x01;}	\
	}

#define YV2(dat) \
	{ \
			if (dat==1)		 {can_can_send_data[0] |= 0x02;}  \
			else if(dat==0){can_can_send_data[0] &= ~0x02;} \
			else				 	 {can_can_send_data[0] ^= 0x02;}	\
	}

#define YV3(dat) \
	{ \
			if (dat==1)		 {can_can_send_data[0] |= 0x04;}  \
			else if(dat==0){can_can_send_data[0] &= ~0x04;} \
			else				 	 {can_can_send_data[0] ^= 0x04;}	\
	}

#define YV4(dat) \
	{ \
			if (dat==1)		 {can_can_send_data[0] |= 0x08;}  \
			else if(dat==0){can_can_send_data[0] &= ~0x08;} \
			else				 	 {can_can_send_data[0] ^= 0x08;}	\
	}

#define YV5(dat) \
	{ \
			if (dat==1)		 {can_can_send_data[0] |= 0x10;}  \
			else if(dat==0){can_can_send_data[0] &= ~0x10;} \
			else				 	 {can_can_send_data[0] ^= 0x10;}	\
	}
	
#define YV6(dat) \
	{ \
			if (dat==1)		 {can_can_send_data[0] |= 0x20;}  \
			else if(dat==0){can_can_send_data[0] &= ~0x20;} \
			else				 	 {can_can_send_data[0] ^= 0x20;}	\
	}

#define YV7(dat) \
	{ \
			if (dat==1)		 {can_can_send_data[0] |= 0x40;}  \
			else if(dat==0){can_can_send_data[0] &= ~0x40;} \
			else				 	 {can_can_send_data[0] ^= 0x40;}	\
	}

#define YV8(dat) \
	{ \
			if (dat==1)		 {can_can_send_data[0] |= 0x80;}  \
			else if(dat==0){can_can_send_data[0] &= ~0x80;} \
			else				 	 {can_can_send_data[0] ^= 0x80;}	\
	}
	
#define YV9(dat) \
	{ \
			if (dat==1)		 {can_can_send_data[1] |= 0x01;}  \
			else if(dat==0){can_can_send_data[1] &= ~0x01;} \
			else				 	 {can_can_send_data[0] ^= 0x01;}	\
	}

#define YV10(dat) \
	{ \
			if (dat==1)		 {can_can_send_data[1] |= 0x02;}  \
			else if(dat==0){can_can_send_data[1] &= ~0x02;} \
			else				 	 {can_can_send_data[0] ^= 0x02;}	\
	}

	


//	电磁阀刷新控制函数
//	hcan	选择can口
	
void YV_flash(CAN_HandleTypeDef *hcan);
void YV_flash_mcp2515(MCP2515_HandleTypeDef *hcan);

extern uint8_t  can_can_send_data[8];

#endif

