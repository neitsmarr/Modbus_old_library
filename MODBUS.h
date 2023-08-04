#ifndef __MODBUS_H
#define __MODBUS_H

#include "main.h"

/*DEFINE NUMBER OF REGISTERS*/
#define I_REG_COUNT				10	//number of the input registers
#define H_REG_COUNT				60	//number of the holding registers
#define H_REG_HIDDEN			10	//number of last holding registers that cannot be overwritten with default value (for calibration and etc.)
#define S_REG_COUNT				11	//number of the special registers (the first 11 are used to keep UID and PID)

/*MODBUS LIBRARY SETTINGS*/
#define UPDATE_HW_VERSION			0		//update HW version after default values of HR4-HR6 were changed: 0=OFF, 1=ON

/*FUNCTIONS THAT CAN BE USED IN OTHER MODULES*/
void MBL_Init_Modbus(UART_HandleTypeDef *huart);	//call this function in main.c after initialisation of all hardware
void MBL_Check_For_Request(void);
void MBL_Rewrite_Register(uint16_t register_number, uint16_t reg_data);	//call this function to overwrite HR value in uint_hold_reg[] and EEPROM
void MBL_Inc_Tick(void);	//call this function inside SysTick_Handler
void MBL_Switch_DE_Callback(uint8_t state);	//weak ref, can be defined in other modules. state variants: 0=reset_DERE, 1=set_DERE
uint8_t MBL_Check_Restrictions_Callback(uint16_t register_address, uint16_t register_data);	//weak ref, can be defined in other modules. return 0 when OK, return 1 when NOK
void MBL_Register_Update_Callback(uint16_t register_address, uint16_t register_data);


/*BUFFERS AND FLAGS THAT CAN BE USED IN OTHER MODULES [READ-ONLY]*/
/*buffers*/
extern uint16_t uint_input_reg[I_REG_COUNT];	//input registers	//TODO union signed/unsigned
extern uint16_t uint_hold_reg[H_REG_COUNT];	//holding registers
extern uint16_t uint_spec_reg[S_REG_COUNT];	//special registers
/*flags*/
extern uint8_t flg_modbus_no_comm;	//raises after uint_hold_reg[7] seconds
extern uint8_t flg_modbus_packet_received;




struct structHRVA
{
	uint16_t virtualAddress;
	uint8_t RW;
	uint8_t signedUnsigned;
	uint16_t Minimum;
	uint16_t Maximum;
	uint16_t DefaultValue;
};

/*example of definition of holding registers*/
//const struct structHRVA RegVirtAddr[H_REG_COUNT] =		// 0-RW, 1-RO, 2-NA
//{//		addr	r/w		sgn 	min 	max 	def
//	{0xA001,	0,		0,		1,		247,	1,		},	//1		Device slave address
//	{0xA002,	0,		0,		0,		6,		2		},	//2		Modbus bound rate
//	{0xA003,	0,		0,		0,		2,		1		},	//3		Modbus parity
//	{0xA004,	1,		0,		0,		0,		DEV_TYPE},	//4		Device type
//	{0xA005,	1,		0,		0,		0,		DEV_HW	},	//5		HW version
//	{0xA006,	1,		0,		0,		0,		DEV_FW	},	//6		FW version
//	{0xA000,	2,		0,		0,		0,		0		},	//7		NA
//	{0xA008,	0,		0,		0,		60,		0		},	//8		Modbus safety timeout
//	{0xA009,	0,		0,		0,		1,		0		},	//9		NBT
//	{0xA00A,	0,		0,		0,		1,		0		},	//10	Modbus reset
//};

#endif
