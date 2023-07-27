/*Modbus.c*/
#include "MODBUS.h"
#include "EEPROM.h"
#include <stdlib.h>

#define UID_ADDRESS					0x1FFFF7AC
#define PID_ADDRESS 				0x08001FF0

#define MODBUS_BUFFER_SIZE			256

/*Modbus function codes*/
#define READ_HOLDING_REGISTERS 		0x03
#define READ_INPUT_REGISTERS 		0x04
#define WRITE_SINGLE_REGISTER 		0x06
#define WRITE_MULTIPLE_REGISTERS 	0x10
#define MODBUS_ERROR_CODE			0x80

/*init_default_values() parameters*/
#define SET_VALUES 0
#define COMM_VALUES 1
#define ALL_VALUES 2

struct response_s {
	uint8_t exception;;
	uint8_t frame_size;
	uint8_t flg_response;
};


extern const struct structHRVA RegVirtAddr[H_REG_COUNT];

/*associated with the bootloader*/
volatile uint8_t __attribute__((section ("buf_section"))) buff_app_boot[0x10];
volatile uint8_t __attribute__((section ("puf_section"))) buff_puf[0x10];
volatile uint32_t __attribute__((section ("vectors_section"))) VectorTable[48];

/*VARIABLES*/
/*for internal and external usage*/
uint16_t uint_input_reg[I_REG_COUNT];
uint16_t uint_hold_reg[H_REG_COUNT];
uint16_t uint_spec_reg[S_REG_COUNT];
uint8_t flg_modbus_no_comm;
/*for internal usage only*/
UART_HandleTypeDef *modbus_huart;
uint16_t cnt_modbus_no_comm;
uint8_t len_modbus_frame;
uint8_t buf_modbus[MODBUS_BUFFER_SIZE];
uint8_t flg_modbus_packet_received;
uint8_t flg_reinit_modbus;
volatile uint16_t cnt_autoassignment_delay;
uint8_t flg_modbus_error;

/*FUNCTION PROTOTYPES*/
/*for internal use only*/
void HW_FW_version_check(void);
void USART_DMA_Init(void);
void Comm_Param_Update(void);
void Send_Response_Via_DMA(uint8_t count);
void Init_Default_Values(uint8_t values);
void check_frame(void);
void Autoassignment(struct response_s *response_s);
uint8_t Auto_asignment_scanning_delay(void);
void Set_DERE_pin(void);
void Reset_DERE_pin(void);
void Set_NBT_pin(void);
void Reset_NBT_pin(void);
void Process_Request();
void process_broadcast_request(uint8_t *buffer);
void EEPROM_Write_dummy(uint16_t register_number, uint16_t reg_data);
void Modbus_registers_check(void);
void Send_Exeption(uint8_t exeption_code);
uint16_t EEPROM_Read_dummy(uint16_t register_number);


/*Modbus CRC predefined values*/
static uint16_t CRC16(uint8_t *buf, uint16_t len)
{
	static const uint16_t crc_table[] = {
			0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
			0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
			0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
			0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
			0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
			0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
			0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
			0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
			0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
			0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
			0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
			0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
			0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
			0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
			0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
			0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
			0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
			0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
			0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
			0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
			0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
			0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
			0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
			0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
			0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
			0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
			0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
			0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
			0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
			0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
			0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
			0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

	uint8_t xor;
	uint16_t crc = 0xFFFF;

	while(len--)
	{
		xor = *buf++ ^ crc;
		crc >>= 8;
		crc ^= crc_table[xor];
	}

	return crc;
}

void MBL_Modbus(void)
{
	if(flg_modbus_packet_received)
//	if(modbus_huart->Instance->ISR & USART_ISR_RTOF)
	{
		flg_modbus_packet_received = 0;
//		modbus_huart->Instance->ICR = USART_ICR_RTOCF;
		if(!flg_modbus_error)
		{
			len_modbus_frame = MODBUS_BUFFER_SIZE - modbus_huart->hdmarx->Instance->CNDTR;
			if(len_modbus_frame > 7)
			{
				check_frame();
			}
		}
		else
		{
			flg_modbus_error = 0;
		}
		HAL_UART_AbortReceive_IT(modbus_huart);
		HAL_UART_Receive_DMA(modbus_huart, buf_modbus, 0x100);
//		modbus_huart->hdmarx->Instance->CCR &= ~DMA_CCR_EN;
//		modbus_huart->hdmarx->Instance->CNDTR = MODBUS_BUFFER_SIZE;
//		modbus_huart->hdmarx->Instance->CCR |= DMA_CCR_EN;
	}

//	if(modbus_huart->Instance->ISR & USART_ISR_TC)
//	{
//		modbus_huart->Instance->ICR = USART_ICR_TCCF;
//
//		Reset_DERE_pin();
//	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//	huart->Instance->ICR = 0xFFFFFFFF;
	if(flg_reinit_modbus)
	{
		flg_reinit_modbus = 0;
		Comm_Param_Update();
	}
	Reset_DERE_pin();
}

void check_frame(void)
{
	uint16_t crc_int;

	crc_int = (buf_modbus[len_modbus_frame-1]<<8) + buf_modbus[len_modbus_frame-2];	//get CRC16 bytes from the received packet

	if(crc_int == CRC16(buf_modbus,(len_modbus_frame-2)))	// Check does the CRC match
	{
		if ((buf_modbus[0] == uint_hold_reg[0]) || buf_modbus[0] == 0x00)	//Check if the device address is correct
		{
			Process_Request();	// Return flag OK;
			flg_modbus_no_comm = 0;
			cnt_modbus_no_comm = 0;
		}
	}
}

void Read_Input_Registers(struct response_s *response_s)
{
	uint16_t start_address  = (buf_modbus[2]<<8)+ buf_modbus[3];
	uint16_t register_count, crc16;

	register_count =  (buf_modbus[4]<<8)+ buf_modbus[5];
	buf_modbus[2] = register_count*2;	// byte count

	if(register_count + start_address > I_REG_COUNT)
	{
		response_s->exception = 0x02;
	}
	else
	{
		for(uint32_t i = start_address; i < start_address + register_count; i++)
		{
			buf_modbus[3+(i-start_address)*2] = (uint_input_reg[i]>>8);
			buf_modbus[4+(i-start_address)*2] = uint_input_reg[i];
		}

		crc16 = CRC16(buf_modbus,3+buf_modbus[2]);
		buf_modbus[3+buf_modbus[2]] = crc16;	// CRC Lo byte
		buf_modbus[4+buf_modbus[2]] = crc16>>8;	// CRC Hi byte
	}

	response_s->frame_size = 5 + buf_modbus[2];
}

void Read_Holding_Registers(struct response_s *response_s)
{
	uint16_t start_address  = (buf_modbus[2]<<8)+ buf_modbus[3];
	uint16_t register_count, crc16;

	register_count =  (buf_modbus[4]<<8)+ buf_modbus[5];
	buf_modbus[2] = register_count*2;	// byte count

	if(start_address + register_count < 1000)
	{
		if(register_count + start_address > H_REG_COUNT)
		{
			response_s->exception = 0x02;
		}
		else
		{
			for(uint32_t i = start_address; i < start_address + register_count; i++)
			{
				buf_modbus[3+(i-start_address)*2] = uint_hold_reg[i]>>8;
				buf_modbus[4+(i-start_address)*2] = uint_hold_reg[i];
			}
		}
	}
	else if (start_address > 998 && start_address < 1010)
	{
		if(start_address + register_count > 1010)
		{
			response_s->exception = 0x02;
		}
		else
		{
			for(uint32_t i = start_address; i < start_address + register_count; i++)
			{
				buf_modbus[3+(i-start_address)*2] = uint_spec_reg[i-999]>>8;
				buf_modbus[4+(i-start_address)*2] = uint_spec_reg[i-999];
			}
		}
	}

	crc16 = CRC16(buf_modbus,3+buf_modbus[2]);
	buf_modbus[3+buf_modbus[2]] = crc16;	// CRC Lo byte
	buf_modbus[4+buf_modbus[2]] = crc16>>8;	// CRC Hi byte

	response_s->frame_size = 5 + buf_modbus[2];

	if(start_address == 0 && register_count == 4)
	{
		response_s->flg_response = 1;
	}
}

void Write_Multiple_Registers(struct response_s *response_s)
{
	uint16_t start_address  = (buf_modbus[2]<<8)+ buf_modbus[3];
	uint16_t register_count, crc16;
	uint16_t reg_data;
	uint16_t uint_hold_reg_temporary[H_REG_COUNT] = {0};

	register_count =  (buf_modbus[4]<<8)+ buf_modbus[5];

	if(start_address < 1000)
	{
		if(register_count + start_address > H_REG_COUNT)
		{
			response_s->exception = 0x02;
		}
		else if(buf_modbus[6] != len_modbus_frame-9)	//buffer[6] - byte count: 7 bytes - header, 2 bytes - CRC
		{
			response_s->exception = 0x03;
		}
		else
		{
			for(uint32_t i = start_address; i < start_address + register_count; i++)
			{
				if(RegVirtAddr[i].RW == 0)
				{
					reg_data = (buf_modbus[7+(i-start_address)*2]<<8) + buf_modbus[8+(i-start_address)*2];

					if(RegVirtAddr[i].signedUnsigned)	//unsigned
					{
						if(((int16_t)RegVirtAddr[i].Minimum <= (int16_t)reg_data)&&((int16_t)reg_data <= (int16_t)RegVirtAddr[i].Maximum))
						{
							if(MBL_Check_Dynamic_Restrictions(i, reg_data))
							{
								response_s->exception = 0x03;
								break;
							}
							else
							{
								uint_hold_reg_temporary[i] = reg_data;
							}
						}
						else //exceptions when the data is outside of the limits
						{
							response_s->exception = 0x03;
							break;
						}
					}
					else	//signed
					{
						if((RegVirtAddr[i].Minimum <= reg_data)&&(reg_data <= RegVirtAddr[i].Maximum))
						{
							if(MBL_Check_Dynamic_Restrictions(i, reg_data))
							{
								response_s->exception = 0x03;
								break;
							}
							else
							{
								uint_hold_reg_temporary[i] = reg_data;
							}
						}
						else //exceptions when the data is outside of the limits
						{
							response_s->exception = 0x03;
							break;
						}
					}
				}
			}
			if(response_s->exception)
			{
//				break;	//from case:
			}
		}
	}

	for(uint32_t i = start_address; i < start_address + register_count; i++)//write the new data;
	{
		if(RegVirtAddr[i].RW == 0)
		{
			EEPROM_Write_dummy(i, uint_hold_reg_temporary[i]);
		}
	}

	crc16 = CRC16(buf_modbus,6);
	buf_modbus[6] = crc16;								// CRC Lo byte
	buf_modbus[7] = crc16>>8;							// CRC Hi byte
	response_s->frame_size = 8;

	if(start_address < 3)
	{
		flg_reinit_modbus = 1;
	}
	else if((start_address<=8) && (start_address+register_count>8))
	{
		if(uint_hold_reg[8]) Set_NBT_pin();
		else Reset_NBT_pin();
	}
	else if((start_address<=9) && (start_address+register_count>9))
	{
		Init_Default_Values(SET_VALUES);
	}
}

void Write_Single_Register(struct response_s *response_s)
{
	uint16_t start_address  = (buf_modbus[2]<<8)+ buf_modbus[3];
	uint16_t crc16;
	uint16_t reg_data;

	reg_data = (buf_modbus[4]<<8)+ buf_modbus[5];

	if(start_address == 989 && reg_data == 1338)//go to BL
	{
		for(uint32_t i=0;i<0x10;i++)
		{
			buff_app_boot[i] = 138;
		}
		HAL_NVIC_SystemReset();
	}
	else
	{
		if(start_address >= H_REG_COUNT)
		{
			response_s->exception = 0x02;
		}
		if(RegVirtAddr[start_address].RW == 0)//is register writable
		{
			if(RegVirtAddr[start_address].signedUnsigned)	//signed
			{
				if(((int16_t)RegVirtAddr[start_address].Minimum <= (int16_t)reg_data)&&((int16_t)reg_data <= (int16_t)RegVirtAddr[start_address].Maximum))//is the data in the limits
				{
					if(MBL_Check_Dynamic_Restrictions(start_address, reg_data))
					{
						response_s->exception = 0x03;
					}
					else
					{
						EEPROM_Write_dummy(start_address, reg_data);
					}
				}
				else //exceptions when the data is outside of the limits
				{
					response_s->exception = 0x03;
				}
			}
			else	//unsigned
			{
				if((RegVirtAddr[start_address].Minimum <= reg_data)&&(reg_data <= RegVirtAddr[start_address].Maximum))//is the data in the limits
				{
					if(MBL_Check_Dynamic_Restrictions(start_address, reg_data))
					{
						response_s->exception = 0x03;
					}
					else
					{
						EEPROM_Write_dummy(start_address, reg_data);
					}
				}
				else //exceptions when the data is outside of the limits
				{
					response_s->exception = 0x03;
				}
			}
		}
	}

	buf_modbus[4] = reg_data>>8;	//Register value 1st byte
	buf_modbus[5] = reg_data;	//Register value 2nd byte
	crc16 = CRC16(buf_modbus,6);
	buf_modbus[6] = crc16;	//CRC Lo byte
	buf_modbus[7] = crc16>>8;	//CRC Hi byte
	response_s->frame_size = 8;

	if(start_address < 3)
	{
		flg_reinit_modbus = 1;
	}
	else if(start_address == 8)
	{
		if(uint_hold_reg[8]) Set_NBT_pin();
		else Reset_NBT_pin();
	}
	else if(start_address == 9)
	{
		Init_Default_Values(SET_VALUES);
	}
}


void Process_Request()
{
	struct response_s response_s = {0, 0, 0};

	if(buf_modbus[0])
	{
		response_s.flg_response = 1;
	}

	switch(buf_modbus[1])
	{
	case READ_INPUT_REGISTERS:
		Read_Input_Registers(&response_s);
		break;

	case READ_HOLDING_REGISTERS:
		Read_Holding_Registers(&response_s);
		break;

	case WRITE_SINGLE_REGISTER:
		Write_Single_Register(&response_s);
		break;

	case WRITE_MULTIPLE_REGISTERS:
		Write_Multiple_Registers(&response_s);
		break;

	case 103:	//GO TO AUTOASSIGNMENT MODE
	case 100:	//SEND RECOGNITION ANSWER
	case 101:	//CONFIRMATION STEP
	case 102:	//GET THE NEW ID
	case 104:	//LEAVE AUTOASSIGNMENT MODE
		Autoassignment(&response_s);
		break;

	default:	//if the command is not supported
		response_s.exception = 0x01;
	}

	if(response_s.flg_response)
	{
		if(response_s.exception)
		{
			Send_Exeption(response_s.exception);
		}
		else
		{
			Send_Response_Via_DMA(response_s.frame_size);	// Send packet response
		}
	}
}

void Send_Response_Via_DMA(uint8_t count)
{
	Set_DERE_pin(); //Transmit mode
	//	HAL_UART_Transmit_IT(modbus_huart, buffer, count);
	HAL_UART_Transmit_DMA(modbus_huart, buf_modbus, count);
	//	HAL_Delay(1);
	//	HAL_UART_AbortReceive_IT(modbus_huart);
	//	HAL_Delay(1);
	//	HAL_UART_Receive_DMA(modbus_huart, buf_modbus, 0x100);
	//	HAL_Delay(1);
}

void Send_Exeption(uint8_t exeption_code)
{
	uint16_t crc16;
	buf_modbus[0] = uint_hold_reg[0];	// Device address
	buf_modbus[1] += MODBUS_ERROR_CODE;	// Modbus error code (0x80+command)
	buf_modbus[2] = exeption_code;	// exception code
	crc16 = CRC16(buf_modbus,3);
	buf_modbus[3] = crc16;	// CRC Lo byte
	buf_modbus[4] = crc16>>8;	// CRC Hi byte
	Send_Response_Via_DMA(5);	// Send packet response
}

void Autoassignment(struct response_s *response_s)
{
	static uint8_t flg_autoassignment_status, flg_autoassignment_mode;

	uint16_t a, r, crc16;

	switch (buf_modbus[1])
	{
	case 103:	//GO TO AUTOASSIGNMENT MODE
		flg_autoassignment_mode = 1;
		flg_autoassignment_status = 100;
		break;
	case 100:  //SEND RECOGNITION ANSWER
		if((flg_autoassignment_status == 101)&&(flg_autoassignment_mode == 1))
		{
			flg_autoassignment_status = 100;
		}
		if((flg_autoassignment_status == 100)&&(flg_autoassignment_mode == 1))
		{
			if((buf_modbus[2] == 0xAA)&&(buf_modbus[3] == 0xAA)&&(buf_modbus[4] == 0xAA)&&(buf_modbus[5] == 0xAA))
			{
				buf_modbus[0] = uint_hold_reg[0];						// Device address
				buf_modbus[1] = 100;										// Command
				for(uint32_t i = 0; i < 11; i++)					// unique ID and Production ID
				{
					buf_modbus[2+2*i] = uint_spec_reg[i]>>8;
					buf_modbus[2+2*i+1] = uint_spec_reg[i];
				}
				a = EEPROM_Read_dummy(3);
				buf_modbus[24] = a>>8;									// Device type Low byte
				buf_modbus[25] = a;										// Device type High byte
				crc16 = CRC16(buf_modbus,26);
				buf_modbus[26] = crc16;									// CRC Low byte
				buf_modbus[27] = crc16>>8;								// CRC High byte

				//////////////////////// Random Delay generation and scanning ////////////////////////
				if(Auto_asignment_scanning_delay() == 0)//w8 random delay and reply only if this is the first slave replying
				{
					//					send_response_via_DMA(buf_modbus, 28);								// Send response packet
					flg_autoassignment_status = 101;
					response_s->frame_size = 28;

				}
			}
		}
		break;

	case 101:  //CONFIRMATION STEP
		if((flg_autoassignment_status == 101)&&(flg_autoassignment_mode == 1))//is this controller in this mode?
		{
			///////////Compare the  unique ID and Production ID
			uint16_t Special_registers_compare[11];
			for(uint32_t i = 0; i < 11; i++)
			{
				Special_registers_compare[i] = buf_modbus[7+2*i];
				Special_registers_compare[i] <<= 8;
				Special_registers_compare[i] += buf_modbus[7+2*i+1];
				if(uint_spec_reg[i] != Special_registers_compare[i])
				{
					flg_autoassignment_status = 100;
				}
			}
			//////////////Compare the Device Type
			r = buf_modbus[29];
			r <<= 8;
			r += buf_modbus[30];
			if(r != EEPROM_Read_dummy(3))
			{
				flg_autoassignment_status = 100;
			}

			//////////////If everything is the same reply
			if(flg_autoassignment_status == 101)
			{
				buf_modbus[0] = uint_hold_reg[0];						// Device address
				buf_modbus[1] = 101;										// Command
				for(uint32_t i = 0; i < 11; i++)// 							unique ID and Production ID
				{
					buf_modbus[2+2*i] = uint_spec_reg[i]>>8;
					buf_modbus[2+2*i+1] = uint_spec_reg[i];
				}
				a = EEPROM_Read_dummy(3);
				buf_modbus[24] = a>>8;									// Device type Low byte
				buf_modbus[25] = a;										// Device type High byte
				crc16 = CRC16(buf_modbus,26);
				buf_modbus[26] = crc16;									// CRC Low byte
				buf_modbus[27] = crc16>>8;								// CRC High byte
				//				send_response_via_DMA(buf_modbus, 28);								// Send response packet
				flg_autoassignment_status = 102;
				response_s->frame_size = 28;
			}
		}
		break;

	case 102:	//GET THE NEW ID
		if((flg_autoassignment_status == 102)&&(flg_autoassignment_mode == 1))	//is this controller in this mode?
		{
			///////////Compare the  unique ID and Production ID
			uint16_t Special_registers_compare[11];
			uint8_t flg_another_controller_addressed = 0;
			for(uint32_t i = 0; i < 11; i++)
			{
				Special_registers_compare[i] = buf_modbus[9+2*i];
				Special_registers_compare[i] <<= 8;
				Special_registers_compare[i] += buf_modbus[9+2*i+1];
				if(uint_spec_reg[i] != Special_registers_compare[i])
				{
					flg_another_controller_addressed = 1;
				}
			}
			//////////////Compare the Device Type
			r = buf_modbus[31];
			r <<= 8;
			r += buf_modbus[32];
			if(r != uint_hold_reg[3])
			{
				flg_another_controller_addressed = 1;
			}

			//////////////If everything is the same get the new Slave ID and reply
			if(flg_another_controller_addressed == 0)
			{
				EEPROM_Write_dummy(0, buf_modbus[8]);

				buf_modbus[0] = uint_hold_reg[0];						// Device address
				buf_modbus[1] = 102;									// Command
				buf_modbus[2] = 0x55;									// Dummy data
				buf_modbus[3] = 0x55;									// Dummy data
				crc16 = CRC16(buf_modbus,4);
				buf_modbus[4] = crc16;									// CRC Low byte
				buf_modbus[5] = crc16>>8;								// CRC High byte
				//				send_response_via_DMA(buf_modbus, 6);					// Send response packet
				flg_autoassignment_status = 111;
				flg_autoassignment_mode = 0;
				response_s->frame_size = 6;
			}
		}
		break;

	case 104:	//LEAVE AUTOASSIGNMENT MODE
		flg_autoassignment_mode = 0;
		response_s->frame_size = 0;
		break;

	default:
		response_s->frame_size = 0;
	}
}

void Init_Default_Values(uint8_t values)
{
	uint16_t start_register = 0;
	uint16_t end_register = 0;

	switch(values)
	{
	case ALL_VALUES:
		start_register = 0;
		end_register = H_REG_COUNT/* - H_REG_HIDDEN*/;	//TODO H_REG_USED or nonhidden register count
		break;
	case COMM_VALUES:
		start_register = 0;
		end_register = 3;
		break;
	case SET_VALUES:
		start_register = 9;
		end_register = H_REG_COUNT - H_REG_HIDDEN;	//TODO H_REG_USED
		break;
	}

	for(uint32_t i=start_register;i<end_register;i++)//For the first three registers (The communication registers according to the Sentera standard)
	{
		EEPROM_Write_dummy(i, RegVirtAddr[i].DefaultValue);//write the default values
	}

	if(values) Comm_Param_Update();
}


void Comm_Param_Update(void)
{
	/*parity*/
	switch (uint_hold_reg[2]) {
	case 0:
		modbus_huart->Init.WordLength = UART_WORDLENGTH_8B;
		modbus_huart->Init.Parity = UART_PARITY_NONE;
		break; //none
	case 1:
		modbus_huart->Init.WordLength = UART_WORDLENGTH_9B;
		modbus_huart->Init.Parity = UART_PARITY_EVEN;
		break; // even
	case 2:
		modbus_huart->Init.WordLength = UART_WORDLENGTH_9B;
		modbus_huart->Init.Parity = UART_PARITY_ODD;
		break; //odd
	} //default is even parity

	switch (uint_hold_reg[1]) {
	case 0:
		modbus_huart->Init.BaudRate = 4800;
		HAL_UART_ReceiverTimeout_Config(modbus_huart, 39);
		break;
	case 1:
		modbus_huart->Init.BaudRate = 9600;
		HAL_UART_ReceiverTimeout_Config(modbus_huart, 39);
		break;
	case 2:
		modbus_huart->Init.BaudRate = 19200;
		HAL_UART_ReceiverTimeout_Config(modbus_huart, 39);
		break;
	case 3:
		modbus_huart->Init.BaudRate = 38400;
		HAL_UART_ReceiverTimeout_Config(modbus_huart, 67);
		break;
	case 4:
		modbus_huart->Init.BaudRate = 57600;
		HAL_UART_ReceiverTimeout_Config(modbus_huart, 101);
		break;
	case 5:
		modbus_huart->Init.BaudRate = 115200;
		HAL_UART_ReceiverTimeout_Config(modbus_huart, 202);
		break;
	case 6:
		modbus_huart->Init.BaudRate = 230400;
		HAL_UART_ReceiverTimeout_Config(modbus_huart, 403);
		break;
	} //default is 19200

	modbus_huart->Instance = USART1;
	modbus_huart->Init.StopBits = UART_STOPBITS_1;
	modbus_huart->Init.Mode = UART_MODE_TX_RX;
	modbus_huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	modbus_huart->Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(modbus_huart);
}

void MBL_Modbus_Init(UART_HandleTypeDef *huart)	//TODO init NBT?
{
	modbus_huart = huart;

	EE_Init();
	USART_DMA_Init();

	if(EEPROM_Read_dummy(0) == 0)	//check is this the first mcu startup
	{
		Init_Default_Values(ALL_VALUES);
	}

	Modbus_registers_check();
	HW_FW_version_check();	//check if there is new FW version

	for (uint16_t i=0; i<H_REG_COUNT; i++)
	{
		if (RegVirtAddr[i].RW != 2)	//if the register is used
		{
			uint_hold_reg[i] = EEPROM_Read_dummy(i);
		}
	}

	Comm_Param_Update();

	for(uint16_t i=0; i<6; i++)
	{
		uint_spec_reg[i] = *(uint16_t*) (UID_ADDRESS + 2*i);	//Unique ID
	}
	for(uint16_t i=6; i<11; i++)
	{
		uint_spec_reg[i] = *(uint16_t*) (PID_ADDRESS + 2*(i-6));	//Production ID
	}
}

void USART_DMA_Init(void)
{
	HAL_UART_ReceiverTimeout_Config(modbus_huart, 34);
	HAL_UART_EnableReceiverTimeout(modbus_huart);
	HAL_UART_Receive_DMA(modbus_huart, buf_modbus, 0x100);
}

void Modbus_registers_check(void)	//UPDATED
{
	uint16_t reg_data;

	for(uint32_t i=0;i<H_REG_COUNT;i++)	//from the first to the last register
	{
		if(RegVirtAddr[i].RW == 0)
		{
			reg_data = EEPROM_Read_dummy(i);

			if(RegVirtAddr[i].signedUnsigned)	//signed
			{
				if(((int16_t)reg_data < (int16_t)RegVirtAddr[i].Minimum) || ((int16_t)RegVirtAddr[i].Maximum < (int16_t)reg_data))
				{
					EEPROM_Write_dummy(i, RegVirtAddr[i].DefaultValue);		//Not OK = write default value
				}
			}
			else	//unigned
			{
				if((reg_data < RegVirtAddr[i].Minimum) || (RegVirtAddr[i].Maximum < reg_data))
				{
					EEPROM_Write_dummy(i, RegVirtAddr[i].DefaultValue);//Not OK = write default value
				}
			}
		}
	}
}

void HW_FW_version_check(void)
{
	if (EEPROM_Read_dummy(3) != RegVirtAddr[3].DefaultValue)  	//Check the Device type
	{
		EEPROM_Write_dummy (3, RegVirtAddr[3].DefaultValue);  	//New device type
		Init_Default_Values(SET_VALUES); // setting to default values if the device type is new
#if UPDATE_HW_VERSION
		if (EEPROM_Read_dummy(4) != RegVirtAddr[4].DefaultValue)
		{
			EEPROM_Write_dummy (4, RegVirtAddr[4].DefaultValue);	//New hardware version
		}
#endif
	}
	if (EEPROM_Read_dummy (5) != RegVirtAddr[5].DefaultValue)
	{
		EEPROM_Write_dummy (5, RegVirtAddr[5].DefaultValue);  	//New firmware version	//TODO how to add new HRs automatically?
	}
}


void Comm_Reset_Jumper_Check(void)
{
	static uint8_t cnt_100ms, cnt_reset_communication_pressed, flg_reset_communication_completed;

	if(cnt_100ms < 100) cnt_100ms++;
	else
	{
		cnt_100ms = 0;

		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14))
		{
			if(!flg_reset_communication_completed && (cnt_reset_communication_pressed == 50))
			{
				Init_Default_Values(COMM_VALUES);
				flg_reset_communication_completed = 1;
			}
			cnt_reset_communication_pressed++;
		}
		else
		{
			flg_reset_communication_completed = 0;
			cnt_reset_communication_pressed = 0;
		}
	}
}
/*
void Modbus_Timeout_Check()
{
	static uint16_t cnt_second, cnt_modbus_no_comm;

	if(cnt_second < 1000)
	{
		cnt_second++;
	}
	else
	{
		cnt_second = 0;

		if(flg_modbus_no_comm == 0)
		{
			if(cnt_modbus_no_comm < uint_hold_reg[7])
			{
				cnt_modbus_no_comm++;
			}
			else
			{
				if(uint_hold_reg[7] > 0) flg_modbus_no_comm = 1;
			}
		}
		else
		{
			cnt_modbus_no_comm = 0;
		}
	}
}
 */
void Modbus_Timeout_Check()
{
	static uint16_t cnt_second;

	if(cnt_second < 1000)
	{
		cnt_second++;
	}
	else
	{
		cnt_second = 0;

		if(uint_hold_reg[7] > 0)
		{
			if(flg_modbus_no_comm == 0)
			{
				if(cnt_modbus_no_comm < (uint_hold_reg[7] * 60))
				{
					cnt_modbus_no_comm++;
				}
				else
				{
					flg_modbus_no_comm = 1;
				}
			}
		}
	}
}

uint8_t Auto_asignment_scanning_delay(void)
{
	volatile uint16_t bbr = 0xffff;

	for(uint8_t i=0;i<0x10;i++)
	{
		bbr ^= buff_puf[i];
		for(uint8_t j=0;j<16;j++)
		{
			if((bbr&0x8000) == 0x8000)
			{
				bbr <<= 1;
				bbr ^= 0x1DB7;	//generating the seed from PUF ram buffer using CRC generation
			}
			else
			{
				bbr <<= 1;
			}
		}
	}
	srand(bbr);																	//seeding the seed
	//	uint64_t responce_delay = (uint64_t)((uint64_t)rand() * 1000) / RAND_MAX;	//transform the rand() value from 0 to 1000 ms
	cnt_autoassignment_delay = (uint16_t)((uint64_t)rand() * 1000UL / RAND_MAX);
	//	if(responce_delay > 1000) responce_delay = 1000; 							//check to not go over 1000ms
	//	cnt_responce_delay = responce_delay;										//load the cnt with the delay value
	//if(cnt_responce_delay > 0)cnt_responce_delay--;							//in interrupt at 1ms

	while(cnt_autoassignment_delay)///////////////scanning
	{
		//check UART RX pin
		if((GPIOA->IDR&GPIO_IDR_10) == 0)	//TODO make it dependent on modbus_huart
		{
			return 1;
		}
	}
	return 0;
}

uint16_t EEPROM_Read_dummy(uint16_t register_number)
{
	return EE_ReadVariable(RegVirtAddr[register_number].virtualAddress);
}

void EEPROM_Write_dummy(uint16_t register_number, uint16_t reg_data)
{
	uint16_t old_value = EEPROM_Read_dummy(register_number);

	if(reg_data != old_value)	//TODO optimise this workaround
	{
		EE_WriteVariable (RegVirtAddr[register_number].virtualAddress, reg_data);
	}

	uint_hold_reg[register_number] = reg_data;
	MBL_Register_Update_Callback(register_number, reg_data);
}

void MBL_Rewrite_Register(uint16_t register_number, uint16_t reg_data)
{
	EEPROM_Write_dummy(register_number, reg_data);
}

void Set_DERE_pin(void)
{
	USART1_DE_GPIO_Port->BSRR = USART1_DE_Pin;
	MBL_Switch_DE_Action(1);
}

void Reset_DERE_pin(void)
{
	USART1_DE_GPIO_Port->BRR = USART1_DE_Pin;
	MBL_Switch_DE_Action(0);
}

void Set_NBT_pin(void)
{
	USART1_NBT_GPIO_Port->BSRR = USART1_NBT_Pin;
}

void Reset_NBT_pin(void)
{
	USART1_NBT_GPIO_Port->BRR = USART1_NBT_Pin;
}

void MBL_Modbus_IRQHandler(void)
{
	if(modbus_huart->Instance->ISR & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE))
	{
		modbus_huart->Instance->ICR |= USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NCF;
		flg_modbus_error = 1;
	}
	if(modbus_huart->Instance->ISR & USART_ISR_RTOF)
	{
		modbus_huart->Instance->ICR |= USART_ICR_RTOCF;
		flg_modbus_packet_received = 1;
	}
	if ((modbus_huart->Instance->ISR & USART_ISR_TC) && (modbus_huart->Instance->CR1 & USART_CR1_TCIE))
	{
		ATOMIC_CLEAR_BIT(modbus_huart->Instance->CR1, USART_CR1_TCIE);
		modbus_huart->gState = HAL_UART_STATE_READY;
		modbus_huart->TxISR = NULL;
		HAL_UART_TxCpltCallback(modbus_huart);
	}
}

void MBL_Modbus_Inc_Tick(void)
{
	Comm_Reset_Jumper_Check();
	Modbus_Timeout_Check();
	if(cnt_autoassignment_delay > 0) cnt_autoassignment_delay--;
}

__weak void MBL_Switch_DE_Action(uint8_t state)
{

}

__weak uint8_t MBL_Check_Dynamic_Restrictions(uint16_t register_address, uint16_t register_data)
{
	return 0;
}

__weak void MBL_Register_Update_Callback(uint16_t register_address, uint16_t register_data)
{

}
