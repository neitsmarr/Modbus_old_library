/*MODBUS.c*/
#include "MODBUS.h"

#define PID_ADDRESS 				0x08001FF0

#define MODBUS_BUFFER_SIZE			0x100

/*Modbus function codes*/
enum function_code_e
{
	read_holding_registers = 0x03,
	read_input_registers = 0x04,
	write_single_register = 0x06,
	write_multiple_registers = 0x10,
	error = 0x80
};

/*init_default_values() parameters*/
enum
{
	seting_values = 0,
	communication_values = 1,
	all_values = 2
};

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

uint8_t (*Read_Dummy)(uint16_t, uint16_t*);
uint8_t (*Write_Dummy)(uint16_t, uint16_t);

/*FUNCTION PROTOTYPES*/
/*for internal use only*/
static void Check_HW_FW_Version(void);
static void Init_USART_DMA(void);
static void Update_Communication_Parameters(void);
static void Send_Response(uint8_t count);
static void Init_Default_Values(uint8_t values);
static void Check_Frame(void);
static void Process_Autoassignment_Request(struct response_s *response_s);
static uint8_t Detain_Autoasignment_Response(void);
static void Set_DE_Pin(void);
static void Reset_DE_Pin(void);
static void Set_NBT_Pin(void);
static void Reset_NBT_Pin(void);
static void Process_Request();
static void Update_Data(uint16_t register_number, uint16_t reg_data);
static void Check_Modbus_Registers(void);
static void Send_Exeption(uint8_t exeption_code);
static void Check_Communication_Reset_Jumper(void);
static void Check_Modbus_Timeout(void);
static uint16_t Calculate_CRC16(uint8_t *buf, uint16_t len);

//example of description:
/**
 * @brief  Start Receive operation in DMA mode.
 * @note   This function could be called by all HAL UART API providing reception in DMA mode.
 * @note   When calling this function, parameters validity is considered as already checked,
 *         i.e. Rx State, buffer address, ...
 *         UART Handle is assumed as Locked.
 * @param  huart UART handle.
 * @param  pData Pointer to data buffer (u8 or u16 data elements).
 * @param  Size  Amount of data elements (u8 or u16) to be received.
 * @retval HAL status
 */


/*PUBLIC FUNCTIONS*/
/**
 * @brief Initialize the Modbus according to the specified parameters in the UART_InitTypeDef.
 * @param huart UART handle.
 * @retval void (HAL status)
 */
void MBL_Init_Modbus(UART_HandleTypeDef *huart, void *read_handler, void *write_handler)
{
	uint8_t flg_init_eeprom = 0;
	uint16_t data;
	modbus_huart = huart;

	Read_Dummy = read_handler;
	Write_Dummy = write_handler;

	Init_USART_DMA();

	flg_init_eeprom = Read_Dummy(0, &data);
	if(flg_init_eeprom)	//check is this the first mcu startup
	{
		Init_Default_Values(all_values);
	}

	Check_Modbus_Registers();
	Check_HW_FW_Version();	//check if there is new FW version

	for (uint16_t i=0; i<H_REG_COUNT; i++)
	{
		if (RegVirtAddr[i].RW != 2)	//if the register is used
		{
			Read_Dummy(i, &uint_hold_reg[i]);
		}
	}

	//init NBT XXX test and optimize
	if(uint_hold_reg[8])
	{
		Set_NBT_Pin();
	}

	Update_Communication_Parameters();

	for(uint16_t i=0; i<6; i++)
	{
		uint_spec_reg[i] = *(uint16_t*) (UID_BASE + 2*i);	//Unique ID
	}
	for(uint16_t i=6; i<11; i++)
	{
		uint_spec_reg[i] = *(uint16_t*) (PID_ADDRESS + 2*(i-6));	//Production ID
	}
}

/**
 * @brief Check for the new received Modbus request.
 * @param none
 * @retval none
 */
void MBL_Check_For_Request(void)
{
	if(flg_modbus_packet_received)
	{
		flg_modbus_packet_received = 0;

		if(modbus_huart->ErrorCode == HAL_UART_ERROR_RTO)
		{
			len_modbus_frame = MODBUS_BUFFER_SIZE - modbus_huart->hdmarx->Instance->CNDTR;
			if(len_modbus_frame > 7)
			{
				Check_Frame();
			}
		}
		else
		{
			modbus_huart->ErrorCode = HAL_UART_ERROR_NONE;	//called in HAL_UART_Receive_DMA() / HAL_UART_Receive_DMA() function
		}

		HAL_UART_Receive_DMA(modbus_huart, buf_modbus, MODBUS_BUFFER_SIZE);
	}
}

/**
 * @brief Modbus clock. Should be called every 1ms.
 * @param none
 * @retval none
 */
void MBL_Inc_Tick(void)
{
	static uint16_t cnt_second;

	if(cnt_second < 1000)
	{
		cnt_second++;
	}
	else
	{
		Check_Communication_Reset_Jumper();
		Check_Modbus_Timeout();
	}

	if(cnt_autoassignment_delay > 0)
	{
		cnt_autoassignment_delay--;
	}
}

/**
 * @brief Update holding register value in EEPROM and in the buffer.
 * @param none
 * @retval none
 */
void MBL_Rewrite_Register(uint16_t register_number, uint16_t reg_data)
{
	Update_Data(register_number, reg_data);
}


/*CALLBACKS*/
/**
 * @brief This function is called every time when DE pin state changes.
 * @param none
 * @retval none
 */
__weak void MBL_Switch_DE_Callback(uint8_t state)
{
	UNUSED(state);	//can be ommited in C2X
}

/**
 * @brief This function is called every time when Modbus master tries to update holding register value.
 * @param none
 * @retval 0 = ok (new value is allowed), 1 = not ok (new value is not allowed)
 */
__weak uint8_t MBL_Check_Restrictions_Callback(uint16_t register_address, uint16_t register_data)
{
	UNUSED(register_address);
	UNUSED(register_data);
	return 0;
}

/**
 * @brief This function is called when holding register value has been updated.
 * @param none
 * @retval none
 */
__weak void MBL_Register_Update_Callback(uint16_t register_address, uint16_t register_data)
{
	UNUSED(register_address);
	UNUSED(register_data);
}


/*HAL CALLBACKS*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	flg_modbus_packet_received = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	Reset_DE_Pin();

	if(flg_reinit_modbus)	//XXX test it
	{
		flg_reinit_modbus = 0;
		Update_Communication_Parameters();
	}
}


/*PRIVATE FUNCTIONS*/
static uint16_t Calculate_CRC16(uint8_t *buf, uint16_t len)
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


static void Check_Frame(void)
{
	uint16_t crc_int;

	crc_int = (buf_modbus[len_modbus_frame-1]<<8) + buf_modbus[len_modbus_frame-2];	//get CRC16 bytes from the received packet

	if(crc_int == Calculate_CRC16(buf_modbus,(len_modbus_frame-2)))	// Check does the CRC match
	{
		if ((buf_modbus[0] == uint_hold_reg[0]) || buf_modbus[0] == 0x00)	//Check if the device address is correct
		{
			Process_Request();	// Return flag OK;
			flg_modbus_no_comm = 0;
			cnt_modbus_no_comm = 0;
		}
	}
}

static void Read_Input_Registers(struct response_s *response_s)
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

		crc16 = Calculate_CRC16(buf_modbus,3+buf_modbus[2]);
		buf_modbus[3+buf_modbus[2]] = crc16;	// CRC Lo byte
		buf_modbus[4+buf_modbus[2]] = crc16>>8;	// CRC Hi byte
	}

	response_s->frame_size = 5 + buf_modbus[2];
}

static void Read_Holding_Registers(struct response_s *response_s)
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

	crc16 = Calculate_CRC16(buf_modbus,3+buf_modbus[2]);
	buf_modbus[3+buf_modbus[2]] = crc16;	// CRC Lo byte
	buf_modbus[4+buf_modbus[2]] = crc16>>8;	// CRC Hi byte

	response_s->frame_size = 5 + buf_modbus[2];

	if(start_address == 0 && register_count == 4)
	{
		response_s->flg_response = 1;
	}
}

static void Write_Multiple_Registers(struct response_s *response_s)
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
							if(MBL_Check_Restrictions_Callback(i, reg_data))
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
							if(MBL_Check_Restrictions_Callback(i, reg_data))
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
			Update_Data(i, uint_hold_reg_temporary[i]);
		}
	}

	crc16 = Calculate_CRC16(buf_modbus,6);
	buf_modbus[6] = crc16;								// CRC Lo byte
	buf_modbus[7] = crc16>>8;							// CRC Hi byte
	response_s->frame_size = 8;

	if(start_address < 3)
	{
		flg_reinit_modbus = 1;
	}
	else if((start_address<=8) && (start_address+register_count>8))
	{
		if(uint_hold_reg[8]) Set_NBT_Pin();
		else Reset_NBT_Pin();
	}
	else if((start_address<=9) && (start_address+register_count>9))
	{
		Init_Default_Values(seting_values);
	}
}

static void Write_Single_Register(struct response_s *response_s)
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
					if(MBL_Check_Restrictions_Callback(start_address, reg_data))
					{
						response_s->exception = 0x03;
					}
					else
					{
						Update_Data(start_address, reg_data);
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
					if(MBL_Check_Restrictions_Callback(start_address, reg_data))
					{
						response_s->exception = 0x03;
					}
					else
					{
						Update_Data(start_address, reg_data);
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
	crc16 = Calculate_CRC16(buf_modbus,6);
	buf_modbus[6] = crc16;	//CRC Lo byte
	buf_modbus[7] = crc16>>8;	//CRC Hi byte
	response_s->frame_size = 8;

	if(start_address < 3)
	{
		flg_reinit_modbus = 1;
	}
	else if(start_address == 8)
	{
		if(uint_hold_reg[8]) Set_NBT_Pin();
		else Reset_NBT_Pin();
	}
	else if(start_address == 9)
	{
		Init_Default_Values(seting_values);
	}
}


static void Process_Request(void)
{
	struct response_s response_s = {0, 0, 0};

	if(buf_modbus[0])
	{
		response_s.flg_response = 1;
	}

	switch(buf_modbus[1])
	{
	case read_input_registers:
		Read_Input_Registers(&response_s);
		break;

	case read_holding_registers:
		Read_Holding_Registers(&response_s);
		break;

	case write_single_register:
		Write_Single_Register(&response_s);
		break;

	case write_multiple_registers:
		Write_Multiple_Registers(&response_s);
		break;

	case 103:	//GO TO AUTOASSIGNMENT MODE
	case 100:	//SEND RECOGNITION ANSWER
	case 101:	//CONFIRMATION STEP
	case 102:	//GET THE NEW ID
	case 104:	//LEAVE AUTOASSIGNMENT MODE
		Process_Autoassignment_Request(&response_s);
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
			Send_Response(response_s.frame_size);	// Send packet response
		}
	}
}

static void Send_Response(uint8_t count)
{
	Set_DE_Pin(); //Transmit mode
	//	HAL_UART_Transmit_IT(modbus_huart, buffer, count);
	HAL_UART_Transmit_DMA(modbus_huart, buf_modbus, count);
	//	HAL_Delay(1);
	//	HAL_UART_AbortReceive_IT(modbus_huart);
	//	HAL_Delay(1);
	//	HAL_UART_Receive_DMA(modbus_huart, buf_modbus, 0x100);
	//	HAL_Delay(1);
}

static void Send_Exeption(uint8_t exeption_code)
{
	uint16_t crc16;
	buf_modbus[0] = uint_hold_reg[0];	// Device address
	buf_modbus[1] += error;	// Modbus error code (0x80+command)
	buf_modbus[2] = exeption_code;	// exception code
	crc16 = Calculate_CRC16(buf_modbus,3);
	buf_modbus[3] = crc16;	// CRC Lo byte
	buf_modbus[4] = crc16>>8;	// CRC Hi byte
	Send_Response(5);	// Send packet response
}

static void Process_Autoassignment_Request(struct response_s *response_s)
{
	static uint8_t flg_autoassignment_status, flg_autoassignment_mode;

	uint16_t data;

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
				Read_Dummy(3, &a);
				buf_modbus[24] = a>>8;									// Device type Low byte
				buf_modbus[25] = a;										// Device type High byte
				crc16 = Calculate_CRC16(buf_modbus,26);
				buf_modbus[26] = crc16;									// CRC Low byte
				buf_modbus[27] = crc16>>8;								// CRC High byte

				//////////////////////// Random Delay generation and scanning ////////////////////////
				if(Detain_Autoasignment_Response() == 0)//w8 random delay and reply only if this is the first slave replying
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

			Read_Dummy(3, &data);
			if(r != data)
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

				Read_Dummy(3, &data);
				a = data;
				buf_modbus[24] = a>>8;									// Device type Low byte
				buf_modbus[25] = a;										// Device type High byte
				crc16 = Calculate_CRC16(buf_modbus,26);
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
				Update_Data(0, buf_modbus[8]);

				buf_modbus[0] = uint_hold_reg[0];						// Device address
				buf_modbus[1] = 102;									// Command
				buf_modbus[2] = 0x55;									// Dummy data
				buf_modbus[3] = 0x55;									// Dummy data
				crc16 = Calculate_CRC16(buf_modbus,4);
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

static void Init_Default_Values(uint8_t values)
{
	uint16_t start_register = 0;
	uint16_t end_register = 0;

	switch(values)
	{
	case all_values:
		start_register = 0;
		end_register = H_REG_COUNT/* - H_REG_HIDDEN*/;	//TODO H_REG_USED or nonhidden register count
		break;
	case communication_values:
		start_register = 0;
		end_register = 3;
		break;
	case seting_values:
		start_register = 9;
		end_register = H_REG_COUNT - H_REG_HIDDEN;	//TODO H_REG_USED
		break;
	}

	for(uint32_t i=start_register;i<end_register;i++)//For the first three registers (The communication registers according to the Sentera standard)
	{
		Update_Data(i, RegVirtAddr[i].DefaultValue);//write the default values
	}

	if(values) Update_Communication_Parameters();
}


static void Update_Communication_Parameters(void)
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


static void Init_USART_DMA(void)
{
	HAL_UART_ReceiverTimeout_Config(modbus_huart, 34);
	HAL_UART_EnableReceiverTimeout(modbus_huart);
	HAL_UART_Receive_DMA(modbus_huart, buf_modbus, 0x100);
}

static void Check_Modbus_Registers(void)	//UPDATED
{
	uint16_t reg_data;

	for(uint32_t i=0;i<H_REG_COUNT;i++)	//from the first to the last register
	{
		if(RegVirtAddr[i].RW == 0)
		{
			Read_Dummy(i, &reg_data);

			if(RegVirtAddr[i].signedUnsigned)	//signed
			{
				if(((int16_t)reg_data < (int16_t)RegVirtAddr[i].Minimum) || ((int16_t)RegVirtAddr[i].Maximum < (int16_t)reg_data))
				{
					Update_Data(i, RegVirtAddr[i].DefaultValue);		//Not OK = write default value
				}
			}
			else	//unigned
			{
				if((reg_data < RegVirtAddr[i].Minimum) || (RegVirtAddr[i].Maximum < reg_data))
				{
					Update_Data(i, RegVirtAddr[i].DefaultValue);//Not OK = write default value
				}
			}
		}
	}
}

static void Check_HW_FW_Version(void)
{
	uint16_t data;

	Read_Dummy(3, &data);
	if (data != RegVirtAddr[3].DefaultValue)  	//Check the Device type
	{
		Update_Data (3, RegVirtAddr[3].DefaultValue);  	//New device type
		Init_Default_Values(seting_values); // setting to default values if the device type is new
#if UPDATE_HW_VERSION
		Read_Dummy(4, &data);
		if (data != RegVirtAddr[4].DefaultValue)
		{
			Update_Data (4, RegVirtAddr[4].DefaultValue);	//New hardware version
		}
#endif
	}
	Read_Dummy(5, &data);
	if (data != RegVirtAddr[5].DefaultValue)
	{
		Update_Data (5, RegVirtAddr[5].DefaultValue);  	//New firmware version	//TODO how to add new HRs automatically?
	}
}


static void Check_Communication_Reset_Jumper(void)
{
	static uint8_t cnt_reset_communication_pressed, flg_reset_communication_completed;

	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14))
	{
		if(!flg_reset_communication_completed && (cnt_reset_communication_pressed == 50))
		{
			Init_Default_Values(communication_values);
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

static void Check_Modbus_Timeout(void)
{
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

static uint8_t Detain_Autoasignment_Response(void)
{
	uint16_t device_unique_value;

	device_unique_value = Calculate_CRC16((uint8_t*)UID_BASE, 12);

	cnt_autoassignment_delay =  device_unique_value & 0x3FF;	//XXX test it

	while(cnt_autoassignment_delay)	//scanning
	{
		//check UART RX pin
		if((USART1_RX_GPIO_Port->IDR&USART1_RX_Pin) == 0)	//XXX test it
		{
			return 1;
		}
	}
	return 0;
}

static void Update_Data(uint16_t register_number, uint16_t reg_data)
{
	Write_Dummy(RegVirtAddr[register_number].virtualAddress, reg_data);
	uint_hold_reg[register_number] = reg_data;
	MBL_Register_Update_Callback(register_number, reg_data);
}


static void Set_DE_Pin(void)
{
	USART1_DE_GPIO_Port->BSRR = USART1_DE_Pin;
	MBL_Switch_DE_Callback(1);
}

static void Reset_DE_Pin(void)
{
	USART1_DE_GPIO_Port->BRR = USART1_DE_Pin;
	MBL_Switch_DE_Callback(0);
}

static void Set_NBT_Pin(void)
{
	USART1_NBT_GPIO_Port->BSRR = USART1_NBT_Pin;
}

static void Reset_NBT_Pin(void)
{
	USART1_NBT_GPIO_Port->BRR = USART1_NBT_Pin;
}



