/*
 * ReadSensorValues.c
 *
 *  Created on: Jun 4, 2020
 *      Author: Holly
 */
#include "main.h"
#include <stdlib.h>
#include <string.h>
#include "shtc3.h"
//#include "sht3x.h"
#include "MCP_Read_Write.h"
#include "HTS221_SPI.h"
#include "lis2dh12_reg.h"
#include "ReadSensorValues.h"

//configuration parameter falsh addreass
#define STM32_UUID ((uint32_t *)0x1FFFF7AC)

#define EVERAGE_TIME 8
uint32_t ADCRowval[EVERAGE_TIME][F_ROWS_PERTILE];
uint32_t AverageValue[F_ROWS_PERTILE];


volatile uint32_t 	ADC_BUF[F_ROWS_PERTILE];
volatile uint16_t 	SampleCount 			= 0;
volatile uint8_t 	isDataReady = 0;
//volatile uint32_t 	ADCval[F_ROWS_PERTILE]; // +2 for extra data come in
volatile uint32_t ADCval[EVERAGE_TIME][F_ROWS_PERTILE]; // +2 for extra data come in


float temperature_values[TOTAL_TH_SENSORS_PERTILE];
float humidity_values[TOTAL_TH_SENSORS_PERTILE];
uint32_t force_values[TOTAL_F_SENSORS_PERTILE];
float ref_temp, ref_hum;

uint8_t 		chip_id[15];
stmdev_ctx_t 	dev_ctx;
uint16_t 		Acce_X,Acce_Y,Acce_Z;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint32_t  DeviceUID;
uint8_t  CANDeviceID;

uint8_t TxData[8];
uint8_t RxData[8];

uint8_t 	FreeMailBox;

hts221_ctx_t hts221_ctx1;//[NUMBERS_OF_SPI];
hts221_ctx_t hts221_ctx2;//[NUMBERS_OF_SPI];
hts221_ctx_t hts221_ctx3;//[NUMBERS_OF_SPI];

shtc3_driver_t dev_ctxc3;

int8_t user_spi1_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	HAL_StatusTypeDef res;
	chip_select(id);

	res=HAL_SPI_Transmit(&hspi1,&reg_addr,1,1000);
	if(res==HAL_OK) res=HAL_SPI_Receive(&hspi1,data,len,1000);

	chip_unselect(id);
  return (int8_t)res;
}

int8_t user_spi1_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
 	HAL_StatusTypeDef res;
	chip_select(id);
	uint8_t* tmp_buf=malloc(len+1);
	tmp_buf[0]=reg_addr;
	memcpy(tmp_buf+1,data,len);
 	res=HAL_SPI_Transmit(&hspi1,tmp_buf,len+1,1000);
	free(tmp_buf);
	chip_unselect(id);
	return (int8_t)res;
}
int8_t user_spi2_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	HAL_StatusTypeDef res;
	chip_select(id);

	res=HAL_SPI_Transmit(&hspi2,&reg_addr,1,1000);
	if(res==HAL_OK) res=HAL_SPI_Receive(&hspi2,data,len,1000);

	chip_unselect(id);
  return (int8_t)res;
}

int8_t user_spi2_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
 	HAL_StatusTypeDef res;
	chip_select(id);
	uint8_t* tmp_buf=malloc(len+1);
	tmp_buf[0]=reg_addr;
	memcpy(tmp_buf+1,data,len);
 	res=HAL_SPI_Transmit(&hspi2,tmp_buf,len+1,1000);
	free(tmp_buf);
	chip_unselect(id);
  return (int8_t)res;
}

int8_t user_spi3_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
	HAL_StatusTypeDef res;
	chip_select(id);

	res=HAL_SPI_Transmit(&hspi3,&reg_addr,1,1000);
	if(res==HAL_OK) res=HAL_SPI_Receive(&hspi3,data,len,1000);

	chip_unselect(id);
  return (int8_t)res;
}

int8_t user_spi3_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
 	HAL_StatusTypeDef res;
	chip_select(id);
	uint8_t* tmp_buf=malloc(len+1);
	tmp_buf[0]=reg_addr;
	memcpy(tmp_buf+1,data,len);
 	res=HAL_SPI_Transmit(&hspi3,tmp_buf,len+1,1000);
	free(tmp_buf);
	chip_unselect(id);
  return (int8_t)res;
}


void send_sensor_value(CAN_HandleTypeDef *hcan, uint8_t sensID,uint8_t ReqType)
{
    uint32_t force=0;
    float   hmdt=1.0;
    float   tmpr=2.0;
    uint32_t TxMailbox[8];

    switch(ReqType)
    {
        case HUMIDITY_REQ:
			hmdt=humidity_values[sensID];
			TxHeader.ExtId=generate_can_header(CANDeviceID,sensID,ReqType,DATA_REPLY_MSG);
			memcpy(TxData,&hmdt,sizeof(hmdt));
			if ((FreeMailBox = HAL_CAN_GetTxMailboxesFreeLevel(hcan))>= 0)
			{
			  TxHeader.DLC=sizeof(hmdt);;
			  if( HAL_CAN_AddTxMessage(hcan,&TxHeader, (uint8_t *)TxData, TxMailbox)!= HAL_OK)
			  {
				Error_Handler();
			  }
			}
			break;

        case TEMPERATURE_REQ:
			tmpr=temperature_values[sensID];
			TxHeader.ExtId=generate_can_header(CANDeviceID,sensID,ReqType,DATA_REPLY_MSG);
			memcpy(TxData,&tmpr,sizeof(tmpr));

			if ((FreeMailBox = HAL_CAN_GetTxMailboxesFreeLevel(hcan))>= 0)
			{
			  TxHeader.DLC=sizeof(tmpr);
			  if( HAL_CAN_AddTxMessage(hcan,&TxHeader, (uint8_t *)TxData, TxMailbox)!= HAL_OK)
			  {
				Error_Handler();
			  }
			}
			break;

        case FORCE_REQ:
			force=force_values[sensID];
			TxHeader.ExtId=generate_can_header(CANDeviceID,sensID,ReqType,DATA_REPLY_MSG);
			memcpy(TxData,&force,sizeof(force));
			if ((FreeMailBox = HAL_CAN_GetTxMailboxesFreeLevel(hcan))>= 0)
			{
			  TxHeader.DLC=sizeof(force);
			  if( HAL_CAN_AddTxMessage(hcan,&TxHeader, (uint8_t *)TxData, TxMailbox)!= HAL_OK)
			  {
				Error_Handler();
			  }
			}
			break;

        case DEVICEID_REQ:
			TxHeader.ExtId=generate_can_header(CANDeviceID,0,ReqType,DEVICEID_REPLY_MSG);

			if ((FreeMailBox = HAL_CAN_GetTxMailboxesFreeLevel(hcan))>= 0)
			{
			  TxHeader.DLC=5;
			  if( HAL_CAN_AddTxMessage(hcan,&TxHeader, (uint8_t *)RxData, TxMailbox)!= HAL_OK)
			  {
				Error_Handler();
			  }
			}
        	break;

        case HUMIDITY_REF_REQ:
			TxHeader.ExtId=generate_can_header(CANDeviceID,0,ReqType,REF_SENSOR_REPLY_MSG);

			if ((FreeMailBox = HAL_CAN_GetTxMailboxesFreeLevel(hcan))>= 0)
			{
			  TxHeader.DLC=sizeof(ref_hum);
			  if( HAL_CAN_AddTxMessage(hcan, &TxHeader, (uint8_t *)&ref_hum, TxMailbox)!= HAL_OK)
			  {
				Error_Handler();
			  }
			}
        	break;
        case TEMPERATURE_REF_REQ:
			TxHeader.ExtId=generate_can_header(CANDeviceID,0,ReqType,REF_SENSOR_REPLY_MSG);

			if ((FreeMailBox = HAL_CAN_GetTxMailboxesFreeLevel(hcan))>= 0)
			{
			  TxHeader.DLC=sizeof(ref_temp);
			  if( HAL_CAN_AddTxMessage(hcan,&TxHeader, (uint8_t *)&ref_temp, TxMailbox)!= HAL_OK)
			  {
				Error_Handler();
			  }
			}
        	break;

        case ACCELERAMTER_REQ:
  			TxHeader.ExtId=generate_can_header(CANDeviceID,0, ReqType, ACCELERAMETER_REPLY_MSG);

  			if ((FreeMailBox = HAL_CAN_GetTxMailboxesFreeLevel(hcan))>= 0)
  			{
  				TxData[0] = Acce_X & 0xff;
		  		TxData[1] = (Acce_X >> 8) & 0xff;
				TxData[2] = Acce_Y & 0xff;
				TxData[3] = (Acce_Y >> 8) & 0xff;
				TxData[4] = Acce_Z & 0xff;
				TxData[5] = (Acce_Z >> 8) & 0xff;

				TxHeader.DLC=6;
				if( HAL_CAN_AddTxMessage(hcan,&TxHeader, TxData, TxMailbox)!= HAL_OK)
				{
				Error_Handler();
				}
  			}
          	break;

        default: break;

    }
}
void user_delay_ms(uint32_t period)
{
	HAL_Delay(period);
}
pin_port_s get_chip_pin(uint8_t id)
{
	pin_port_s temp;
	switch(id)
	{
	case 0:
	case 5:
	case 10:
			temp.Pin=CS010611_Pin;
			temp.Port=CS010611_GPIO_Port;
			break;
	case 1:
	case 6:
	case 11:
			temp.Pin=CS020712_Pin;
			temp.Port=CS020712_GPIO_Port;
			break;
	case 2:
	case 7:
	case 12:
			temp.Pin=CS030813_Pin;
			temp.Port=CS030813_GPIO_Port;
			break;
	case 3:
	case 8:
	case 13:
			temp.Pin=CS040914_Pin;
			temp.Port=CS040914_GPIO_Port;
			break;
	case 4:
	case 9:
	case 14:
			temp.Pin=CS051015_Pin;
			temp.Port=CS051015_GPIO_Port;
			break;
	}
 return temp;
}

void chip_select(uint8_t id)
{
  pin_port_s temp = get_chip_pin(id);
  HAL_GPIO_WritePin(temp.Port, temp.Pin, GPIO_PIN_RESET);
}

void chip_unselect(uint8_t id)
{
   pin_port_s temp = get_chip_pin(id);
   HAL_GPIO_WritePin(temp.Port, temp.Pin, GPIO_PIN_SET);
}

pin_port_s get_force_chip_pin(uint8_t id)
{
	pin_port_s temp;
	switch(id)
	{
		case 0:
			temp.Pin=R1_Pin;
			temp.Port=R1_GPIO_Port;
			break;
		case 1:
			temp.Pin=R2_Pin;
			temp.Port=R2_GPIO_Port;
			break;
		case 2:
			temp.Pin=R3_Pin;
			temp.Port=R3_GPIO_Port;
			break;
		case 3:
			temp.Pin=R4_Pin;
			temp.Port=R4_GPIO_Port;
			break;
		case 4:
			temp.Pin=R5_Pin;
			temp.Port=R5_GPIO_Port;
			break;
		case 5:
			temp.Pin=R6_Pin;
			temp.Port=R6_GPIO_Port;
			break;
		case 6:
			temp.Pin=R7_Pin;
			temp.Port=R7_GPIO_Port;
			break;
		case 7:
			temp.Pin=R8_Pin;
			temp.Port=R8_GPIO_Port;
			break;
		case 8:
			temp.Pin=R9_Pin;
			temp.Port=R9_GPIO_Port;
			break;
		case 9:
			temp.Pin=R10_Pin;
			temp.Port=R10_GPIO_Port;
			break;

		default:
			break;
	}
 return temp;
}

void force_sensor_select(uint8_t row)
{
	pin_port_s f_row=get_force_chip_pin(row);
	HAL_GPIO_WritePin(f_row.Port, f_row.Pin, GPIO_PIN_SET);
}


void force_sensor_unselect(uint8_t row)
{
	pin_port_s f_row=get_force_chip_pin(row);
	HAL_GPIO_WritePin(f_row.Port, f_row.Pin, GPIO_PIN_RESET);
}

void get_temperature_humidity (void)
{
	for (int i=0; i < TOTAL_TH_SENSORS_PERTILE; i++)
	{
		  switch (i)
		  {
		  case 0:
		  case 1:
		  case 2:
		  case 3:
		  case 4:
			  if(chip_id[i] == HTS221_ID)
			  {
				hts221_get_temp_humid(&hts221_ctx1, &temperature_values[i], &humidity_values[i], i);
			  }
			  else
			  {
				  // chip_id error
				  temperature_values[i] = 98.9;
				  humidity_values[i] = 989;
			  }
			break;

		  case 5:
		  case 6:
		  case 7:
		  case 8:
		  case 9:
			if(chip_id[i] == HTS221_ID)
			{
				hts221_get_temp_humid(&hts221_ctx2, &temperature_values[i], &humidity_values[i], i);
			}
			else
			{
			  // chip_id error
			  temperature_values[i] = 98.9;
			  humidity_values[i] = 989;
			}
			break;

		  case 10:
		  case 11:
		  case 12:
		  case 13:
		  case 14:
			if(chip_id[i] == HTS221_ID)
			{
				hts221_get_temp_humid(&hts221_ctx3, &temperature_values[i], &humidity_values[i], i);
			}
			else
			{
			  // chip_id error
			  temperature_values[i] = 98.9;
			  humidity_values[i] = 989;
			}
			break;

		  }
		chip_unselect(i);
	}
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		for (int i= 0; i <F_ROWS_PERTILE; i++)
		{
			ADCval[SampleCount][i] = ADC_BUF[i];
		}
		SampleCount++;
	}
}

// remove the min, max and average the rest
void  DataProcessing (void)
{
	uint8_t  i, j;
    uint16_t Smallest[F_ROWS_PERTILE];
    uint16_t Largest[F_ROWS_PERTILE];
    uint32_t Sum[F_ROWS_PERTILE]= {0};

    for (i = 0; i < F_ROWS_PERTILE; i++)
    {
			Smallest[i] = ADCval[0][i];
			Largest[i]  = ADCval [EVERAGE_TIME-1][i];
    }
    for(i=0; i<EVERAGE_TIME; i++)
    {
			for (j = 0; j < F_ROWS_PERTILE; j++)
			{
					Sum[j] += ADCval[i][j];

					if(ADCval[i][j] < Smallest[j])
					{
							Smallest[j] = ADCval[i][j];
					}
					if(ADCval[i][j] > Largest[j])
					{
							Largest[j] = ADCval[i][j];
					}
			}

    }


	for (j = 0; j < F_ROWS_PERTILE; j++)
	{
		AverageValue[j] = ((Sum[j] - Smallest[j] - Largest[j])/(EVERAGE_TIME-2));
	}
}



void ClearBuffer (void)
{
	for (int i= 0; i < F_ROWS_PERTILE; i++)
	{
		ADC_BUF[i] = 0;
	}

	for (int i= 0; i < EVERAGE_TIME; i++)
	{
		for (int j =0; j < F_ROWS_PERTILE; j++)
		{
			ADCval[i][j] = 0;
		}
	}
	SampleCount = 0;
}
void get_force_sensor_value(void)
{
	int OldSmapleCount = 0;

	for(int row=0;row<F_COLUMNS_PERTILE;row++)
	{
		force_sensor_select (row);

		HAL_Delay(200);

	    ClearBuffer ();
		OldSmapleCount = SampleCount = 0;

		// read EVERAGE_TIME times
		for (int i = 0; i < EVERAGE_TIME; i++)
		{
			HAL_ADC_Start_DMA (&hadc1, (uint32_t*) &ADC_BUF[0], 6);
			HAL_ADC_Start_IT(&hadc1);

			while (OldSmapleCount == SampleCount);
			OldSmapleCount = SampleCount;

			HAL_ADC_Stop_DMA (&hadc1);
			HAL_ADC_Stop_IT(&hadc1);
		}

		if (EVERAGE_TIME > 2)
		{
			// remove the min, max and average the rest
			DataProcessing ();
		}
		else
		{
			AverageValue[0] = ADC_BUF[0];
			AverageValue[1] = ADC_BUF[1];
			AverageValue[2] = ADC_BUF[2];
			AverageValue[3] = ADC_BUF[3];
			AverageValue[4] = ADC_BUF[4];
			AverageValue[5] = ADC_BUF[5];
		}
		force_sensor_unselect (row);

		force_values[row]    	= AverageValue[0]; // 1-10
		force_values[row+10] 	= AverageValue[1]; // 11-20
		force_values[row+20] 	= AverageValue[2]; // 21-30
		force_values[row+30] 	= AverageValue[3]; // 31-40
		force_values[row+40] 	= AverageValue[4]; // 41-50
		force_values[row+50] 	= AverageValue[5]; // 51-60

	}
}

/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		for (int i= 0; i <F_ROWS_PERTILE; i++)
		{
			ADCval[i] = ADC_BUF[i];
		}
		SampleCount++;
	}
}
void get_force_sensor_value(void)
{
	for(int row=0;row<F_COLUMNS_PERTILE;row++)
	{

		force_sensor_select (row);

		HAL_ADC_Start_DMA (&hadc1, (uint32_t*) ADC_BUF, 6);
		user_delay_ms(100);
		HAL_ADC_Stop_DMA (&hadc1);

		force_sensor_unselect (row);
		user_delay_ms(50);

		if(SampleCount)
		{
			force_values[row]    	= ADC_BUF[0]; // 1-10
			force_values[row+10] 	= ADC_BUF[1]; // 11-20
			force_values[row+20] 	= ADC_BUF[2]; // 21-30
			force_values[row+30] 	= ADC_BUF[3]; // 31-40
			force_values[row+40] 	= ADC_BUF[4]; // 41-50
			force_values[row+50] 	= ADC_BUF[5]; // 51-60
			user_delay_ms(400);
		}
		SampleCount = 0;
	}
}
*/
//read STM32 UniqueID
#define STM32_UUID ((uint32_t *)0x1FFFF7AC)

void ReadUID()
{
	DeviceUID = STM32_UUID[0] & 0xFFFFFFFF;
    //do something with the overall 96 bits
}



void update_sensor_values(void)
{
	// update force values
	get_force_sensor_value ();
	// update temperature and humiduty
	get_temperature_humidity ();
	// the following two should be scheduled update
	// update accelerameter
	lis2dh12_get_XYZ(&dev_ctx, &Acce_X, &Acce_Y, &Acce_Z);

	// update reference temperature and humidity
	shtc3_get_temp_humid(&dev_ctxc3, &ref_temp, &ref_hum);

}

void update_other_values(void)
{
	get_temperature_humidity ();

	lis2dh12_get_XYZ(&dev_ctx, &Acce_X, &Acce_Y, &Acce_Z);

	shtc3_get_temp_humid(&dev_ctxc3, &ref_temp, &ref_hum);
}


// Init HTS221 sensors
void init_HTS221 (void)
{
	for (int i =0; i < 5; i++)
	{
		hts221_init(&hts221_ctx1, i, &hspi1);
		hts221_device_id_get(&hts221_ctx1, &chip_id[i],  i);
		chip_unselect(i);
	}

	for (int i =5; i < 10; i++)
	{
		 hts221_init(&hts221_ctx2, i, &hspi2);
		 hts221_device_id_get(&hts221_ctx2, &chip_id[i],  i);
		 chip_unselect(i);
	}

	for (int i =10; i < 15; i++)
	{
		 hts221_init(&hts221_ctx3, i, &hspi3);
		 hts221_device_id_get(&hts221_ctx3, &chip_id[i],  i);
		 chip_unselect(i);
	}
}


void parse_can_header(uint32_t can_header,uint8_t* device_ID,uint8_t* sensor_ID, uint8_t* sensor_type,uint8_t* message_type)
{
    *device_ID=(uint8_t)((can_header&0x00FF0000) >> 16);
    //*sensor_ID=(uint8_t)((can_header&0xFF000000) >> 24);
    //*sensor_type=(uint8_t)(can_header&0x000000FF);
    *sensor_ID=(uint8_t)((can_header&0x000000FF));
    *sensor_type=(uint8_t)((can_header&0x1F000000) >> 24);
    *message_type=(uint8_t)((can_header&0x0000FF00) >> 8);
}


uint32_t generate_can_header(uint8_t device_ID,uint8_t sensor_ID, uint8_t sensor_type,uint8_t message_type)
{
   uint32_t tmp,res;
   	tmp=0;
   	res=0;
   	tmp=device_ID;
   	tmp<<=16;
   	res|=tmp;
   	tmp=0;

   	tmp=sensor_type;
   	tmp<<=24;
   	res|=tmp;
   	tmp=0;

   	tmp=sensor_ID;
   	res|=tmp;
   	tmp=0;

   	tmp=message_type;
   	tmp<<=8;
   	res|=tmp;
   	tmp=0;

   	return res;


}
