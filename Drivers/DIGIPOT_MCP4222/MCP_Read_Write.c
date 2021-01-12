/*
 ******************************************************************************
 * @file    CP_Read_Write.c.c
 * @author  Holly
 * @brief   Mar 9, 2020
 ******************************************************************************
 * @attention
 *
 *
*/
#include "main.h"
#include "MCP_Read_Write.h"

uint8_t VMAdd[6];
uint8_t NVMAdd[6];
uint8_t RVdata[6];
uint8_t RNVdata[6];
uint16_t I2CAddress[6];


void mcp_init (void)
{
	VMAdd[0] = VW0;
	VMAdd[1] = VW1;
	VMAdd[2] = VW2;
	VMAdd[3] = VW3;
	VMAdd[4] = VW0;
	VMAdd[5] = VW1;

	NVMAdd[0] = NVW0;
	NVMAdd[1] = NVW1;
	NVMAdd[2] = NVW2;
	NVMAdd[3] = NVW3;
	NVMAdd[4] = NVW0;
	NVMAdd[5] = NVW1;


	I2CAddress[0] = MCP1_ADRESS;
	I2CAddress[1] = MCP1_ADRESS;
	I2CAddress[2] = MCP1_ADRESS;
	I2CAddress[3] = MCP1_ADRESS;
	I2CAddress[4] = MCP2_ADRESS;
	I2CAddress[5] = MCP2_ADRESS;
}

void mcp1_init (stmdev_ctx_mcp *ctx, I2C_HandleTypeDef *bus)
{
	VMAdd[0] = VW0;
	VMAdd[1] = VW1;
	VMAdd[2] = VW2;
	VMAdd[3] = VW3;

	NVMAdd[0] = NVW0;
	NVMAdd[1] = NVW1;
	NVMAdd[2] = NVW2;
	NVMAdd[3] = NVW3;


	I2CAddress[0] = MCP1_ADRESS;
	I2CAddress[1] = MCP1_ADRESS;
	I2CAddress[2] = MCP1_ADRESS;
	I2CAddress[3] = MCP1_ADRESS;
	I2CAddress[4] = MCP2_ADRESS;
	I2CAddress[5] = MCP2_ADRESS;

	ctx->write_reg = mcp1_write;
	ctx->read_reg = mcp1_read;
	ctx->handle = bus;

}

void mcp2_init (stmdev_ctx_mcp *ctx, I2C_HandleTypeDef *bus)
{
	VMAdd[4] = VW0;
	VMAdd[5] = VW1;

	NVMAdd[4] = NVW0;
	NVMAdd[5] = NVW1;

	ctx->write_reg = mcp2_write;
	ctx->read_reg = mcp2_read;
	ctx->handle = bus;

}
/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t mcp_read_reg(stmdev_ctx_mcp* ctx, uint8_t reg, uint8_t* data,
                          uint16_t len)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len);
  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t mcp_write_reg(stmdev_ctx_mcp* ctx, uint8_t reg, uint8_t* data,
                           uint16_t len)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len);
  return ret;
}


int32_t mcp1_write(stmdev_ctx_mcp *ctx, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    HAL_I2C_Mem_Write(ctx, MCP1_ADRESS, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    return 0;
}

int32_t mcp1_read(stmdev_ctx_mcp *ctx, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Read(ctx, MCP1_ADRESS, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}


int32_t mcp2_write(stmdev_ctx_mcp *ctx, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    HAL_I2C_Mem_Write(ctx, MCP2_ADRESS, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    return 0;
}

int32_t mcp2_read(stmdev_ctx_mcp *ctx, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	HAL_I2C_Mem_Read(ctx, MCP2_ADRESS, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}
// Set one digipot
void set_mcp(int i, uint8_t value)
{
    uint8_t WVdata[2], WNVdata[2];

    WVdata[0] = VMAdd[i] | CMD_WRITE;
    WNVdata[0] = NVMAdd[i] | CMD_WRITE;
    WVdata[1] = WNVdata[1] = value;

    HAL_I2C_Master_Transmit(&hi2c1,I2CAddress[i],WVdata,2,1000);
    HAL_Delay(5);
    HAL_I2C_Master_Transmit(&hi2c1,I2CAddress[i],WNVdata,2,1000);
    HAL_Delay(5);

}

// get one setting value
void get_mcp(uint8_t i)
{
    uint8_t RVdataL[2], RNVdataL[2];


    RVdataL[0] = VMAdd[i] | CMD_READ;
    RNVdataL[0] = NVMAdd[i] | CMD_READ;

    HAL_I2C_Master_Transmit(&hi2c1,I2CAddress[i],&RVdataL[0],1,1000) ;
    HAL_I2C_Master_Receive(&hi2c1,I2CAddress[i]+1,RVdataL,2,1000);
    HAL_Delay(5);
    HAL_I2C_Master_Transmit(&hi2c1,I2CAddress[i],&RNVdataL[0],1,1000);
    HAL_I2C_Master_Receive(&hi2c1,I2CAddress[i]+1,RNVdataL,2,1000);
    HAL_Delay(5);

    RVdata[i] = RVdataL[1];
    RNVdata[i] = RNVdataL[1];


}
//Setting mcp wipers MCP1 (WP0, WP1, WP2, WP3) for (op6,op5,op4,op3)
//and MCP2 (WP0, WP1, x,x) for (op2, op1)in Kohm
void Set_DigipotMCPs(uint8_t *SetValues)
{
	for (int i =0; i < 6; i++)
	{
		set_mcp (i, SetValues[i]);
	}

	for (int i =0; i < 6; i++)
	{
//		get_mcp(i);
	}

}
/*
	mcp1_init(&dev_ctx_mcp1, &hi2c1);
	for (int i = 0; i < 4; i++)
	{
		 mcp_write_reg(&dev_ctx_mcp1, VMAdd[i], SetValues[i], 1);
		 user_delay (5);
		 mcp_write_reg(&dev_ctx_mcp1, NVMAdd[i], SetValues[i], 1);
		 user_delay (5);

		 mcp_read_reg(&dev_ctx_mcp1, VMAdd[i], RVdataL, 2);
		 RVdata[i] = RVdataL[1];
		 user_delay (5);
		 mcp_read_reg(&dev_ctx_mcp1, NVMAdd[i], RVdataL, 2);
		 RVdata[i] = RNVdataL[1];

		 user_delay (5);
	}
	mcp2_init(&dev_ctx_mcp2, &hi2c1);

	for (int i = 4; i < 6; i++)
	{
		 mcp_write_reg(&dev_ctx_mcp2, VMAdd[i], SetValues[i], 1);
		 user_delay (5);
		 mcp_write_reg(&dev_ctx_mcp2, NVMAdd[i], SetValues[i], 1);
		 user_delay (5);

		 mcp_read_reg(&dev_ctx_mcp2, VMAdd[i], RVdataL, 2);
		 RVdata[i] = RVdataL[1];

		 user_delay (5);
		 mcp_read_reg(&dev_ctx_mcp2, NVMAdd[i], RVdataL, 2);
		 RVdata[i] = RNVdataL[1];

		 user_delay (5);
	}
*/
