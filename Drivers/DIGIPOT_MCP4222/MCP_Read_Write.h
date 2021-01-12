/*
 * MCP_Read_Write.h
 *
 *  Created on: Mar 9, 2020
 *      Author: Holly
 */

#ifndef DRIVERS_DIGIPOT_MCP4222_MCP_READ_WRITE_H_
#define DRIVERS_DIGIPOT_MCP4222_MCP_READ_WRITE_H_

#define MCP1_ADRESS 0x58
#define MCP2_ADRESS 0x5c

#define VW0 0x00
#define VW1 0x10
#define VW2 0x60
#define VW3 0x70
#define NVW0 0x20
#define NVW1 0x30
#define NVW2 0x80
#define NVW3 0x90

//#define CONT0 0x40
#define CONT0 0xa0


#define CMD_WRITE 0x00
#define CMD_READ 0x0c
#define CMD_INC 0x04
#define CMD_DEC 0x08


typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*stmdev_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_mcp;

void mcp_init (void);

void set_mcp(int i, uint8_t value);
void get_mcp(uint8_t i);
void Set_DigipotMCPs(uint8_t *SetValues);

int32_t mcp1_write(stmdev_ctx_mcp *ctx, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t mcp1_read(stmdev_ctx_mcp *ctx, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t mcp2_write(stmdev_ctx_mcp *ctx, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t mcp2_read(stmdev_ctx_mcp *ctx, uint8_t reg, uint8_t *bufp, uint16_t len);


void mcp1_init (stmdev_ctx_mcp *ctx, I2C_HandleTypeDef *bus);
void mcp2_init (stmdev_ctx_mcp *ctx, I2C_HandleTypeDef *bus);

extern I2C_HandleTypeDef hi2c1;

#endif /* DRIVERS_DIGIPOT_MCP4222_MCP_READ_WRITE_H_ */
