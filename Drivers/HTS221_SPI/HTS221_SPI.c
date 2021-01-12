/*
 ******************************************************************************
 * @file    hts221_reg.c
 * @author  MEMS Software Solution Team
 * @brief   HTS221 driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include "HTS221_SPI.h"
#include <string.h>
#include "stm32f3xx_hal.h"
#include "main.h"

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
/**
  * @defgroup  HTS221
  * @brief     This file provides a set of functions needed to drive the
  *            hts221 enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup  HTS221_interfaces_functions
  * @brief     This section provide a set of functions used to read and write
  *            a generic register of the device.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_read_reg(hts221_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len, uint8_t cs)
{
  int32_t ret;
  ret = ctx->read_reg(ctx->handle, reg, data, len, cs);
  return ret;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_write_reg(hts221_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len, uint8_t cs)
{
  int32_t ret;
  ret = ctx->write_reg(ctx->handle, reg, data, len, cs);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  HTS221_Data_Generation
  * @brief     This section group all the functions concerning data generation
  * @{
  *
  */

/**
  * @brief  The numbers of averaged humidity samples.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of avgh in reg AV_CONF
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_humidity_avg_set(hts221_ctx_t *ctx, hts221_avgh_t val, uint8_t cs)
{
  hts221_av_conf_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_AV_CONF, (uint8_t*) &reg, 1, cs);

  if(ret == 0){
    reg.avgh = (uint8_t)val;
    ret = hts221_write_reg(ctx, HTS221_AV_CONF, (uint8_t*) &reg, 1, cs);
  }

  return ret;
}

/**
  * @brief  The numbers of averaged humidity samples.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Get the values of avgh in reg AV_CONF
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_humidity_avg_get(hts221_ctx_t *ctx, hts221_avgh_t *val, uint8_t cs)
{
  hts221_av_conf_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_AV_CONF, (uint8_t*) &reg, 1, cs);

  switch (reg.avgh) {
    case HTS221_H_AVG_4:
      *val = HTS221_H_AVG_4;
      break;
    case HTS221_H_AVG_8:
      *val = HTS221_H_AVG_8;
      break;
    case HTS221_H_AVG_16:
      *val = HTS221_H_AVG_16;
      break;
    case HTS221_H_AVG_32:
      *val = HTS221_H_AVG_32;
      break;
    case HTS221_H_AVG_64:
      *val = HTS221_H_AVG_64;
      break;
    case HTS221_H_AVG_128:
      *val = HTS221_H_AVG_128;
      break;
    case HTS221_H_AVG_256:
      *val = HTS221_H_AVG_256;
      break;
    case HTS221_H_AVG_512:
      *val = HTS221_H_AVG_512;
      break;
    default:
      *val = HTS221_H_AVG_ND;
      break;
  }

  return ret;
}

/**
  * @brief  The numbers of averaged temperature samples.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of avgt in reg AV_CONF
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temperature_avg_set(hts221_ctx_t *ctx, hts221_avgt_t val, uint8_t cs)
{
  hts221_av_conf_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_AV_CONF, (uint8_t*) &reg, 1, cs);

  if(ret == 0){
    reg.avgt = (uint8_t)val;
    ret = hts221_write_reg(ctx, HTS221_AV_CONF, (uint8_t*) &reg, 1, cs);
  }

  return ret;
}

/**
  * @brief  The numbers of averaged temperature samples.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Get the values of avgt in reg AV_CONF
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temperature_avg_get(hts221_ctx_t *ctx, hts221_avgt_t *val, uint8_t cs)
{
  hts221_av_conf_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_AV_CONF, (uint8_t*) &reg, 1, cs);

  switch (reg.avgh) {
    case HTS221_T_AVG_2:
      *val = HTS221_T_AVG_2;
      break;
    case HTS221_T_AVG_4:
      *val = HTS221_T_AVG_4;
      break;
    case HTS221_T_AVG_8:
      *val = HTS221_T_AVG_8;
      break;
    case HTS221_T_AVG_16:
      *val = HTS221_T_AVG_16;
      break;
    case HTS221_T_AVG_32:
      *val = HTS221_T_AVG_32;
      break;
    case HTS221_T_AVG_64:
      *val = HTS221_T_AVG_64;
      break;
    case HTS221_T_AVG_128:
      *val = HTS221_T_AVG_128;
      break;
    case HTS221_T_AVG_256:
      *val = HTS221_T_AVG_256;
      break;
    default:
      *val = HTS221_T_AVG_ND;
      break;
  }

  return ret;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of odr in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_data_rate_set(hts221_ctx_t *ctx, hts221_odr_t val, uint8_t cs)
{
  hts221_ctrl_reg1_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG1, (uint8_t*) &reg, 1, cs);

  if(ret == 0){
    reg.odr = (uint8_t)val;
    ret = hts221_write_reg(ctx, HTS221_CTRL_REG1, (uint8_t*) &reg, 1, cs);
  }

  return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Get the values of odr in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_data_rate_get(hts221_ctx_t *ctx, hts221_odr_t *val, uint8_t cs)
{
  hts221_ctrl_reg1_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG1, (uint8_t*) &reg, 1, cs);

  switch (reg.odr) {
    case HTS221_ONE_SHOT:
      *val = HTS221_ONE_SHOT;
      break;
    case HTS221_ODR_1Hz:
      *val = HTS221_ODR_1Hz;
      break;
    case HTS221_ODR_7Hz:
      *val = HTS221_ODR_7Hz;
      break;
    case HTS221_ODR_12Hz5:
      *val = HTS221_ODR_12Hz5;
      break;
    default:
      *val = HTS221_ODR_ND;
      break;
  }

  return ret;
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of bdu in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_block_data_update_set(hts221_ctx_t *ctx, uint8_t val, uint8_t cs)
{
  hts221_ctrl_reg1_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG1, (uint8_t*) &reg, 1, cs);

  if(ret == 0){
    reg.bdu = val;
    ret = hts221_write_reg(ctx, HTS221_CTRL_REG1, (uint8_t*) &reg, 1, cs);
  }

  return ret;
}

/**
  * @brief  Block data update.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of bdu in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_block_data_update_get(hts221_ctx_t *ctx, uint8_t *val, uint8_t cs)
{
  hts221_ctrl_reg1_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG1, (uint8_t*) &reg, 1, cs);
  *val = reg.bdu;

  return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of one_shot in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_one_shoot_trigger_set(hts221_ctx_t *ctx, uint8_t val, uint8_t cs)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG2, (uint8_t*) &reg, 1, cs);

  if(ret == 0){
    reg.one_shot = val;
    ret = hts221_write_reg(ctx, HTS221_CTRL_REG2, (uint8_t*) &reg, 1, cs);
  }

  return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of one_shot in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_one_shoot_trigger_get(hts221_ctx_t *ctx, uint8_t *val, uint8_t cs)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG2, (uint8_t*) &reg, 1, cs);
  *val = reg.one_shot;

  return ret;
}

/**
  * @brief  Temperature data available.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of t_da in reg STATUS_REG
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temp_data_ready_get(hts221_ctx_t *ctx, uint8_t *val, uint8_t cs)
{
  hts221_status_reg_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_STATUS_REG, (uint8_t*) &reg, 1, cs);
  *val = reg.t_da;

  return ret;
}

/**
  * @brief  Humidity data available.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of h_da in reg STATUS_REG
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_hum_data_ready_get(hts221_ctx_t *ctx, uint8_t *val, uint8_t cs)
{
  hts221_status_reg_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_STATUS_REG, (uint8_t*) &reg, 1, cs);
  *val = reg.h_da;

  return ret;
}

/**
  * @brief  Humidity output value[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_humidity_raw_get(hts221_ctx_t *ctx, uint8_t *buff, uint8_t cs)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, HTS221_HUMIDITY_OUT_L, buff, 2, cs);
  return ret;
}

/**
  * @brief  Temperature output value[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temperature_raw_get(hts221_ctx_t *ctx, uint8_t *buff, uint8_t cs)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, HTS221_TEMP_OUT_L, buff, 2, cs);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  HTS221_common
  * @brief     This section group common usefull functions
  * @{
  *
  */

/**
  * @brief  Device Who amI.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_device_id_get(hts221_ctx_t *ctx, uint8_t *buff, uint8_t cs)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, HTS221_WHO_AM_I, buff, 1, cs);
  return ret;
}

/**
  * @brief  Switch device on/off.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of pd in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_power_on_set(hts221_ctx_t *ctx, uint8_t val, uint8_t cs)
{
  hts221_ctrl_reg1_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG1, (uint8_t*) &reg, 1, cs);

  if(ret == 0){
    reg.pd = val;
    ret = hts221_write_reg(ctx, HTS221_CTRL_REG1, (uint8_t*) &reg, 1, cs);
  }
  return ret;
}

/**
  * @brief  Switch device on/off.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of pd in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_power_on_get(hts221_ctx_t *ctx, uint8_t *val, uint8_t cs)
{
  hts221_ctrl_reg1_t reg;
  int32_t mm_error;

  mm_error = hts221_read_reg(ctx, HTS221_CTRL_REG1, (uint8_t*) &reg, 1, cs);
  *val = reg.pd;

  return mm_error;
}

/**
  * @brief  Heater enable / disable.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of heater in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_heater_set(hts221_ctx_t *ctx, uint8_t val, uint8_t cs)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG2, (uint8_t*) &reg, 1, cs);

  if(ret == 0){
    reg.heater = val;
    ret = hts221_write_reg(ctx, HTS221_CTRL_REG2, (uint8_t*) &reg, 1, cs);
  }

  return ret;
}

/**
  * @brief  Heater enable / disable.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of heater in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_heater_get(hts221_ctx_t *ctx, uint8_t *val, uint8_t cs)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG2, (uint8_t*) &reg, 1, cs);
  *val = reg.heater;

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of boot in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_boot_set(hts221_ctx_t *ctx, uint8_t val, uint8_t cs)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG2, (uint8_t*) &reg, 1, cs);

  if(ret == 0){
    reg.boot = val;
    ret = hts221_write_reg(ctx, HTS221_CTRL_REG2, (uint8_t*) &reg, 1, cs);
  }

  return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of boot in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_boot_get(hts221_ctx_t *ctx, uint8_t *val, uint8_t cs)
{
  hts221_ctrl_reg2_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG2, (uint8_t*) &reg, 1, cs);
  *val = reg.boot;

  return ret;
}

/**
  * @brief  Info about device status.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Registers STATUS_REG
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_status_get(hts221_ctx_t *ctx, hts221_status_reg_t *val, uint8_t cs)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, HTS221_STATUS_REG, (uint8_t*) val, 1, cs);
  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  HTS221_interrupts
  * @brief   This section group all the functions that manage interrupts
  * @{
  *
  */

/**
  * @brief  Data-ready signal on INT_DRDY pin.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of drdy in reg CTRL_REG3
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_drdy_on_int_set(hts221_ctx_t *ctx, uint8_t val, uint8_t cs)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG3, (uint8_t*) &reg, 1, cs);

  if(ret == 0){
    reg.drdy = val;
    ret = hts221_write_reg(ctx, HTS221_CTRL_REG3, (uint8_t*) &reg, 1, cs);
  }

  return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of drdy in reg CTRL_REG3
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_drdy_on_int_get(hts221_ctx_t *ctx, uint8_t *val, uint8_t cs)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG3, (uint8_t*) &reg, 1, cs);
  *val = reg.drdy;

  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of pp_od in reg CTRL_REG3
  *
  */
int32_t hts221_pin_mode_set(hts221_ctx_t *ctx, hts221_pp_od_t val, uint8_t cs)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG3, (uint8_t*) &reg, 1, cs);

  if(ret == 0){
    reg.pp_od = (uint8_t)val;
    ret = hts221_write_reg(ctx, HTS221_CTRL_REG3, (uint8_t*) &reg, 1, cs);
  }

  return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Get the values of pp_od in reg CTRL_REG3
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_pin_mode_get(hts221_ctx_t *ctx, hts221_pp_od_t *val, uint8_t cs)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG3, (uint8_t*) &reg, 1, cs);

  switch (reg.pp_od) {
    case HTS221_PUSH_PULL:
      *val = HTS221_PUSH_PULL;
      break;
    case HTS221_OPEN_DRAIN:
      *val = HTS221_OPEN_DRAIN;
      break;
    default:
      *val = HTS221_PIN_MODE_ND;
      break;
  }

  return ret;
}

/**
  * @brief  Interrupt active-high/low.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of drdy_h_l in reg CTRL_REG3
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_int_polarity_set(hts221_ctx_t *ctx, hts221_drdy_h_l_t val, uint8_t cs)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG3, (uint8_t*) &reg, 1, cs);

  if(ret == 0){
    reg.drdy_h_l = (uint8_t)val;
    ret = hts221_write_reg(ctx, HTS221_CTRL_REG3, (uint8_t*) &reg, 1, cs);
  }

  return ret;
}

/**
  * @brief  Interrupt active-high/low.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Get the values of drdy_h_l in reg CTRL_REG3
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_int_polarity_get(hts221_ctx_t *ctx, hts221_drdy_h_l_t *val, uint8_t cs)
{
  hts221_ctrl_reg3_t reg;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_CTRL_REG3, (uint8_t*) &reg, 1, cs);

  switch (reg.drdy_h_l) {
    case HTS221_ACTIVE_HIGH:
      *val = HTS221_ACTIVE_HIGH;
      break;
    case HTS221_ACTIVE_LOW:
      *val = HTS221_ACTIVE_LOW;
      break;
    default:
      *val = HTS221_ACTIVE_ND;
      break;
  }

  return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  HTS221_calibration
  * @brief     This section group all the calibration coefficients need
  *            for reading data
  * @{
  *
  */

/**
  * @brief  First calibration point for Rh Humidity.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_hum_rh_point_0_get(hts221_ctx_t *ctx, uint8_t *buff, uint8_t cs)
{
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_H0_RH_X2, buff, 1, cs);
  *buff = (uint8_t)(((uint16_t)(*buff) >> 1) & 0x7FFFu);

  return ret;
}

/**
  * @brief  Second calibration point for Rh Humidity.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_hum_rh_point_1_get(hts221_ctx_t *ctx, uint8_t *buff, uint8_t cs)
{
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_H1_RH_X2, buff, 1, cs);
  *buff = (uint8_t)(((uint16_t)(*buff) >> 1) & 0x7FFFu);

  return ret;
}

/**
  * @brief  First calibration point for degC temperature.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temp_deg_point_0_get(hts221_ctx_t *ctx, uint8_t *buff, uint8_t cs)
{
  hts221_t1_t0_msb_t reg;
  uint8_t coeff_h, coeff_l;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_T0_DEGC_X8, &coeff_l, 1, cs);

  if(ret == 0){
    ret = hts221_read_reg(ctx, HTS221_T1_T0_MSB, (uint8_t*) &reg, 1, cs);
    coeff_h = reg.t0_msb;
    *(buff) = (uint8_t)(((coeff_h << 8) + coeff_l) >> 3);
  }

  return ret;
}

/**
  * @brief  Second calibration point for degC temperature.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temp_deg_point_1_get(hts221_ctx_t *ctx, uint8_t *buff, uint8_t cs)
{
  hts221_t1_t0_msb_t reg;
  uint8_t coeff_h, coeff_l;
  int32_t ret;

  ret = hts221_read_reg(ctx, HTS221_T1_DEGC_X8, &coeff_l, 1, cs);

  if(ret == 0){
    ret = hts221_read_reg(ctx, HTS221_T1_T0_MSB, (uint8_t*) &reg, 1, cs);
    coeff_h = reg.t1_msb;
    *(buff) = (uint8_t)(((coeff_h << 8) + coeff_l) >> 3);
  }

  return ret;
}

/**
  * @brief  First calibration point for humidity in LSB.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_hum_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff, uint8_t cs)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, HTS221_H0_T0_OUT_L, buff, 2, cs);
  return ret;
}

/**
  * @brief  Second calibration point for humidity in LSB.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_hum_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff, uint8_t cs)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, HTS221_H1_T0_OUT_L, buff, 2, cs);
  return ret;
}

/**
  * @brief  First calibration point for temperature in LSB.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temp_adc_point_0_get(hts221_ctx_t *ctx, uint8_t *buff, uint8_t cs)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, HTS221_T0_OUT_L, buff, 2, cs);
  return ret;
}

/**
  * @brief  Second calibration point for temperature in LSB.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temp_adc_point_1_get(hts221_ctx_t *ctx, uint8_t *buff, uint8_t cs)
{
  int32_t ret;
  ret = hts221_read_reg(ctx, HTS221_T1_OUT_L, buff, 2, cs);
  return ret;
}

float linear_interpolation(lin_t *lin, int16_t x)
{
 return ((lin->y1 - lin->y0) * x +  ((lin->x1 * lin->y0) - (lin->x0 * lin->y1))) / (lin->x1 - lin->x0);
}

int32_t platform_writeSPI(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len, uint8_t cs)
{
	reg |= 0x40;
	chip_select(cs);
	HAL_SPI_Transmit(handle, &reg, 1, 1000);
	HAL_SPI_Transmit(handle, bufp, len, 1000);
	chip_unselect(cs);
	return 0;
}

int32_t platform_readSPI(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len, uint8_t cs)
{
    reg |= 0xC0;
	chip_select(cs);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
	chip_unselect(cs);
	return 0;
}

void hts221_init(hts221_ctx_t *ctx, uint8_t cs, SPI_HandleTypeDef *bus)
{
	ctx->handle = bus;
	ctx->write_reg = platform_writeSPI;
	ctx->read_reg = platform_readSPI;
	hts221_block_data_update_set(ctx, PROPERTY_ENABLE,cs);
	hts221_data_rate_set(ctx, HTS221_ODR_1Hz,cs);
//	hts221_power_on_set(ctx, PROPERTY_ENABLE,cs);
}
#if(0)
// new driver for HTS221 from ST, it has to be applied to the current driver carefully
/***@briefRead
 * HTS221Humidityoutput registers, and calculatehumidity.
 * *@paramPointerto thereturnedhumidityvaluethatmustbedividedby10 togetthevaluein[%].
 * *@retvalErrorcode [HTS221_OK, HTS221_ERROR].*/

HTS221_Error_et HTS221_Get_Humidity(uint16_t*value)
{
	int16_t H0_T0_out, H1_T0_out,H_T_out;
	int16_t H0_rh,H1_rh; uint8_t buffer[2];
	int32_ttmp;
	/*1.ReadH0_rHandH1_rHcoefficients*/
	if(HTS221_ReadReg(HTS221_H0_RH_X2, 2, buffer))
		H0_rh=buffer[0]>>1;H1_rh=buffer[1]>>1;
	/*2.ReadH0_T0_OUT*/
	if(HTS221_ReadReg(HTS221_H0_T0_OUT_L, 2,buffer))
		returnHTS221_ERROR;H0_T0_out=(((uint16_t)buffer[1])<<8)|(uint16_t)buffer[0];
	/*3.ReadH1_T0_OUT*/
	if(HTS221_ReadReg(HTS221_H1_T0_OUT_L, 2,buffer))
		returnHTS221_ERROR;H1_T0_out=(((uint16_t)buffer[1])<<8)|(uint16_t)buffer[0];
	/*4.ReadH_T_OUT*/if(HTS221_ReadReg(HTS221_HR_OUT_L_REG, 2, buffer))
		returnHTS221_ERROR;H_T_out=(((uint16_t)buffer[1])<<8)|(uint16_t)buffer[0];
	/*5.ComputetheRH[%]valuebylinearinterpolation*/
	tmp=((int32_t)(H_T_out-H0_T0_out))*((int32_t)(H1_rh-H0_rh)*10);
	*value= (tmp/(H1_T0_out-H0_T0_out)+H0_rh*10);
	/*Saturationcondition*/
	if(*value>1000)*value=1000;
	returnHTS221_OK;
}

/**
 * *@briefReadHTS221temperatureoutput registers,andcalculate temperature.
 * *@paramPointerto thereturnedtemperaturevaluethatmust bedividedby10 togetthevaluein ['C].
 * *@retvalErrorcode [HTS221_OK, HTS221_ERROR].
 * */

HTS221_Error_etHTS221_Get_Temperature(hts221_ctx_t *ctx, int16_t*value, , uint8_t cs)
{
	int16_tT0_out,T1_out,T_out,T0_degC_x8_u16,T1_degC_x8_u16;
	int16_tT0_degC,T1_degC;uint8_t buffer[4],tmp;int32_ttmp32;
	/*1. Readfrom0x32&0x33registers thevalueofcoefficientsT0_degC_x8andT1_degC_x8*/
	if(HTS221_ReadReg(HTS221_T0_DEGC_X8,2, buffer)) //hts221_temp_deg_point_0_get
		returnHTS221_ERROR;
	/*2. Readfrom0x35register thevalueoftheMSBbitsofT1_degCandT0_degC*/
	if(HTS221_ReadReg(HTS221_T0_T1_DEGC_H2,1,&tmp))returnHTS221_ERROR;
	/*Calculate theT0_degCandT1_degCvalues*/
	T0_degC_x8_u16=(((uint16_t)(tmp &0x03))<<8)|((uint16_t)buffer[0]);
	T1_degC_x8_u16=(((uint16_t)(tmp &0x0C))<<6)|((uint16_t)buffer[1]);
	T0_degC=T0_degC_x8_u16>>3;T1_degC=T1_degC_x8_u16>>3;
	/*3.Readfrom0x3C&0x3Dregisters thevalueofT0_OUT*/
	/*4.Readfrom0x3E&0x3Fregisters thevalueofT1_OUT*/
	if(HTS221_ReadReg(HTS221_T0_OUT_L,4, buffer))
		returnHTS221_ERROR;T0_out=(((uint16_t)buffer[1])<<8)|(uint16_t)buffer[0];
	T1_out=(((uint16_t)buffer[3])<<8)|(uint16_t)buffer[2];
	/*5.Readfrom0x2A&0x2Bregisters thevalueT_OUT(ADC_OUT).*/
	if(HTS221_ReadReg(HTS221_TEMP_OUT_L_REG,2, buffer))
		returnHTS221_ERROR;
	T_out=(((uint16_t)buffer[1])<<8)|(uint16_t)buffer[0];
	/*6.Compute theTemperaturevaluebylinearinterpolation*/
	tmp32=((int32_t)(T_out-T0_out))*((int32_t)(T1_degC-T0_degC)*10);
	*value=tmp32/(T1_out-T0_out)+T0_degC*10;
	returnHTS221_OK;
}
#endif

void hts221_get_temp_humid(hts221_ctx_t *ctx, float *temperature, float *humidity, uint8_t cs)
{
	axis1bit16_t data_raw_temperature, data_raw_humidity, coeff;
	lin_t lin_temp, lin_hum;

	hts221_power_on_set(ctx, PROPERTY_ENABLE,cs);
	user_delay_ms(100);

	hts221_hum_adc_point_0_get(ctx, coeff.u8bit, cs);
	lin_hum.x0 = (float)coeff.i16bit;
	hts221_hum_rh_point_0_get(ctx, coeff.u8bit, cs);
	lin_hum.y0 = (float)coeff.u8bit[0];
	hts221_hum_adc_point_1_get(ctx, coeff.u8bit, cs);
	lin_hum.x1 = (float)coeff.i16bit;
	hts221_hum_rh_point_1_get(ctx, coeff.u8bit, cs);
	lin_hum.y1 = (float)coeff.u8bit[0];

	hts221_temp_adc_point_0_get(ctx, coeff.u8bit, cs);
	lin_temp.x0 = (float)coeff.i16bit;
	hts221_temp_deg_point_0_get(ctx, coeff.u8bit, cs);
	lin_temp.y0 = (float)coeff.u8bit[0];
	hts221_temp_adc_point_1_get(ctx, coeff.u8bit, cs);
	lin_temp.x1 = (float)coeff.i16bit;
	hts221_temp_deg_point_1_get(ctx, coeff.u8bit, cs);
	lin_temp.y1 = (float)coeff.u8bit[0];

    memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
    memset(data_raw_humidity.u8bit, 0x00, sizeof(int16_t));

	hts221_temperature_raw_get(ctx, data_raw_temperature.u8bit, cs);
	hts221_humidity_raw_get(ctx, data_raw_humidity.u8bit, cs);

	*temperature = linear_interpolation(&lin_temp, data_raw_temperature.i16bit);
	if(*temperature < 10 || *temperature > 50) *temperature = 97.9;
	*humidity = linear_interpolation(&lin_hum, data_raw_humidity.i16bit);
	// subtract 10% from raw reading
	*humidity = (*humidity * 0.9);
	// outof range
	if(*humidity < 10) *humidity = 979;
	if(*humidity > 95) *humidity = 978;
	hts221_power_on_set(ctx, PROPERTY_DISABLE,cs);

}

/**
  * @}
  *
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
