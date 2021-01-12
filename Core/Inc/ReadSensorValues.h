/*
 * ReadSensorValues.h
 *
 *  Created on: Jun 4, 2020
 *      Author: Holly
 */

#ifndef INC_READSENSORVALUES_H_
#define INC_READSENSORVALUES_H_

int8_t user_spi1_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_spi1_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_spi2_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_spi2_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_spi3_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_spi3_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
//Read STM32 unique ID
void ReadUID();
void user_delay_ms(uint32_t period);
void chip_select(uint8_t id);
void chip_unselect(uint8_t id);
pin_port_s get_chip_pin(uint8_t id);
pin_port_s get_force_chip_pin(uint8_t id);
void force_sensor_select(uint8_t row);
void force_sensor_unselect(uint8_t chip_ID);
void get_force_sensor_value(void);
void parse_can_header(uint32_t can_header,uint8_t* device_ID,uint8_t* sensor_ID, uint8_t* sensor_type,uint8_t* message_type);
uint32_t generate_can_header(uint8_t device_ID,uint8_t sensor_ID, uint8_t sensor_type,uint8_t message_type);
void send_sensor_value(CAN_HandleTypeDef *hcan, uint8_t sensID,uint8_t ReqType);
void update_sensors_values(void);
void get_force_sensor_value(void);
void update_other_values(void);
void update_sensor_values(void);
void init_HTS221 (void);

extern float temperature_values[TOTAL_TH_SENSORS_PERTILE];
extern float humidity_values[TOTAL_TH_SENSORS_PERTILE];
extern uint32_t force_values[TOTAL_F_SENSORS_PERTILE];
extern float ref_temp, ref_hum;

extern uint8_t 		chip_id[15];
extern uint8_t 		FreeMailBox;
extern stmdev_ctx_t 	dev_ctx;
extern uint16_t 		Acce_X,Acce_Y,Acce_Z;

extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;

extern uint8_t TxData[8];
extern uint8_t RxData[8];

extern uint32_t  DeviceUID;
extern uint8_t  CANDeviceID;

extern struct bme280_dev dev1;
extern struct bme280_dev dev2;
extern struct bme280_dev dev3;

extern hts221_ctx_t hts221_ctx1;//[NUMBERS_OF_SPI];
extern hts221_ctx_t hts221_ctx2;//[NUMBERS_OF_SPI];
extern hts221_ctx_t hts221_ctx3;//[NUMBERS_OF_SPI];

extern shtc3_driver_t dev_ctxc3;


#endif /* INC_READSENSORVALUES_H_ */
