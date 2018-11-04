/*
 * ICM20948.h
 *
 *  Created on: Oct 26, 2018
 *      Author: cory
 */

#ifndef ICM20948_H_
#define ICM20948_H_

uint16_t accel_data[3];
uint16_t gyro_data[3];
int16_t mag_data[3];

#define SPI_BUS			(&hspi1) // ***
#define UART_BUS		(&huart3) // ***

#define USER_BANK_SEL	(0x7F)
#define USER_BANK_0		(0x00)
#define USER_BANK_1		(0x10)
#define USER_BANK_2		(0x20)
#define USER_BANK_3		(0x30)

#define PWR_MGMT_1 		(0x06)
#define PWR_MGMT_2		(0x07)
#define GYRO_CONFIG_1	(0x01)


#define CLK_BEST_AVAIL	(0x01)
#define GYRO_RATE_250	(0x00)
#define GYRO_LPF_17HZ 	(0x29)

void ICM_PowerOn();
uint8_t ICM_WHOAMI(void);
void ICM_SelectBank(uint8_t bank);
void ICM_ReadAccelGyro(void);
void ICM_ReadMag(int16_t magn[3]);
uint16_t ICM_Initialize(void);
void ICM_SelectBank(uint8_t bank);
void ICM_Disable_I2C(void);
void ICM_CSHigh(void);
void ICM_CSLow(void);
void ICM_SetClock(uint8_t clk);
void ICM_AccelGyroOff(void);
void ICM_AccelGyroOn(void);
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf);
void ICM_SetGyroLPF(uint8_t lpf);

#endif /* ICM20948_H_ */
