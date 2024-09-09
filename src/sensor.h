#ifndef _SENSOR_H_
#define _SENSOR_H_
#include "Arduino.h"
#include "Wire.h"


#define DEV_I2C Wire1
#define SerialPort Serial

#define LPN_PIN 0
#define I2C_RST_PIN 2

//#define ENABLE_I2C_SENSOR_DBG
#ifdef ENABLE_I2C_SENSOR_DBG
#define I2C_SENSOR_DBG(...) {Serial.print("i2c_sensor ");Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define I2C_SENSOR_DBG(...)
#endif


typedef struct{
    uint16_t _8X8Data[64];
    uint16_t _4X4Data[16];
}sVL53Data_t;

/**
 * @fn initSensor
 * @brief 初始化传感器
*/
void initSensor_8x8(void);
void initSensor_4x4(void);

/**
 * @breif 初始化I2C主机
 */
void initI2CMaster(void);
/**
 * @fn getData
 * @brief 获取传感器数据
*/
void getData(void);


#endif
