#include "sensor.h"
#include "tof_slave.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "Arduino.h"


extern SemaphoreHandle_t xSemaphore;
extern StaticSemaphore_t xMutexBuffer;



void setup(){
    Serial.begin(115200);
    
    i2cSlaveInit();//初始化I2C从机 
    initI2CMaster();
    //pinMode(4, OUTPUT);
    
}

//主loop用于处理I2C从机问题
void loop(){
    i2cloop();//
    //Serial.println("hello");
    delay(1);
}
