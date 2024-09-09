#include <vl53l7cx_class.h>
#include "sensor.h"
#include "stdio.h"
#include "stdlib.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#define STACK_SIZE 4 * 1024

VL53L7CX sensor_vl53l7cx_top(&DEV_I2C, LPN_PIN, I2C_RST_PIN);
VL53L7CX_ResultsData Results;
sVL53Data_t *vl53 = NULL;
SemaphoreHandle_t xSemaphore = NULL;//数据操作锁
StaticSemaphore_t xMutexBuffer;
uint8_t _mode = 8;//默认8*8
int8_t _getDataMode = -1;
uint16_t _obstacle_threshold = 0;

StaticTask_t xTaskBuffer_A;
StackType_t xStack_A[ STACK_SIZE ];
TaskHandle_t xHandle = NULL;

uint16_t histogram[8];
uint16_t sector_count[8];



//边缘碰撞
static uint8_t flootDir0 = 0b111;
static uint8_t flootDir7 = 0b111;
//转向趋势
static uint8_t dirTrend = 0b111;
static uint8_t dirTrend1 = 0b111;

#define LINE2MIN    180 
#define LINE4MIN    80
#define LINE6MIN    32
#define LINE8MIN    100
#define LINE1MIN    100//左上角和右上角阈值判断



#define FORWARD  0b010
#define BACKWARD  2
#define LEFT  0b100
#define RIGHT  0b001

uint8_t realDir = FORWARD;
uint8_t realSpeed = 0;
bool LineAferTurn = false;

uint16_t hedging = 110;//紧急避险
uint16_t wall = 100;

uint16_t LObstacleDistance = 0;//左侧障碍物距离
uint16_t MObstacleDistance = 0;//中间障碍物距离
uint16_t RObstacleDistance = 0;//右侧障碍物距离

#define NUM_SECTORS 8

void initI2CMaster(void){
    //初始化传感器通信I2C
    
    DEV_I2C.setSDA(6);
    DEV_I2C.setSCL(7); 
    DEV_I2C.setClock(400000);//400K频率 400000
    DEV_I2C.begin();
}

uint8_t deleteTask = 0;
void get_data_task(void *pvParameters)//处理数据任务
{
  (void) pvParameters;
  while (1)
  {
    getData();//获取传感器数据
    if(deleteTask == 1){
      sensor_vl53l7cx_top.vl53l7cx_stop_ranging();
      vTaskDelete(NULL);
    }
    delay(1);
  }

}

void initSensor_8x8(void){
    uint8_t status;
    uint32_t integration_time_ms;
    if(xHandle){
      //sensor_vl53l7cx_top.vl53l7cx_stop_ranging();
      //
      //vTaskDelete(xHandle);
      deleteTask = 1;
      free(vl53);
      delay(100);
      xHandle = NULL;
    }
    deleteTask = 0;
    I2C_SENSOR_DBG("init sensor ...");
    // Configure VL53L7CX satellite component.
    sensor_vl53l7cx_top.begin();
    sensor_vl53l7cx_top.vl53l7cx_i2c_reset();
    sensor_vl53l7cx_top.init_sensor();
    
    status = sensor_vl53l7cx_top.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_8X8);
    if (status) {
        I2C_SENSOR_DBG("vl53l7cx_set_resolution failed");
    }

    status = sensor_vl53l7cx_top.vl53l7cx_set_ranging_frequency_hz(15);
    if (status) {
        I2C_SENSOR_DBG("vl53l7cx_set_ranging_frequency_hz failed");
    }


    status = sensor_vl53l7cx_top.vl53l7cx_set_target_order(VL53L7CX_TARGET_ORDER_CLOSEST);
    if (status) {
        I2C_SENSOR_DBG("vl53l7cx_set_target_order failed");
    }

    status = sensor_vl53l7cx_top.vl53l7cx_get_integration_time_ms(&integration_time_ms);
    if (status) {
       I2C_SENSOR_DBG("vl53l7cx_get_integration_time_ms failed");
    }
    
    // Start Measurements
    sensor_vl53l7cx_top.vl53l7cx_start_ranging();
    
    vl53 = (sVL53Data_t*)malloc(sizeof(sVL53Data_t));
    _mode = 8;
    I2C_SENSOR_DBG("end");
    
    //创建获取传感器数据任务
    xHandle = xTaskCreateStatic(get_data_task, "get_data_task", STACK_SIZE, NULL, configMAX_PRIORITIES - 1, xStack_A, &xTaskBuffer_A);
    xSemaphore = xSemaphoreCreateMutexStatic( &xMutexBuffer );

}

void initSensor_4x4(void){
    uint8_t status;
    uint32_t integration_time_ms;
    if(xHandle){
      sensor_vl53l7cx_top.vl53l7cx_stop_ranging();
      //sensor_vl53l7cx_top.vl53l7cx_i2c_reset();
      //vTaskDelete(xHandle);
      deleteTask = 1;
      free(vl53);
      delay(100);
      xHandle = NULL;
    }
    I2C_SENSOR_DBG("init sensor ...");
    // Configure VL53L7CX satellite component.
    
    //delay(50);
    sensor_vl53l7cx_top.begin();
    sensor_vl53l7cx_top.vl53l7cx_i2c_reset();
    sensor_vl53l7cx_top.init_sensor();
    
    status = sensor_vl53l7cx_top.vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4);
    if (status) {
        I2C_SENSOR_DBG("vl53l7cx_set_resolution failed");
    }

    status = sensor_vl53l7cx_top.vl53l7cx_set_ranging_frequency_hz(60);
    if (status) {
        I2C_SENSOR_DBG("vl53l7cx_set_ranging_frequency_hz failed");
    }



    status = sensor_vl53l7cx_top.vl53l7cx_set_target_order(VL53L7CX_TARGET_ORDER_CLOSEST);
    if (status) {
        I2C_SENSOR_DBG("vl53l7cx_set_target_order failed");
    }
    
    // status = sensor_vl53l7cx_top.vl53l7cx_set_integration_time_ms(5);
    // if (status) {
    //     I2C_SENSOR_DBG("vl53l7cx_set_integration_time_ms failed");
    // }

    status = sensor_vl53l7cx_top.vl53l7cx_get_integration_time_ms(&integration_time_ms);
    I2C_SENSOR_DBG(integration_time_ms);
    if (status) {
       I2C_SENSOR_DBG("vl53l7cx_get_integration_time_ms failed");
    }
      
    // Start Measurements
    sensor_vl53l7cx_top.vl53l7cx_start_ranging();


    vl53 = (sVL53Data_t*)malloc(sizeof(sVL53Data_t));
    _mode = 4;
    I2C_SENSOR_DBG("end");

    
    //创建获取传感器数据任务
    xHandle = xTaskCreateStatic(get_data_task, "get_data_task", STACK_SIZE, NULL, configMAX_PRIORITIES - 1, xStack_A, &xTaskBuffer_A);
    xSemaphore = xSemaphoreCreateMutexStatic( &xMutexBuffer );

}



/**
 * @brief 求取n个射线的和，并且当射线的最大值大于传入参数时，以传入参数为准
 * 
 * @param begin 
 * @param end 
 * @param max 
 * @return uint16_t 
 */
static uint16_t raySum(uint8_t begin, uint8_t end, uint16_t max){
  uint16_t tempSum = 0;
  for(uint8_t i = begin; i< end; i++){
    if(histogram[i] > max){
      tempSum += max;
    }else{
      tempSum += histogram[i];
    }
  }
  return tempSum;
}


/**
 * @brief 求两个射线的差值
 * 
 * @param first 
 * @param second 
 * @return int 
 */
static int rayDiff(uint8_t first, uint8_t second){
  return (int)(histogram[first] - histogram[second]);
}

/**
 * @brief 获取某个数组的某段范围内的最小值
 * 
 * @param array 
 * @param start 
 * @param end 
 * @return int 
 */
static int findMin(uint16_t *array, int start, int end) {
  if (start > end) {
    // 如果起始索引大于结束索引，返回一个大值表示错误
    return -1; 
  }
  
  int minValue = array[start];
  
  for (int i = start + 1; i <= end; i++) {
    if (array[i] < minValue) {
      minValue = array[i];
    }
  }
  return minValue;
}

static void printBinary(uint8_t value) {
  for (int i = 2; i >= 0; i--) {
    if (value & (1 << i)) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
  }
  Serial.println();
}

#if 0

/**
 * @brief 将射线分为多层，进行处理
 * @n 默认值，所有层均可以直行、左转、右转
 * @n 每一层根据实际情况，关闭直行、左转、右转
 */
static uint8_t flootDir07 = 0b111;
static uint8_t flootDir16 = 0b111;
static uint8_t flootDir25 = 0b111;
static uint8_t flootDir34 = 0b111;

/**
 * @brief 平滑数据
 * 
 */
static void SmoothingData(VL53L7CX_ResultsData *result){
  memset(sector_count, 0, sizeof(sector_count));
  memset(histogram, 0, sizeof(histogram));
  for (int i = 3; i <= 5; i++) {
    for (int j = 0; j < 8; j++) {
      float angle = (j - (8 / 2)) * 8.5;
      int sector = (int)(angle / 8.5);
      sector += 4;
      if (sector >= NUM_SECTORS) {
         sector = NUM_SECTORS - 1;
      }
      histogram[sector] += result->distance_mm[i*8 + j];
      sector_count[sector]++;
    }
  }
    for (int i = 0; i < NUM_SECTORS; i++) {
        if (sector_count[i] > 0) {
            histogram[i] /= sector_count[i];
        }
    }
}



static void logicalFloor(VL53L7CX_ResultsData *result){
  flootDir07 = 0b111;
  flootDir16 = 0b111;
  flootDir25 = 0b111;
  flootDir34 = 0b111;
  dirTrend = 0b111;
  if(findMin(histogram,0,7) < wall){//判断8根射线的最小值小于限制，则说明前面全被挡了
    flootDir07 &= 0b101;//禁止直行
  }

  if(findMin(histogram,1,6) < (wall + LINE6MIN)){//判断6根射线的最小值小于限制，则说明6根射线层被挡了
     flootDir16 &= 0b101;//该层禁止直行
     if(histogram[0] < (wall + LINE6MIN)){//禁止左转
        flootDir16 &= 0b011;
     }
     if(histogram[7] < (wall + LINE6MIN)){//禁止右转
        flootDir16 &= 0b110;
     }
  }else{//该层允许直行
    if(histogram[0] < histogram[1] ){//禁止左转
      flootDir16 &= 0b011;
    }
    if(histogram[7] < histogram[6]){//禁止右转
      flootDir16 &= 0b110;
    }
  }

  if(findMin(histogram,2,5) < (wall + LINE4MIN)){//判断4根射线的最小值小于限制，则说明4根射线层被挡了
    flootDir25 &= 0b101;//该层禁止直行
    if((histogram[0] < (wall + LINE4MIN)) || (histogram[1] < (wall + LINE4MIN))){//禁止左转
      flootDir25 &= 0b001;//该层禁止左转
    }
    if((histogram[6] < (wall + LINE4MIN)) || (histogram[7] < (wall + LINE4MIN))){//禁止右转
      flootDir25 &= 0b100;//该层禁止右转
      
    }
  }else{//该层允许直行
    if((histogram[0] < (wall + LINE4MIN)) || (histogram[1] < (wall + LINE4MIN))){//禁止左转
      flootDir25 &= 0b011;//该层禁止左转
    }
    if((histogram[6] < (wall + LINE4MIN)) || (histogram[7] < (wall + LINE4MIN))){//禁止右转
      flootDir25 &= 0b110;//该层禁止右转
    }
  }

  if(findMin(histogram,3,4) < (wall + LINE2MIN)){//判断2根射线的最小值小于限制，则说明2根射线成被挡了
     flootDir34 &= 0b101;//该层禁止直行
    if((histogram[1] < (wall + LINE2MIN)) || (histogram[2] < (wall + LINE2MIN))){//禁止左转
      flootDir34 &= 0b001;//该层禁止左转
    }
    if((histogram[5] < (wall + LINE2MIN)) || (histogram[6] < (wall + LINE2MIN))){//禁止右转
      flootDir34 &= 0b100;//该层禁止右转
    }
  }else{//直行射程范围内没有被挡
    if((histogram[1] < (wall + LINE2MIN)) || (histogram[2] < (wall + LINE2MIN))){//禁止左转
      flootDir34 &= 0b011;//该层禁止左转
    }
    if((histogram[5] < (wall + LINE2MIN)) || (histogram[6] < (wall + LINE2MIN))){//禁止右转
      flootDir34 &= 0b110;//该层禁止右转
    }
  }

  int diff07 = rayDiff(0, 7);
  int diff16 = rayDiff(1, 6);
  if((diff07 > 100) && (diff16 > 100)){//左转趋势
    dirTrend &= 0b110;//关闭右转趋势
    dirTrend1 = dirTrend;
  }else if((diff07 < -100) && (diff16 < -100)){
    dirTrend &= 0b011;//关闭左转趋势
    dirTrend1 = dirTrend;
  }else{
    dirTrend = dirTrend1;
  }
  Serial.print("07:");printBinary(flootDir07);
  Serial.print("16:");printBinary(flootDir16);
  Serial.print("25:");printBinary(flootDir25);
  Serial.print("34:");printBinary(flootDir34);
  Serial.print("FF:");printBinary(dirTrend);
}

/**
 * @brief 逻辑处理，真正的控制命令执行处
 * 
 */
static void logicalProcess(VL53L7CX_ResultsData *result){
  
  if(((flootDir07 >> 1) & (flootDir16 >> 1) & (flootDir25 >> 1) & (flootDir34 >> 1) & 0x01) == 0x01){//所有层都告诉我可以直行
  // int diff07 = rayDiff(0, 7);
  //   if(diff07 > 200){
  //     realDir = RIGHT;
  //     realSpeed = 15;
  //   }else if(diff07 < -200){
  //     realDir = LEFT;
  //     realSpeed = 15;
  //   }else{
      realDir = FORWARD;
    //}
    
    
  }else if((flootDir07 >> 2) & (flootDir16 >> 2) & (flootDir25 >> 2) & (flootDir34 >> 2)){//所有层都告诉我可以左转，那我就左转
    realDir = LEFT;
    realSpeed = 15;
  }else if((flootDir07 & 0x01) & (flootDir16 & 0x01) & (flootDir25 & 0x01) & (flootDir34 & 0x01)){//所有层都告诉我可以右转，那我就右转
    realDir = RIGHT;
    realSpeed = 15;
  }else{
    if(((flootDir07>>1) & 0x01) != 0x01){//该层不可直行，原地左转或右转
      if((dirTrend >> 2)){//左转趋势
        realDir = LEFT;
      }else{
        realDir = RIGHT;
      }
      realSpeed = 0;
    }else if(((flootDir16 >> 1) & 0x01) != 0x01){//该层不可直行
      if((flootDir16 & dirTrend)>>2){//左转趋势同时该层支持左转
        realDir = LEFT;
      }else if((flootDir16 & dirTrend) & 0x01){//右转趋势同时该层支持右转
        realDir = RIGHT;
      }else{
        if(dirTrend >> 2){//左转
          realDir = LEFT;
         }else if(dirTrend &0x01){//右转
            realDir = RIGHT;
         }else{
          realDir = LEFT;
          Serial.println("flootDir16 error");
          }
      }
      LineAferTurn = false;
      realSpeed = 0;
    }else if(((flootDir25>>1) & 0x01) != 0x01){//该层不可直行
      if(LineAferTurn){
        Serial.println("LAT=true");
        realDir = FORWARD;
      }else{
        if(flootDir25 == 0x00 && histogram[3] > 320){
          LineAferTurn = true;
          Serial.println("enable=true");
          //realDir = FORWARD;
        }else{
          if((flootDir25 & dirTrend) >> 2){///左转趋势同时该层支持左转
            realDir = LEFT;
          }else if(((flootDir25 & dirTrend)& 0x01)){////右转趋势同时该层支持右转
            realDir = RIGHT;
          }else{//flootDir25 数据为000 根据趋势做方向判断
            if(dirTrend >> 2){//左转
              realDir = LEFT;
             }else if(dirTrend &0x01){//右转
                realDir = RIGHT;
             }else{
              realDir = LEFT;
              Serial.println("flootDir25 error");
              }
          }
          realSpeed = 15;
        }
      }
    }else if(((flootDir34 >> 1 ) &0x01) != 0x01){//该层不能直行
      if((flootDir34 & dirTrend)>>2){///左转趋势同时该层支持左转
        realDir = LEFT;
      }else if((flootDir34 & dirTrend) & 0x01){//右转趋势同时该层支持右转
        realDir = RIGHT;
      }
    }else{//则按默认直行即可
      realDir = FORWARD;
    }
    
  }
  
}

#else 

#define LINE04MIN     100
#define LINE12MIN     300

/**
 * @brief 将射线分为多层，进行处理
 * @n 默认值，所有层均可以直行、左转、右转
 * @n 每一层根据实际情况，关闭直行、左转、右转
 */
static uint8_t flootDir04 = 0b111;
static uint8_t flootDir12 = 0b111;

/**
 * @brief 平滑数据
 * 
 */
static void SmoothingData(VL53L7CX_ResultsData *result){
  memset(sector_count, 0, sizeof(sector_count));
  memset(histogram, 0, sizeof(histogram));
  for (int i = 1; i <= 2; i++) {
    for (int j = 0; j < 4; j++) {
      float angle = (j - (4 / 2)) * 15;
      int sector = (int)(angle / 15);
      sector += 2;
      if (sector >= 4) {
         sector = 4 - 1;
      }
      histogram[3-sector] += result->distance_mm[i*4 + j];
      sector_count[3-sector]++;
    }
  }
    for (int i = 0; i < 4; i++) {
        if (sector_count[i] > 0) {
            histogram[i] /= sector_count[i];
        }
    }
}


static void logicalFloor(VL53L7CX_ResultsData *result){
  flootDir04 = 0b111;
  flootDir12 = 0b111;
  dirTrend = 0b111;
  if(findMin(histogram,0,3) < LINE04MIN){//判断4根线的最小值小于限制，则说明4根线层被挡住
    flootDir04 &= 0b101;//禁止直行
  }

  if(findMin(histogram,1,2) < wall){//判断2根射线的最小值小于限制，则说明2根射线层被挡了
     flootDir12 &= 0b101;//该层禁止直行
     if(histogram[0] < wall){//禁止左转
        flootDir12 &= 0b011;
     }
     if(histogram[3] < wall){//禁止右转
        flootDir12 &= 0b110;
     }
  }else{//该层允许直行
    if(histogram[0] < histogram[1] ){//禁止左转
      flootDir12 &= 0b011;
    }
    if(histogram[3] < histogram[2]){//禁止右转
      flootDir12 &= 0b110;
    }
  }

  int diff03= rayDiff(0, 3);
  if((diff03 > 100)){//左转趋势
    dirTrend &= 0b110;//关闭右转趋势
    dirTrend1 = dirTrend;
  }else if((diff03 < -100)){
    dirTrend &= 0b011;//关闭左转趋势
    dirTrend1 = dirTrend;
  }else{
    dirTrend = dirTrend1;
  }
  // Serial.print("04:");printBinary(flootDir04);
  // Serial.print("12:");printBinary(flootDir12);
  // Serial.print("FF:");printBinary(dirTrend);
}

/**
 * @brief 逻辑处理，真正的控制命令执行处
 * 
 */
static void logicalProcess(VL53L7CX_ResultsData *result){
  
  if(((flootDir04 >> 1) & (flootDir12 >> 1)  & 0x01) == 0x01){//所有层都告诉我可以直行
    realDir = FORWARD;
  }else if((flootDir04 >> 2) & (flootDir12 >> 2)){//所有层都告诉我可以左转，那我就左转
    realDir = LEFT;
    realSpeed = 15;
  }else if((flootDir04 & 0x01) & (flootDir12 & 0x01)){//所有层都告诉我可以右转，那我就右转
    realDir = RIGHT;
    realSpeed = 15;
  }else{
    if(((flootDir04>>1) & 0x01) != 0x01){//该层不可直行，原地左转或右转
      if((dirTrend >> 2)){//左转趋势
        realDir = LEFT;
      }else{
        realDir = RIGHT;
      }
      realSpeed = 0;
    }else if(((flootDir12 >> 1) & 0x01) != 0x01){//该层不可直行
      if((flootDir12 & dirTrend)>>2){//左转趋势同时该层支持左转
        realDir = LEFT;
      }else if((flootDir12 & dirTrend) & 0x01){//右转趋势同时该层支持右转
        realDir = RIGHT;
      }else{
        if(dirTrend >> 2){//左转
          realDir = LEFT;
         }else if(dirTrend &0x01){//右转
            realDir = RIGHT;
         }else{
          realDir = LEFT;
          Serial.println("flootDir16 error");
          }
      }
      LineAferTurn = false;
      realSpeed = 0;
    }else{//则按默认直行即可
      realDir = FORWARD;
    }
    
  }
  
}


#endif

void getData(void){
    
    uint8_t NewDataReady = 0;
    char report[64];
    uint8_t status;
   
    status = sensor_vl53l7cx_top.vl53l7cx_check_data_ready(&NewDataReady);
    delay(2);
    
    if ((!status) && (NewDataReady != 0)) {
     
      status = sensor_vl53l7cx_top.vl53l7cx_get_ranging_data(&Results);

      if(_getDataMode == 0){//原始数据模式
        if(_mode == 8){//8*8矩阵
          if(_obstacle_threshold > 50){//设置阈值处理
            xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
            for(uint8_t i = 0;i < 64;i++){
              if(Results.distance_mm[i] < _obstacle_threshold){//判断是否小于阈值
                vl53->_8X8Data[i] = 1;
              }else{
                vl53->_8X8Data[i] = 0;
              }
            }
            xSemaphoreGive( xSemaphore );
          }else{
            xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
            memcpy(vl53->_8X8Data,&Results.distance_mm,_mode*_mode*2);//复制数据
            xSemaphoreGive( xSemaphore );
          }
          for(uint8_t i = 0; i < 64;i++){
            Serial.print(Results.distance_mm[i]);
            Serial.print(",");
          }
          Serial.println("end");
        
        }else{
          if(_obstacle_threshold > 50){
            xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
            for(uint8_t i = 0;i < 16;i++){
              if(Results.distance_mm[i] < _obstacle_threshold){
                vl53->_4X4Data[i] = 1;
              }else{
                vl53->_4X4Data[i] = 0;
              }
            }
            xSemaphoreGive( xSemaphore );
          }else{
            xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
            memcpy(vl53->_4X4Data,&Results.distance_mm,_mode*_mode*2);//复制数据
            xSemaphoreGive( xSemaphore );
          }
          for(uint8_t i = 0; i < 8;i++){
            Serial.print(Results.distance_mm[i]);
            Serial.print(",");
          }
          Serial.println("end");
        }
      }else{
        SmoothingData(&Results);//平滑中间数据
        // //拿取平滑后的数据
        LObstacleDistance = histogram[0];
        MObstacleDistance = findMin(histogram,1,2);
        RObstacleDistance =  histogram[3];
        logicalFloor(&Results);
        logicalProcess(&Results);
        Serial.print(realDir);
        Serial.println(realSpeed);
        Serial.println("end");
      }
      
    }
}

