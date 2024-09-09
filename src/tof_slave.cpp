#include "tof_slave.h"
#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>
#include "sensor.h"
#include "Arduino.h"

#define TWIRE  Wire
#define ERR_CODE_NONE               0x00 ///< 通信正常
#define ERR_CODE_CMD_INVAILED       0x01 ///< 无效命令
#define ERR_CODE_RES_PKT            0x02 ///< 响应包错误
#define IO11 11
#define IO12 12
#define SEND_BUFFER_SIZE  4096


uint16_t recvIndex = 0;

uint8_t readBuf[10];
uint16_t dataLen = 0;
uint16_t readCount = 0;
bool analysis = false;
uint8_t _addr = 0x30;

extern uint16_t _obstacle_threshold;
extern int8_t _getDataMode;
extern sVL53Data_t *vl53;
extern uint8_t _mode;
extern SemaphoreHandle_t xSemaphore;

extern uint8_t realDir;
extern uint8_t realSpeed;
extern uint16_t hedging;//紧急避险
extern uint16_t wall;
extern uint16_t LObstacleDistance;//左侧障碍物距离
extern uint16_t MObstacleDistance;//中间障碍物距离
extern uint16_t RObstacleDistance;//右侧障碍物距离

pSendPacktet_t sendBufferPtr = NULL;

void i2cSlaveInit(void){
  pinMode(IO11, INPUT);
  pinMode(IO12, INPUT);

  uint8_t io11State = digitalRead(IO11);
  uint8_t io12State = digitalRead(IO12);

  if(io11State && io12State){
    _addr = 0x30;
  }else if((!io11State) && io12State){
    _addr = 0x31;
  }else if(io11State && (!io12State)){
    _addr = 0x32;
  }else{
    _addr = 0x33;
  }

  sendBufferPtr = i2cSendCuappEnqueue(SEND_BUFFER_SIZE);
  I2C_SLAVE_DBG(_addr);
  TWIRE.setSDA(16);
  TWIRE.setSCL(17);
  TWIRE.begin(_addr);
  
  //TWIRE.setClock(100000);
  //注册一个I2C接收事件，如果I2C从机接收到主机发来的数据，则调用receiveEvent接收数据
  TWIRE.onReceive(receiveEvent);
  //注册一个I2C发送事件，如果I2C从机接收到主机发来数据请求，则调用sendEvent发送数据
  TWIRE.onRequest(sendEvent);
  
}

void i2c_slave_end(){
  TWIRE.end();
}

void receiveEvent(int len){
  
  while(len){
    switch(recvIndex){
      case 0:  
        if(TWIRE.read() == 0x55){
            readBuf[0] = 0x55;
            recvIndex++;
        }
        readCount = 0;
        len -= 1;
      break;
      case 1:
        dataLen = (TWIRE.read() << 8);
        readBuf[1] = dataLen >> 8;
        recvIndex++;
        len -= 1;
      break;
      case 2:
        dataLen |= TWIRE.read();
        readBuf[2] = dataLen  & 0xff;
        recvIndex++;
        len -= 1;
      break;
      default:
        if(readCount < dataLen){
            readBuf[3+readCount] = TWIRE.read();
            readCount++;
        }else{
            TWIRE.read();
            if(readCount == dataLen){
                analysis = true;
                recvIndex = 0;
            }
        }
      break;
    }
  }
    
}

void sendEvent(void){
  if(sendBufferPtr->flag){
    if(sendBufferPtr->index < sendBufferPtr->total){
      
      noInterrupts();
      TWIRE.write(sendBufferPtr->data[sendBufferPtr->index]);
      interrupts();
      sendBufferPtr->index += 1;
      if(sendBufferPtr->index == sendBufferPtr->total){
        sendBufferPtr->flag = false;
      }
      return;
    } 
  }
  noInterrupts();
  TWIRE.write(0xFF);
  interrupts();
 
}

void i2cloop(){
  if(analysis){
    analysis = false;
    I2C_SLAVE_DBG("i2cloop");
    //if(checkSum(readBuf,8) == readBuf[8]){//验证校验位保证数据正确性
        parseCmdPkt(readBuf);//处理指令
    //}
  }
}

uint8_t checkSum(uint8_t *buf, uint8_t len){
  uint16_t sum = 0;
  uint8_t ret = 0;
  for(uint8_t i = 0; i < len; i++){
    sum += buf[i];
  }
  ret = sum & 0xff;
  return ret;
}


void parseCmdPkt(uint8_t *buf){
  uint8_t errCode = ERR_CODE_NONE;
  I2C_SLAVE_DBG("parseCmdPkt");
  if(buf == NULL){
    I2C_SLAVE_DBG("buf is null or len is zero");
    return;
  }
  I2C_SLAVE_DBG(buf[3]);
  I2C_SLAVE_DBG(dataLen);
  //uint8_t *_buf = (uint8_t*)malloc(sizeof(buf));
  //memcpy(_buf, buf, sizeof(buf));
  switch(buf[3]){ 
    case CMD_SETMODE://模式配置
    errCode = setMode(buf);
    break;
    case CMD_ALLData://获取全部数据
    errCode = getALLData(buf);
    break;
    case CMD_FIXED_POINT://获取指定点数据
    errCode = getFixedPointData(buf);
    break;
    case CMD_LINE://获取指定行列数据
    errCode = getLineData(buf);
    break;
    case CMD_LIST://获取指定列数据
    errCode = getListData(buf);
    break;
    case CMD_AVOID_OBSTACLE://避障返回数据
    errCode = getAvoid(buf);
    break;
    case CMD_CONFIG_AVOID://配置避障
      errCode = configAvoid(buf);
    break;
    case CMD_OBSTACLE_DISTANCE://获取障碍物距离
      errCode = retOD(buf);//返回障碍物距离
    break;
    default:
      I2C_SLAVE_DBG("Invaild command: ");
      errCode = ERR_CODE_CMD_INVAILED;
    break;
    }
    I2C_SLAVE_DBG(errCode);
  if(errCode != ERR_CODE_NONE){
    uint8_t sendData[RESPONSE_ERR_PKT_LEN];
    memset(sendData, 0, sizeof(sendData));
    sendData[RESPONSE_STATUS]   = STATUS_FAILED;
    sendData[RESPONSE_CMD]      = buf[3];
    sendData[RESPONSE_LEN_L]    = 1;
    sendData[RESPONSE_LEN_H]    = 0;
    sendData[RESPONSE_ERR_CODE] = errCode;
    DFRobot_Response(sendData, sizeof(sendData));
  }
  //free(_buf);
}


uint8_t setMode(uint8_t *buf){
  uint8_t *_buf = buf;
  I2C_SLAVE_DBG(_buf[7]);
  if(_buf[7] == 8){
    initSensor_8x8();//初始化8*8矩阵
  }else if(_buf[7] == 4){
    initSensor_4x4();//初始化4*4矩阵
  }

  _obstacle_threshold = _buf[5] | _buf[6] << 8; //定义阈值

  _getDataMode = _buf[4];//获取数据处理模式

  uint8_t sendData[RESPONSE_ERR_PKT_LEN - 1];
  memset(sendData, 0, sizeof(sendData));
  sendData[RESPONSE_STATUS] =  STATUS_SUCCESS;
  sendData[RESPONSE_CMD]    =  _buf[3];
  sendData[RESPONSE_LEN_L]  =  0X00;
  sendData[RESPONSE_LEN_H]  =  0X00;
  DFRobot_Response(sendData, sizeof(sendData));
  return ERR_CODE_NONE;

}

uint8_t getALLData(uint8_t *buf){
  uint8_t *_buf = buf;
  uint8_t dataLen = _mode * _mode * 2;
  uint8_t sendData[RESPONSE_ERR_PKT_LEN - 1 + dataLen];

  memset(sendData, 0, sizeof(sendData));
  sendData[RESPONSE_STATUS] =  STATUS_SUCCESS;
  sendData[RESPONSE_CMD]    =  _buf[3];
  sendData[RESPONSE_LEN_L]  =  dataLen & 0xff;
  sendData[RESPONSE_LEN_H]  =  (dataLen >> 8) & 0xff;

  if(_mode == 8){
    xSemaphoreTake(xSemaphore, ( TickType_t ) portMAX_DELAY );
    memcpy(&sendData[4],vl53->_8X8Data,dataLen);//复制数据
    xSemaphoreGive(xSemaphore);
  }else{  
    xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
    memcpy(&sendData[4],vl53->_4X4Data,dataLen);//复制数据
    xSemaphoreGive( xSemaphore );
  }
  DFRobot_Response(sendData, sizeof(sendData));
  return ERR_CODE_NONE;
  
}

uint8_t getFixedPointData(uint8_t *buf){
  uint8_t *_buf = buf;
  uint8_t sendData[RESPONSE_ERR_PKT_LEN - 1 + 2];
  memset(sendData, 0, sizeof(sendData));
  sendData[RESPONSE_STATUS] =  STATUS_SUCCESS;
  sendData[RESPONSE_CMD]    =  _buf[3];
  sendData[RESPONSE_LEN_L]  =  0X02;
  sendData[RESPONSE_LEN_H]  =  0X00;
  if(_mode == 8){
    xSemaphoreTake(xSemaphore, ( TickType_t ) portMAX_DELAY );
    memcpy(&sendData[4],&vl53->_8X8Data[(buf[5] ) * 8 + (_buf[4])],2);//复制数据
    xSemaphoreGive(xSemaphore);
  }else{  
    xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
    memcpy(&sendData[4],&vl53->_4X4Data[(buf[5] ) * 4 + (_buf[4])],2);//复制数据 计算方法 (Y - 1) * 8 + X
    xSemaphoreGive( xSemaphore );
  }
  DFRobot_Response(sendData, sizeof(sendData));
  return ERR_CODE_NONE;

}

uint8_t getLineData(uint8_t *buf){
  uint8_t *_buf = buf;
  uint8_t dataLen = _mode * 2;
  uint8_t sendData[RESPONSE_ERR_PKT_LEN - 1 + dataLen];

  memset(sendData, 0, sizeof(sendData));
  sendData[RESPONSE_STATUS] =  STATUS_SUCCESS;
  sendData[RESPONSE_CMD]    =  _buf[3];
  sendData[RESPONSE_LEN_L]  =  dataLen & 0xff;
  sendData[RESPONSE_LEN_H]  =  (dataLen >> 8) & 0xff;

  if(_mode == 8){
    xSemaphoreTake(xSemaphore, ( TickType_t ) portMAX_DELAY );
    memcpy(&sendData[4],&vl53->_8X8Data[(buf[4]) * 8],dataLen);//复制数据
    xSemaphoreGive(xSemaphore);
  }else{  
    xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
    memcpy(&sendData[4],&vl53->_4X4Data[(buf[4]) * 4],dataLen);//复制数据
    xSemaphoreGive( xSemaphore );
  }
  DFRobot_Response(sendData, sizeof(sendData));
  return ERR_CODE_NONE;
  
}

uint8_t getListData(uint8_t *buf){
  uint8_t *_buf = buf;
  uint8_t dataLen = _mode * 2;
  uint8_t sendData[RESPONSE_ERR_PKT_LEN - 1 + dataLen];

  memset(sendData, 0, sizeof(sendData));
  sendData[RESPONSE_STATUS] =  STATUS_SUCCESS;
  sendData[RESPONSE_CMD]    =  _buf[3];
  sendData[RESPONSE_LEN_L]  =  dataLen & 0xff;
  sendData[RESPONSE_LEN_H]  =  (dataLen >> 8) & 0xff;

  if(_mode == 8){
    uint8_t *bufdata = (uint8_t*)malloc(dataLen * 8);
    if(bufdata == NULL){
      return ERR_CODE_NONE;
    }
    xSemaphoreTake(xSemaphore, ( TickType_t ) portMAX_DELAY );
    memcpy(bufdata,vl53->_8X8Data,dataLen * 8);//复制数据
    xSemaphoreGive(xSemaphore);
    for(uint8_t i = 0; i < 16;){
      sendData[4+i] = bufdata[((i * 8) + (buf[4] * 2))]; // 0 2 4
      sendData[5+i] = bufdata[((i * 8) + ((buf[4] * 2) + 1))];
      i+=2;
    }
    free(bufdata);
  }else{  
    uint8_t *bufdata = (uint8_t*)malloc(dataLen * 4);
    if(bufdata == NULL){
      return ERR_CODE_NONE;
    }
    xSemaphoreTake( xSemaphore, ( TickType_t ) portMAX_DELAY );
    memcpy(bufdata,vl53->_4X4Data,dataLen * 4);//复制数据
    xSemaphoreGive( xSemaphore );
    for(uint8_t i = 0; i < 8;){
      sendData[4+i] = bufdata[((i * 4) + (buf[4] * 2))]; // 0 2 4 6
      sendData[5+i] = bufdata[((i * 4) + ((buf[4] * 2) + 1))];
      i+=2;
    }
    free(bufdata);
  }
  DFRobot_Response(sendData, sizeof(sendData));
  
  return ERR_CODE_NONE;
} 



pSendPacktet_t i2cSendCuappEnqueue(uint16_t len){
    pSendPacktet_t p;
    p = (pSendPacktet_t)malloc(sizeof(sSendPacktet_t) + len);
    p->flag = 0;
    p->total = len;
    p->index = 0;
    return p;
}

void DFRobot_Response(uint8_t *data, uint16_t len){
  i2cSend(data, len);
}

void i2cSend(uint8_t *data, uint16_t len){
  if(sendBufferPtr->flag){
    sendBufferPtr->flag = false;
  }

  if(len <= 4096){
    memset(sendBufferPtr->data, 0, 4069);
    memcpy(sendBufferPtr->data, data, len);
    sendBufferPtr->index = 0;
    sendBufferPtr->total = len;
    sendBufferPtr->flag = true;
  }
}

uint8_t getAvoid(uint8_t *buf){
  uint8_t *_buf = buf;
  uint8_t dataLen = 2;
  uint8_t sendData[RESPONSE_ERR_PKT_LEN - 1 + dataLen];
  memset(sendData, 0, sizeof(sendData));
  sendData[RESPONSE_STATUS] =  STATUS_SUCCESS;
  sendData[RESPONSE_CMD]    =  _buf[3];
  sendData[RESPONSE_LEN_L]  =  dataLen & 0xff;
  sendData[RESPONSE_LEN_H]  =  (dataLen >> 8) & 0xff;
  sendData[4] = realDir & 0xff;
  sendData[5] = realSpeed & 0xff;
  DFRobot_Response(sendData, sizeof(sendData));
  return ERR_CODE_NONE;

}

uint8_t configAvoid(uint8_t *buf){
  uint8_t *_buf = buf;
  //hedging = buf[4] << 8 | buf[5];//暂时设置为固定值
  wall = _buf[4] << 8 | _buf[5];
  _getDataMode = 1;
  //initSensor_8x8();//初始化8*8矩阵
  initSensor_4x4();
  uint8_t sendData[RESPONSE_ERR_PKT_LEN - 1];
  memset(sendData, 0, sizeof(sendData));
  sendData[RESPONSE_STATUS] =  STATUS_SUCCESS;
  sendData[RESPONSE_CMD]    =  _buf[3];
  sendData[RESPONSE_LEN_L]  =  0X00;
  sendData[RESPONSE_LEN_H]  =  0X00;
  DFRobot_Response(sendData, sizeof(sendData));
  return ERR_CODE_NONE;
}

uint8_t retOD(uint8_t *buf){
  uint8_t *_buf = buf;
  uint8_t dataLen = 6;
  uint8_t sendData[RESPONSE_ERR_PKT_LEN - 1 + dataLen];
  memset(sendData, 0, sizeof(sendData));
  sendData[RESPONSE_STATUS] =  STATUS_SUCCESS;
  sendData[RESPONSE_CMD]    =  _buf[3];
  sendData[RESPONSE_LEN_L]  =  dataLen & 0xff;
  sendData[RESPONSE_LEN_H]  =  (dataLen >> 8) & 0xff;
  sendData[4] = LObstacleDistance & 0xff;
  sendData[5] = (LObstacleDistance >> 8) & 0xff;
  sendData[6] = MObstacleDistance & 0xff;
  sendData[7] = (MObstacleDistance >> 8) & 0xff;
  sendData[8] = RObstacleDistance & 0xff;
  sendData[9] = (RObstacleDistance >> 8) & 0xff;
  DFRobot_Response(sendData, sizeof(sendData));
  return ERR_CODE_NONE;
  }