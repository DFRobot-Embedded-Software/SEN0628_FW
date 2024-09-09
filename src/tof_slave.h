#ifndef TOF_SLAVE_H
#define TOF_SLAVE_H

#include "Arduino.h"
#include "Wire.h"


#define CMD_SETMODE 1
#define CMD_ALLData 2
#define CMD_FIXED_POINT 3
#define CMD_LINE 4
#define CMD_LIST 5
#define CMD_AVOID_OBSTACLE 6
#define CMD_CONFIG_AVOID 7
#define CMD_OBSTACLE_DISTANCE 8

#define CMD_END             CMD_AVOID_OBSTACLE

#define STATUS_SUCCESS      0x53  ///< 响应成功状态   
#define STATUS_FAILED       0x63  ///< 响应成功状态 

#define ERR_CODE_NONE               0x00 ///< 通信正常
#define ERR_CODE_CMD_INVAILED       0x01 ///< 无效命令
#define ERR_CODE_RES_PKT            0x02 ///< 响应包错误
#define ERR_CODE_M_NO_SPACE         0x03 ///< I2C主机内存不够
#define ERR_CODE_RES_TIMEOUT        0x04 ///< 响应包接收超时
#define ERR_CODE_CMD_PKT            0x05 ///< 无效的命令包或者命令不匹配
#define ERR_CODE_SLAVE_BREAK        0x06 ///< 从机故障
#define ERR_CODE_ARGS               0x07 ///< 设置的参数错误
#define ERR_CODE_SKU                0x08 ///< 该SKU为无效SKU，或者传感器通用适配器板(Sensor Universal Adapter Board)不支持
#define ERR_CODE_S_NO_SPACE         0x09 ///< I2C从机内存不够
#define ERR_CODE_I2C_ADRESS         0x0A ///< I2C地址无效
#define ERR_CODE_FILE               0x0B ///<文件系统打开失败

#define  RESPONSE_STATUS              0x00  ///< 响应包状态位
#define  RESPONSE_CMD                 0x01  ///< 响应包命令位
#define  RESPONSE_LEN_L               0x02  ///< 响应包数据长度低字节位
#define  RESPONSE_LEN_H               0x03  ///< 响应包数据长度高字节位
#define  RESPONSE_ERR_CODE            0x04  ///< 响应包错误代码位或数据初始位置
#define  RESPONSE_ERR_PKT_LEN         5     ///< 错误包响应长度


//#define ENABLE_I2C_SLAVE_DBG
#ifdef ENABLE_I2C_SLAVE_DBG
#define I2C_SLAVE_DBG(...) {Serial.print("i2c_slave ");Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define I2C_SLAVE_DBG(...)
#endif


struct sSendPacktet{
    //struct sSendPacktet *next;
    uint8_t flag;   /**< 判定一个反应包是否发送完成 true:可以发送*/
    uint16_t total; /**<总长度*/
    uint16_t index;///< 命令包长度索引
    uint8_t data[0];/**< 0长度数组，存储数据*/
};

  typedef struct sSendPacktet sSendPacktet_t;
  typedef struct sSendPacktet *pSendPacktet_t;

 /**
   * @brief 初始化I2C
   */
  void i2cSlaveInit(void);

  /**
 * @brief 初始化串口从机
 * 
 */
  void initSerialSlave(void);
  
  void receiveEvent(int len);
  void sendEvent(void);

  /**
   * @fn checksum
   * @brief 计算校验和
   * @param buf 接收数据
   * @param len 需要计算数据长度 
  */
  uint8_t checkSum(uint8_t *buf, uint8_t len);

/**
 * @fn parseCmdPkt
 * @brief 解析命令包
 * 
 * @param buf 指向pCmdPacktet_t命令包
 * @return uint8_t 错误代码
 */
  void parseCmdPkt(uint8_t *buf);


  

 /**
   * @brief I2C 从机接收数据
   */
  void i2cloop();

  /**
   * @brief 设置基础属性
   */
  uint8_t setMode(uint8_t *buf);

  /**
   * @brief 获取全部数据
   */
  uint8_t getALLData(uint8_t *buf);

  /**
   * @brief 获取指点数据 
   */
  uint8_t getFixedPointData(uint8_t *buf);

  /**
   * @brief 获取指定行数据
   */
  uint8_t getLineData(uint8_t *buf);

   /**
   * @brief 获取指定列数据
   */
  uint8_t getListData(uint8_t *buf);

  /**
   * @brief 返回避障数据
   */
  uint8_t getAvoid(uint8_t *buf);
  /**
   * @breif 配置避障
   */
  uint8_t configAvoid(uint8_t *buf);
  /**
   * @breif 返回障碍物位置原始数据
   */
  uint8_t retOD(uint8_t *buf);



  /**
   * @fn DFRobot_Response
   * @brief 响应包打包
   * 
   * @param data 数据缓存
   * @param len  数据长度
   */
  void DFRobot_Response(uint8_t *data, uint16_t len);

  pSendPacktet_t i2cSendCuappEnqueue(uint16_t len);

  /**
   * @fn i2cSend
   * @brief I2C发送包打包
   * 
   * @param data 数据缓存
   * @param len  数据长度
   */
  void i2cSend(uint8_t *data, uint16_t len);
  



#endif
