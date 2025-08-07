/*
 *  ethernet_tsk.h
 *  
 *  Created on: Aug 1, 2014
 *  Author: loywong
 */

#ifndef ETHERNET_TSK_H_
#define ETHERNET_TSK_H_

#define ETH_NOTIFICATION_DEAL_VALUE 0x1000
#define ETH_NOTIFICATION_SEND_VALUE 0x2000

#define CMD_BUF_SIZE 256
#define SEND_BUF_SIZE 1024
#define RECV_BUF_SIZE 1024
#define ETH_DATA_PORT 5000
#define ETH_CMD_PORT 5001
#define DATA_SN 0
#define CMD_SN 1
#define LEN_IDX 255

#define VNAME(name) (#name)
#define STRVAL(str, name) (str##name)

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
typedef enum
{
    UARTCMD,
    ETHCMD,
} cmdType;

/*
* main and assist board cmd val
*/
// 依次为： 舵轮运行状态，舵轮目标速度(dr1)，舵轮目标位置(dr2),是否启用主丝杠夹紧(main,dr3),一臂二臂目标夹角(assist，dr3)
const char MAIN_ASSIST_CMD_NAME[5][13] = {"state", 
                                        "tar_v", 
                                        "tar_p", 
                                        "tar_angle", 
                                        "tar_spring"};
const char MAIN_ASSIST_CMD_TYPE_RECORD[5] = {1, 2, 2, 2, 2};
const char MAIN_ASSIST_CMD_MEMBER_NUM = 5;

// * main and assist board back cal
// 依次为：舵轮运行状态，   舵轮目标速度(dr1)，舵轮实际速度(dr1)
//                        舵轮目标位置(dr2)，舵轮实际位置(dr2)
//                        两臂目标夹角，两臂实际夹角
//                        主丝杠目标夹紧状态，主丝杠目前夹紧值（1左2右）
const char MAIN_ASSIST_VAL_NAME[13][13] = { "state", 
                                            "real1_v", 
                                            "real2_v",
                                            "real_p", 
                                            "real_angle",
                                            "real_s1",
                                            "real_s2",
                                            "odom_axis",
                                            "odom_cir" , 
                                            "cur_1",    // 轮 舵 机构 三个电机的实际电流
                                            "cur_2",
                                            "cur_3",
                                            "mech_pos" // 机构电机的实际位置
                                        };
const char MAIN_ASSIST_VAL_TYPE_RECORD[13] = {1, 
                                            2, 2,
                                            2, 2, 
                                            2, 2, 
                                            2, 2, 
                                            2  , 2, 2,
                                            2
                                        };
const char MAIN_ASSIST_VAL_MEMBER_NUM = 13;

// * 推杆控制板的控制指令
const char PUSH_CMD_NAME[3][13] = {
    "f_length",
    "b_length",
    "m_length"};
const char PUSH_CMD_TYPE_RECORD[3] = {2, 2, 2};
const char PUSH_CMD_MEMBER_NUM = 3;
// * 推杆控制板的反馈值
const char PUSH_VAL_NAME[3][13] = {
    "f_length",
    "b_length",
    "m_length"};
const char PUSH_VAL_TYPE_RECORD[3] = {2, 2, 2};
const char PUSH_VAL_MEMBER_NUM = 3;


typedef struct
{
    uint64_t ethTs;
    uint32_t localTs;
} TSRecord;

namespace TskEth
{
    extern uint8_t type;
    // extern QueueHandle_t steerCmdQueue;
    // extern QueueHandle_t steerValQueue;
    //* 用于主、辅助控制板
    extern QueueHandle_t mainAssistCmdQueue; // 用于缓存解包好的主驱动轮和辅助驱动轮控制指令
    extern QueueHandle_t mainAssistValQueue; // 用于缓存等待打包的主驱动轮和辅助驱动轮反馈值
    // * 用于推杆控制板
    extern QueueHandle_t pushCmdQueue; // 用于缓存解包好的推杆控制指令
    extern QueueHandle_t pushValQueue; // 用于缓存等待打包的推杆反馈值

    // * 用于接收数据的队列   
    extern QueueHandle_t  rawDataQueue;     // 网络接收到，等待解包的数据流
    extern QueueHandle_t  sendDataQueue;    // 已经打包好，等待网络发送的数据流
    void Init();
};

#endif /* __cplusplus */

#endif