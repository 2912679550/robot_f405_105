#include "ethernet_tsk.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "w5500_dev.h"
#include "physparams.h"
#include "cmd_tsk.h"

uint8_t rxBuf[RECV_BUF_SIZE];
uint8_t txBuf[SEND_BUF_SIZE];
uint8_t rxDealBuf[RECV_BUF_SIZE];
uint8_t txDealBuf[SEND_BUF_SIZE];

volatile TSRecord sysTs;
SemaphoreHandle_t xMutex;

uint16_t pack_struct_short(void *p, const char name[][13], const char *type, uint8_t len, uint16_t prefix)
{
    memset(txDealBuf + prefix, 0, SEND_BUF_SIZE - prefix);
    for (int i = 0; i < len; i++)
    {
        volatile char type_idx = type[i];
        if (type_idx == 1)
        {
            //            sprintf((char *)txDealBuf + prefix, "%s:", name[i]);
            //            prefix += strlen(name[i])+1;
            memcpy(txDealBuf + prefix, (void *)p, 4);
            txDealBuf[prefix + 4] = ' ';
            prefix += 5;
        }
        else if (type_idx == 2)
        {
            //            sprintf((char *)txDealBuf + prefix, "%s:", name[i]);
            //            prefix += strlen(name[i])+1;
            memcpy(txDealBuf + prefix, (void *)p, 4);
            txDealBuf[prefix + 4] = ' ';
            prefix += 5;
        }
        else if (type_idx == 3)
        {
            uint64_t ts = sysTs.ethTs + (xTaskGetTickCount() - sysTs.localTs) * 1000;
            //            sprintf((char *)txDealBuf + prefix, "%s:", name[i]);
            //            prefix += strlen(name[i])+1;
            memcpy(txDealBuf + prefix, (void *)&ts, 8);
            txDealBuf[prefix + 8] = ' ';
            prefix += 9;
        }
        p = ((char *)p + (type[i] == 3 ? 8 : 4));
    }
    txDealBuf[prefix] = '\r';
    txDealBuf[prefix + 1] = '\n';
    prefix += 2;
    return prefix;
}

/* 
@brief: 将结构体的数据打包成一个字符串格式的字节数组
@param: void *p: 指向要打包的结构体数据的指针
@param: const char name[][13]: 结构体成员的名称,每个名称的最大长度为 13 个字符
@param: const char *type: 结构体成员的类型, 1-uint32_t; 2-float；3-uint64_t
@param: uint8_t len: 结构体成员的个数
@param: uint16_t prefix: 打包的起始位置
*/
void pack_struct(void *p, const char name[][13], const char *type, uint8_t len, uint16_t prefix)
{
    char var_name[13];
    memset(txDealBuf + prefix, 0, SEND_BUF_SIZE - prefix);
    memset(var_name, 0, 13);
    for (int i = 0; i < len; i++)
    {
        prefix = strlen((char *)txDealBuf);
        if (name == NULL)
        {
            var_name[0] = (char)((i % 10) + '0');
            var_name[1] = i / 10 ? (char)((i / 10) + '0') : 0;
        }
        else
        {
            memcpy(var_name, name[i], 13);
        }
        switch (type[i])
        {
        case 1:
            sprintf((char *)txDealBuf + prefix, "%s:%u ", var_name, *((uint32_t *)p));
            break;
        case 2:
            sprintf((char *)txDealBuf + prefix, "%s:%.2f ", var_name, *((float *)p));
            break;
        case 3:
            uint64_t ts = sysTs.ethTs + (xTaskGetTickCount() - sysTs.localTs) * 1000;
            sprintf((char *)txDealBuf + prefix, "%s:%llu ", var_name, ts);
            break;
        }
        p = ((char *)p + (type[i] == 3 ? 8 : 4));
    }
    prefix = strlen((char *)txDealBuf);
    txDealBuf[prefix] = '\r';
    txDealBuf[prefix + 1] = '\n';
    return;
}

void unpack_to_struct(char *frame, void **p, const char name[][13], const char *type, const uint8_t len)
{
    const char *d = " ";
    bool exit = false;
    char *val, *cmd;
    uint16_t prefix = 0;
    char *substr = strstr((char *)frame, name[0]);
    while (!exit)
    {
        cmd = strtok((char *)substr, d);
        // ts is uint64_t, its length is 16, so MUST give >= 16;
        char cmd_s[2][17];
        while (cmd != NULL)
        {
            sscanf(cmd, "%[^:]:%s", cmd_s[0], cmd_s[1]);
            prefix = 0;
            for (int i = 0; i < len; i++)
            {
                val = ((char *)*p + prefix);
                prefix += (type[i] == 3 ? 8 : 4);
                if (!strcmp(cmd_s[0], name[i]))
                {
                    switch (type[i])
                    {
                    case 1:
                        sscanf(cmd_s[1], "%u", (uint32_t *)val);
                        break;
                    case 2:
                        sscanf(cmd_s[1], "%f", (float *)val);
                        break;
                    case 3:
                        sscanf(cmd_s[1], "%llu", (uint64_t *)val);
                        sysTs.ethTs = *(uint64_t *)val;
                        sysTs.localTs = xTaskGetTickCount();
                        break;
                    }
                    break;
                }
            }
            cmd = strtok(NULL, d);
        }
        exit = true;
    }
}
extern wiz_NetInfo cmdEthInfo;
namespace TskEth
{
    const int tskStkSize = 512; // 512
    uint8_t type = BORAD_TYPE::idle;
    SteerCmd *steerCmd = nullptr;
    SteerRunValue *steerVal = nullptr;
    SteerCurInfo *steerCurCmd = nullptr;
    SteerCurInfo *steerCurVal = nullptr;

    QueueHandle_t steerCmdQueue;    // 用于缓存板子接收到的数据
    QueueHandle_t steerValQueue;    // 用于缓存板子向上位机发送的数据

    QueueHandle_t rawDataQueue;
    QueueHandle_t sendDataQueue;

    void ethDealTask(void *pvParameters)
    {
        BaseType_t rtn;
        int32_t ret;
        uint16_t size = 0;
        uint8_t sn = 0;

        while (true)
        {
            rtn = xSemaphoreTake(ethDealTickSem, ethPeriod + 1);
            configASSERT(rtn);

            // TODO 解包接收到的数据
            if (pdPASS == xQueueReceive(rawDataQueue, rxDealBuf, 0))
            {
                if (type == BORAD_TYPE::MAIN_BOARD)
                {
                    void *p = steerCmd;
                    unpack_to_struct(   (char *)rxDealBuf, &p, SteerCmdName,
                                        (const char *)SteerCmdTypeRecord, SteerCmdMemberNum);
                    if (pdFAIL == xQueueOverwrite(steerCmdQueue, steerCmd))
                    {
                        print((char *)("steerCmd send error\r\n"));
                    }
                }else if (type == BORAD_TYPE::PUSH_BOARD){

                }
            }

            uint16_t cur_prefix = 0;
            // TODO 打包并发送数据
            memset(txDealBuf, 0, SEND_BUF_SIZE);
            if (type == BORAD_TYPE::MAIN_BOARD)
            {
                xQueueReceive(steerValQueue, steerVal, 0);
                void *p = steerVal;
                // * pack_struct函数的作用是将结构体的数据打包成一个字符串格式的字节数组，以便于传输或存储。
                // * 输入参数： 
                pack_struct(p, SteerValName, (const char *)SteerValTypeRecord, SteerValMemberNum, 0);
            }
            // txDealBuf[LEN_IDX] = cur_prefix;
            xQueueSend(sendDataQueue, txDealBuf, 1);
        }  
    }

    void Init()
    {
        BaseType_t rtn;
        
        if (type == BORAD_TYPE::MAIN_BOARD)
        {
            steerCmd = (SteerCmd *)pvPortMalloc(sizeof(SteerCmd));
            if (steerCmd == nullptr)
                return;

            steerVal = (SteerRunValue *)pvPortMalloc(sizeof(SteerRunValue));
            if (steerVal == nullptr)
                return;

            steerCurCmd = (SteerCurInfo *)pvPortMalloc(sizeof(SteerCurInfo));
            if (steerCurCmd == nullptr)
                return;

            steerCurVal = (SteerCurInfo *)pvPortMalloc(sizeof(SteerCurInfo));
            if (steerCurVal == nullptr)
                return;

            steerCmdQueue = xQueueCreate(1, sizeof(SteerCmd));
            configASSERT(steerCmdQueue);

            steerValQueue = xQueueCreate(1, sizeof(SteerRunValue));
            configASSERT(steerValQueue);
        }


        // 用于接收数据的队列
        rawDataQueue = xQueueCreate(2, RECV_BUF_SIZE);
        configASSERT(rawDataQueue);

        // 用于发送数据的队列
        sendDataQueue = xQueueCreate(1, SEND_BUF_SIZE);
        configASSERT(sendDataQueue);

        xMutex = xSemaphoreCreateMutex();
        configASSERT(xMutex != NULL);
        
        rtn = xTaskCreate(ethDealTask, (const portCHAR *)"ethDealTask",
                          tskStkSize, NULL, osPriorityBelowNormal, NULL);
        configASSERT(rtn == pdPASS);
    }
}
