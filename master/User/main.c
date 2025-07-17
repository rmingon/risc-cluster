/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/17
 * Description        : Master Cluster Controller for CH32V307 with Ethernet and I2C
 *                      Controls CH32V006 slaves via I2C and provides network interface
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "debug.h"
#include "string.h"
#include "eth_driver.h"
#include "wchnet.h"

/* Reduced buffer sizes for memory optimization */
#define MAX_SLAVES                      8       // Reduced from 16
#define SLAVE_RESPONSE_TIMEOUT          100     // ms
#define CLUSTER_STATUS_PORT             8080
#define CLUSTER_CONTROL_PORT            8081

/* I2C Configuration */
#define I2C_SPEED                       100000  // 100kHz
#define SLAVE_BASE_ADDR                 0x50    // Base address for slaves
#define I2C_TIMEOUT                     1000    // ms

/* Network Settings */
u8 MACAddr[6];
u8 IPAddr[4] = {192, 168, 1, 100};
u8 GWIPAddr[4] = {192, 168, 1, 1};
u8 IPMask[4] = {255, 255, 255, 0};

/* Socket Management - Reduced for memory efficiency */
u8 StatusSocketId;      // TCP server for status monitoring  
u8 ControlSocketId;     // TCP server for control commands

u8 SocketRecvBuf[2][RECE_BUF_LEN];  // Only 2 sockets instead of max
u8 MyBuf[RECE_BUF_LEN];

/* Cluster Management */
typedef struct {
    u8 address;
    u8 online;
    u8 status;
    u8 firmware_version;
    u32 last_heartbeat;
    u8 error_count;
} SlaveNode;

typedef struct {
    u8 cmd;
    u8 slave_addr;
    u8 data_len;
    u8 data[32];
    u16 checksum;
} ClusterMessage;

typedef struct {
    u8 total_slaves;
    u8 online_slaves;
    u8 cluster_status;
    u32 uptime;
    SlaveNode slaves[MAX_SLAVES];
} ClusterStatus;

/* Command definitions */
#define CMD_PING                0x01
#define CMD_GET_STATUS          0x02
#define CMD_SET_CONFIG          0x03
#define CMD_UPDATE_FIRMWARE     0x04
#define CMD_RESET_SLAVE         0x05
#define CMD_BROADCAST           0x06
#define CMD_GET_CLUSTER_STATUS  0x07
#define CMD_SET_SLAVE_ADDR      0x08

/* Status definitions */
#define STATUS_IDLE             0x00
#define STATUS_BUSY             0x01
#define STATUS_ERROR            0x02
#define STATUS_UPDATING         0x03

/* Global variables */
ClusterStatus cluster_status;
u32 system_tick = 0;
u8 cluster_config_mode = 0;

/*********************************************************************
 * @fn      mStopIfError
 *
 * @brief   check if error.
 *
 * @param   iError - error constants.
 *
 * @return  none
 */
void mStopIfError(u8 iError)
{
    if (iError == WCHNET_ERR_SUCCESS)
        return;
    printf("Error: %02X\r\n", (u16) iError);
}

/*********************************************************************
 * @fn      TIM2_Init
 *
 * @brief   Initializes TIM2 for 1ms system tick.
 *
 * @return  none
 */
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0 };

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = SystemCoreClock / 1000000;
    TIM_TimeBaseStructure.TIM_Prescaler = WCHNETTIMERPERIOD * 1000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM2, ENABLE);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    NVIC_SetPriority(TIM2_IRQn, 0x80);
    NVIC_EnableIRQ(TIM2_IRQn);
}

/*********************************************************************
 * @fn      I2C_Configuration
 *
 * @brief   Configure I2C for slave communication
 *
 * @return  none
 */
void I2C_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef I2C_InitTSturcture = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // Configure I2C pins (PB6 - SCL, PB7 - SDA)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // Configure I2C
    I2C_InitTSturcture.I2C_ClockSpeed = I2C_SPEED;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = 0x02;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitTSturcture);

    I2C_Cmd(I2C1, ENABLE);
}

/* ETH_LibInit is now provided by eth_driver_10M.c - removed duplicate */

/*********************************************************************
 * @fn      WCHNET_CreateTcpServer
 *
 * @brief   Create TCP server for cluster management
 *
 * @param   port - server port
 * @param   socketid - pointer to socket id
 *
 * @return  none
 */
void WCHNET_CreateTcpServer(u16 port, u8 *socketid)
{
    u8 i;
    SOCK_INF TmpSocketInf;

    memset((void *) &TmpSocketInf, 0, sizeof(SOCK_INF));
    TmpSocketInf.SourPort = port;
    TmpSocketInf.ProtoType = PROTO_TYPE_TCP;
    i = WCHNET_SocketCreat(socketid, &TmpSocketInf);
    printf("Server Socket ID %d, Port: %d\r\n", *socketid, port);
    mStopIfError(i);
    i = WCHNET_SocketListen(*socketid);
    mStopIfError(i);
}

/*********************************************************************
 * @fn      WCHNET_HandleSockInt
 *
 * @brief   Socket interrupt handler
 *
 * @param   socketid - socket ID
 * @param   intstat - interrupt status
 *
 * @return  none
 */
void WCHNET_HandleSockInt(u8 socketid, u8 intstat)
{
    if (intstat & SINT_STAT_RECV)
    {
        // Handle received data based on socket
        printf("Data received on socket %d\r\n", socketid);
    }
    if (intstat & SINT_STAT_CONNECT)
    {
        printf("Client connected to socket %d\r\n", socketid);
        if (socketid < 2) {  // Safety check for our reduced array
            WCHNET_ModifyRecvBuf(socketid, (u32) SocketRecvBuf[socketid], RECE_BUF_LEN);
        }
    }
    if (intstat & SINT_STAT_DISCONNECT)
    {
        printf("Client disconnected from socket %d\r\n", socketid);
    }
    if (intstat & SINT_STAT_TIM_OUT)
    {
        printf("Socket %d timeout\r\n", socketid);
    }
}

/*********************************************************************
 * @fn      WCHNET_HandleGlobalInt
 *
 * @brief   Global interrupt handler
 *
 * @return  none
 */
void WCHNET_HandleGlobalInt(void)
{
    u8 intstat;
    u16 i;
    u8 socketint;

    intstat = WCHNET_GetGlobalInt();
    if (intstat & GINT_STAT_UNREACH)
    {
        printf("GINT_STAT_UNREACH\r\n");
    }
    if (intstat & GINT_STAT_IP_CONFLI)
    {
        printf("GINT_STAT_IP_CONFLI\r\n");
    }
    if (intstat & GINT_STAT_PHY_CHANGE)
    {
        i = WCHNET_GetPHYStatus();
        if (i & PHY_Linked_Status)
            printf("PHY Link Success\r\n");
    }
    if (intstat & GINT_STAT_SOCKET) {
        for (i = 0; i < 2; i++) {  // Only check our 2 sockets
            socketint = WCHNET_GetSocketInt(i);
            if (socketint)
                WCHNET_HandleSockInt(i, socketint);
        }
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program
 *
 * @return  none
 */
int main(void)
{
    u8 i;
    
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("CH32V307 Master Cluster Controller\r\n");
    printf("SystemClk: %d\r\n", SystemCoreClock);
    printf("ChipID: %08x\r\n", DBGMCU_GetCHIPID());
    printf("WCHNET Version: %x\r\n", WCHNET_GetVer());
    
    if (WCHNET_LIB_VER != WCHNET_GetVer()) {
        printf("Version error.\r\n");
        while(1);
    }
    
    // Initialize cluster status
    memset(&cluster_status, 0, sizeof(ClusterStatus));
    
    // Initialize I2C for slave communication
    I2C_Configuration();
    printf("I2C Initialized\r\n");
    
    // Get MAC address using proper function
    WCHNET_GetMacAddr(MACAddr);
    printf("MAC Address: ");
    for(i = 0; i < 6; i++) 
        printf("%02x ", MACAddr[i]);
    printf("\r\n");
    
    TIM2_Init();
    i = ETH_LibInit(IPAddr, GWIPAddr, IPMask, MACAddr);  // Use driver function
    mStopIfError(i);
    if (i == WCHNET_ERR_SUCCESS)
        printf("WCHNET_LibInit Success\r\n");
    
    // Configure keep alive
#if KEEPALIVE_ENABLE
    {
        struct _KEEP_CFG cfg;
        cfg.KLIdle = 20000;
        cfg.KLIntvl = 15000;
        cfg.KLCount = 9;
        WCHNET_ConfigKeepLive(&cfg);
    }
#endif
    
    // Create only essential network servers (reduced for memory)
    WCHNET_CreateTcpServer(CLUSTER_STATUS_PORT, &StatusSocketId);
    WCHNET_CreateTcpServer(CLUSTER_CONTROL_PORT, &ControlSocketId);
    
    printf("Cluster Controller Ready\r\n");
    printf("Status Server: %d.%d.%d.%d:%d\r\n", IPAddr[0], IPAddr[1], IPAddr[2], IPAddr[3], CLUSTER_STATUS_PORT);
    printf("Control Server: %d.%d.%d.%d:%d\r\n", IPAddr[0], IPAddr[1], IPAddr[2], IPAddr[3], CLUSTER_CONTROL_PORT);
    
    while(1)
    {
        // Handle network tasks  
        WCHNET_MainTask();
        
        // Handle global interrupts
        if(WCHNET_QueryGlobalInt())
        {
            WCHNET_HandleGlobalInt();
        }
        
        // Add your cluster management tasks here
        // - I2C communication with slaves
        // - Status monitoring
        // - Command processing
        
        Delay_Ms(1);
    }
}

/*********************************************************************
 * @fn      TIM2_IRQHandler
 *
 * @brief   Timer interrupt handler for system tick
 *
 * @return  none
 */
void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        system_tick++;
        WCHNET_TimeIsr(1); // 1ms tick for WCHNET
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}