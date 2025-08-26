/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/05/20
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
/*
 *@Note
DHCP example, demonstrating that DHCP automatically obtains an IP address.
For details on the selection of engineering chips,
please refer to the "CH32V30x Evaluation Board Manual" under the CH32V307EVT\EVT\PUB folder.
*/
#include "string.h"
#include "eth_driver.h"

u8 MACAddr[6];                                    //MAC address
u8 IPAddr[4]   = {0, 0, 0, 0};                    //IP address
u8 GWIPAddr[4] = {0, 0, 0, 0};                    //Gateway IP address
u8 IPMask[4]   = {0, 0, 0, 0};                    //subnet mask
u8 DESIP[4]    = {192, 168, 1, 100};              //destination IP address
u16 desport = 1000;                               //destination port
u16 srcport = 1000;                               //source port

u8 SocketId;
u8 SocketRecvBuf[WCHNET_MAX_SOCKET_NUM][RECE_BUF_LEN];  //socket receive buffer
u8 MyBuf[RECE_BUF_LEN];

#define I2C_SPEED              100000
#define I2C_MASTER_ADDRESS     0x3F
#define I2C_PAGE_SIZE          8
#define I2C_TIMEOUT            0x1000

void I2C_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef  I2C_InitStructure = {0};
    
    /* Enable GPIOB, I2C2 and AFIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    
    /* Configure PB10 (SCL) and PB11 (SDA) as AF Open-Drain */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_MASTER_ADDRESS;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
    
    /* Initialize I2C */
    I2C_Init(I2C2, &I2C_InitStructure);
    
    /* Enable I2C */
    I2C_Cmd(I2C2, ENABLE);
}

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
 * @brief   Initializes TIM2.
 *
 * @return  none
 */
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};

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
 * @fn      WCHNET_CreateTcpSocket
 *
 * @brief   Create TCP Socket
 *
 * @return  none
 */
void WCHNET_CreateTcpSocket(void)
{
    u8 i;
    SOCK_INF TmpSocketInf;

    memset((void *) &TmpSocketInf, 0, sizeof(SOCK_INF));
    memcpy((void *) TmpSocketInf.IPAddr, DESIP, 4);
    TmpSocketInf.DesPort = desport;
    TmpSocketInf.SourPort = srcport++;
    TmpSocketInf.ProtoType = PROTO_TYPE_TCP;
    TmpSocketInf.RecvBufLen = RECE_BUF_LEN;
    i = WCHNET_SocketCreat(&SocketId, &TmpSocketInf);
    printf("SocketId %d\r\n", SocketId);
    mStopIfError(i);
    i = WCHNET_SocketConnect(SocketId);                        //make a TCP connection
    mStopIfError(i);
}

/*********************************************************************
 * @fn      WCHNET_DataLoopback
 *
 * @brief   Data loopback function.
 *
 * @param   id - socket id.
 *
 * @return  none
 */
void WCHNET_DataLoopback(u8 id)
{
#if 1
    u8 i;
    u32 len;
    u32 endAddr = SocketInf[id].RecvStartPoint + SocketInf[id].RecvBufLen;       //Receive buffer end address

    if ((SocketInf[id].RecvReadPoint + SocketInf[id].RecvRemLen) > endAddr) {    //Calculate the length of the received data
        len = endAddr - SocketInf[id].RecvReadPoint;
    }
    else {
        len = SocketInf[id].RecvRemLen;
    }
    i = WCHNET_SocketSend(id, (u8 *) SocketInf[id].RecvReadPoint, &len);        //send data
    if (i == WCHNET_ERR_SUCCESS) {
        WCHNET_SocketRecv(id, NULL, &len);                                      //Clear sent data
    }
#else
    u32 len, totallen;
    u8 *p = MyBuf;

    len = WCHNET_SocketRecvLen(id, NULL);                                //query length
    totallen = len;
    WCHNET_SocketRecv(id, MyBuf, &len);                                  //Read the data of the receive buffer into MyBuf
    while(1){
        len = totallen;
        WCHNET_SocketSend(id, p, &len);                                  //Send the data
        totallen -= len;                                                 //Subtract the sent length from the total length
        p += len;                                                        //offset buffer pointer
        if(totallen)continue;                                            //If the data is not sent, continue to send
        break;                                                           //After sending, exit
    }
#endif
}

/*********************************************************************
 * @fn      WCHNET_HandleSockInt
 *
 * @brief   Socket Interrupt Handle
 *
 * @param   socketid - socket id.
 *          intstat - interrupt status
 *
 * @return  none
 */
void WCHNET_HandleSockInt(u8 socketid, u8 intstat)
{
    if (intstat & SINT_STAT_RECV)                               //receive data
    {
        WCHNET_DataLoopback(socketid);                          //Data loopback
    }
    if (intstat & SINT_STAT_CONNECT)                            //connect successfully
    {
        WCHNET_ModifyRecvBuf(socketid, (u32) SocketRecvBuf[socketid], RECE_BUF_LEN);
        printf("TCP Connect Success\r\n");
    }
    if (intstat & SINT_STAT_DISCONNECT)                         //disconnect
    {
        printf("TCP Disconnect\r\n");
    }
    if (intstat & SINT_STAT_TIM_OUT)                            //timeout disconnect
    {
        printf("TCP Timeout\r\n");
        WCHNET_CreateTcpSocket();
    }
}

/*********************************************************************
 * @fn      WCHNET_HandleGlobalInt
 *
 * @brief   Global Interrupt Handle
 *
 * @return  none
 */
void WCHNET_HandleGlobalInt(void)
{
    u8 intstat;
    u16 i;
    u8 socketint;

    intstat = WCHNET_GetGlobalInt();                              //get global interrupt flag
    if (intstat & GINT_STAT_UNREACH)                              //Unreachable interrupt
    {
        printf("GINT_STAT_UNREACH\r\n");
    }
    if (intstat & GINT_STAT_IP_CONFLI)                            //IP conflict
    {
        printf("GINT_STAT_IP_CONFLI\r\n");
    }
    if (intstat & GINT_STAT_PHY_CHANGE)                           //PHY status change
    {
        i = WCHNET_GetPHYStatus();
        if (i & PHY_Linked_Status)
            printf("PHY Link Success\r\n");
    }
    if (intstat & GINT_STAT_SOCKET) {                             //socket related interrupt
        for (i = 0; i < WCHNET_MAX_SOCKET_NUM; i++) {
            socketint = WCHNET_GetSocketInt(i);
            if (socketint)
                WCHNET_HandleSockInt(i, socketint);
        }
    }
}

/*********************************************************************
 * @fn      WCHNET_DHCPCallBack
 *
 * @brief   DHCPCallBack
 *
 * @param   status - status returned by DHCP
 *                   0x00 - Success
 *                   0x01 - Failure
 *          arg - Data returned by DHCP
 *
 * @return  DHCP status
 */
u8 WCHNET_DHCPCallBack(u8 status, void *arg)
{
    u8 *p;
    u8 tmp[4] = {0, 0, 0, 0};

    if(!status)
    {
        p = arg;
        printf("DHCP Success\r\n");
        /*If the obtained IP is the same as the last IP, exit this function.*/
        if(!memcmp(IPAddr, p ,sizeof(IPAddr)))
            return READY;
        /*Determine whether it is the first successful IP acquisition*/
        if(memcmp(IPAddr, tmp ,sizeof(IPAddr))){
            /*The obtained IP is different from the last value,
             * then disconnect the last connection.*/
            WCHNET_SocketClose(SocketId, TCP_CLOSE_NORMAL);
        }
        memcpy(IPAddr, p, 4);
        memcpy(GWIPAddr, &p[4], 4);
        memcpy(IPMask, &p[8], 4);
        printf("IPAddr: %d.%d.%d.%d \r\n", (u16)IPAddr[0], (u16)IPAddr[1],
               (u16)IPAddr[2], (u16)IPAddr[3]);
        printf("GWIPAddr: %d.%d.%d.%d \r\n", (u16)GWIPAddr[0], (u16)GWIPAddr[1],
               (u16)GWIPAddr[2], (u16)GWIPAddr[3]);
        printf("IPMask: %d.%d.%d.%d \r\n", (u16)IPMask[0], (u16)IPMask[1],
               (u16)IPMask[2], (u16)IPMask[3]);
        printf("DNS1: %d.%d.%d.%d \r\n", p[12], p[13], p[14], p[15]);
        printf("DNS2: %d.%d.%d.%d \r\n", p[16], p[17], p[18], p[19]);
        WCHNET_CreateTcpSocket();                                                   //Create a TCP connection
        return READY;
    }
    else
    {
        printf("DHCP Fail %02x \r\n", status);
        /*Determine whether it is the first successful IP acquisition*/
        if(memcmp(IPAddr, tmp ,sizeof(IPAddr))){
            /*The obtained IP is different from the last value*/
            WCHNET_SocketClose(SocketId, TCP_CLOSE_NORMAL);
        }
        return NoREADY;
    }
}

uint8_t I2C_CheckDevice(uint8_t slaveAddr)
{
    uint32_t timeout = I2C_TIMEOUT;
    uint8_t status = 0;
    
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) && timeout--);
    if(timeout == 0) return 1;
    
    I2C_GenerateSTART(I2C2, ENABLE);
    
    timeout = I2C_TIMEOUT;
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) && timeout--);
    if(timeout == 0) return 1;
    
    I2C_Send7bitAddress(I2C2, slaveAddr << 1, I2C_Direction_Transmitter);
    
    timeout = I2C_TIMEOUT;
    while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_ADDR) && timeout--)
    {
        if(I2C_GetFlagStatus(I2C2, I2C_FLAG_AF))
        {
            /* No ACK received - device not present */
            I2C_ClearFlag(I2C2, I2C_FLAG_AF);
            status = 1;
            break;
        }
    }
    
    if(timeout == 0) status = 1;
    
    I2C_GenerateSTOP(I2C2, ENABLE);
    
    return status;
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
    uint8_t led_state = 0;
    uint32_t blink_counter = 0;
    uint32_t blink_interval = 10;

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);                                            //USART initialize

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin   = GPIO_Pin_12;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;   // push-pull output
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &gpio);

    GPIO_SetBits(GPIOB, GPIO_Pin_12);      // OFF (sink config: high = off)

    printf("DHCP Test\r\n");  	
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
    printf("net version:%x\n", WCHNET_GetVer());
    if( WCHNET_LIB_VER != WCHNET_GetVer()){
        printf("version error.\n");
    }
    I2C_Config();
    WCHNET_GetMacAddr(MACAddr);                                           //get the chip MAC address
    printf("mac addr:");
    for(i = 0; i < 6; i++) 
        printf("%x ", MACAddr[i]);
    printf("\n");
    TIM2_Init();
    WCHNET_DHCPSetHostname("WCHNET");                                     //Configure DHCP host name
    i = ETH_LibInit(IPAddr,GWIPAddr,IPMask,MACAddr);                      //Ethernet library initialize
    mStopIfError(i);
    if(i == WCHNET_ERR_SUCCESS)
        printf("WCHNET_LibInit Success\r\n");
    WCHNET_DHCPStart(WCHNET_DHCPCallBack);                                //Start DHCP

    printf("\r\nScanning I2C bus...\r\n");
    uint8_t slaveCount= 0;
    for(i = 0x10; i < 0x78; i++)
    {
        if(I2C_CheckDevice(i) == 0)
        {
            printf("Device found at address: 0x%02X\r\n", i);
            slaveCount++;
        }
    }
    printf("Total devices found: %d\r\n", slaveCount);

    while(1)
    {
        /*Ethernet library main task function,
         * which needs to be called cyclically*/
        WCHNET_MainTask();
        /*Query the Ethernet global interrupt,
         * if there is an interrupt, call the global interrupt handler*/
        if(WCHNET_QueryGlobalInt())
        {
            WCHNET_HandleGlobalInt();
        }

                blink_counter++;
        if(blink_counter >= blink_interval)
        {
            blink_counter = 0;
            led_state = !led_state;
            
            /* In sink mode: LOW = LED ON, HIGH = LED OFF */
            if(led_state)
            {
                GPIO_ResetBits(GPIOB, GPIO_Pin_12);  // Pull low - LED ON
            }
            else
            {
                GPIO_SetBits(GPIOB, GPIO_Pin_12);    // Release high - LED OFF
            }
        }
        
    }
}
