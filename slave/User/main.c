
 /*
 * Address | Description           | R/W | Default
 * --------|----------------------|-----|--------
 * 0x00    | Command Register     | W   | 0x00
 * 0x01    | Status Register      | R   | 0xAA
 * 0x02    | Version High         | R   | 0x10
 * 0x03    | Version Low          | R   | 0x01
 * 0x04    | Config Register      | R/W | 0x00
 * 0x10-0x8F| Data Buffer (128B)   | R/W | 0x00
 * 
 * Command Register Values:
 * 0x01 - Reset
 * 0x02 - Get Status
 * 0x03 - Enter Bootloader
 * 0x10 - Write Flash Page
 * 0x11 - Read Flash Page
 * 0x20 - Erase Flash Page
 */

#include "debug.h"

#define I2C_SLAVE_ADDRESS      0x10
#define I2C_BUFFER_SIZE        128 

typedef enum {
    I2C_STATE_IDLE = 0,
    I2C_STATE_ADDR_MATCH,
    I2C_STATE_RX_REG,
    I2C_STATE_RX_DATA,
    I2C_STATE_TX_DATA
} I2C_State_t;

volatile uint8_t i2c_registers[I2C_BUFFER_SIZE] = {0}; 
volatile uint8_t i2c_reg_addr = 0;
volatile I2C_State_t i2c_state = I2C_STATE_IDLE;
volatile uint8_t i2c_rx_complete = 0;
volatile uint8_t i2c_tx_complete = 0;

void I2C_Slave_Config(void);
void I2C_Slave_IRQHandler(void);
uint8_t I2C_GetRegister(uint8_t addr);
void I2C_SetRegister(uint8_t addr, uint8_t value);
void I2C_ProcessCommand(void);

void I2C_Slave_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef  I2C_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_I2C1, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS << 1;  // 7-bit address shifted
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;  // Up to 400kHz supported
    
    I2C_Init(I2C1, &I2C_InitStructure);
    
    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    
    I2C_Cmd(I2C1, ENABLE);
    
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

/*
 * I2C Event Interrupt Handler
 */
void I2C1_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_EV_IRQHandler(void)
{
    uint32_t event = I2C_GetLastEvent(I2C1);
    
    switch(event)
    {
        case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
            i2c_state = I2C_STATE_ADDR_MATCH;
            break;
            
        case I2C_EVENT_SLAVE_BYTE_RECEIVED:
            if(i2c_state == I2C_STATE_ADDR_MATCH)
            {
                i2c_reg_addr = I2C_ReceiveData(I2C1);
                i2c_state = I2C_STATE_RX_DATA;
            }
            else if(i2c_state == I2C_STATE_RX_DATA)
            {
                if(i2c_reg_addr < I2C_BUFFER_SIZE)
                {
                    i2c_registers[i2c_reg_addr] = I2C_ReceiveData(I2C1);
                    i2c_reg_addr++; /* Auto-increment for sequential writes */
                }
                else
                {
                    I2C_ReceiveData(I2C1);
                }
            }
            break;
            
        case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
            i2c_state = I2C_STATE_TX_DATA;
            if(i2c_reg_addr < I2C_BUFFER_SIZE)
            {
                I2C_SendData(I2C1, i2c_registers[i2c_reg_addr]);
                i2c_reg_addr++;
            }
            else
            {
                I2C_SendData(I2C1, 0xFF);
            }
            break;
            
        case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
            if(i2c_reg_addr < I2C_BUFFER_SIZE)
            {
                I2C_SendData(I2C1, i2c_registers[i2c_reg_addr]);
                i2c_reg_addr++; /* Auto-increment */
            }
            else
            {
                I2C_SendData(I2C1, 0xFF);
            }
            break;
            
        case I2C_EVENT_SLAVE_STOP_DETECTED:
            if(i2c_state == I2C_STATE_RX_DATA)
            {
                i2c_rx_complete = 1;
            }
            else if(i2c_state == I2C_STATE_TX_DATA)
            {
                i2c_tx_complete = 1;
            }
            i2c_state = I2C_STATE_IDLE;
            I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
            break;
            
        default:
            break;
    }
}

void I2C1_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_ER_IRQHandler(void)
{
    if(I2C_GetITStatus(I2C1, I2C_IT_AF))
    {
        /* Acknowledge failure - master sent NACK */
        I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
        i2c_state = I2C_STATE_IDLE;
    }
    
    if(I2C_GetITStatus(I2C1, I2C_IT_BERR))
    {
        /* Bus error */
        I2C_ClearITPendingBit(I2C1, I2C_IT_BERR);
        i2c_state = I2C_STATE_IDLE;
    }
    
    if(I2C_GetITStatus(I2C1, I2C_IT_OVR))
    {
        /* Overrun/Underrun */
        I2C_ClearITPendingBit(I2C1, I2C_IT_OVR);
        i2c_state = I2C_STATE_IDLE;
    }
}

uint8_t I2C_GetRegister(uint8_t addr)
{
    if(addr < I2C_BUFFER_SIZE)
        return i2c_registers[addr];
    return 0xFF;
}

void I2C_SetRegister(uint8_t addr, uint8_t value)
{
    if(addr < I2C_BUFFER_SIZE)
        i2c_registers[addr] = value;
}

void I2C_ProcessCommand(void)
{
    uint8_t cmd = i2c_registers[0x00];
    
    switch(cmd)
    {
        case 0x01:
            printf("Reset command received\r\n");
            NVIC_SystemReset();
            break;
            
        case 0x02:
            i2c_registers[0x01] = 0xAA; 
            break;
            
        case 0x03:
            printf("Bootloader mode requested\r\n");
            break;
            
        case 0x10:  /* Example: Write flash page */
            /* TODO: Flash programming would go here */
            /* Data would be in registers 0x10-0x8F (128 bytes) */
            break;
            
        default:
            break;
    }
    
    i2c_registers[0x00] = 0x00;
}

int main(void)
{
    uint8_t i;
    uint8_t led_state = 0;
    uint32_t blink_counter = 0;
    uint32_t blink_interval = 10;
    
    /* Initialize system */
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    
    printf("CH32V006 I2C Slave Example\r\n");
    printf("System Clock: %d Hz\r\n", SystemCoreClock);
    printf("Slave Address: 0x%02X\r\n", I2C_SLAVE_ADDRESS);
    
    I2C_Slave_Config();
    printf("I2C Slave Initialized\r\n");
    printf("Using pins PC1 (SCL) and PC2 (SDA)\r\n\r\n");
    
    i2c_registers[0x00] = 0x00;  // Command register
    i2c_registers[0x01] = 0xAA;  // Status register
    i2c_registers[0x02] = 0x10;  // Version high
    i2c_registers[0x03] = 0x01;  // Version low
    

    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin   = GPIO_Pin_3;
    gpio.GPIO_Mode  = GPIO_Mode_AF_OD;   // push-pull output
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &gpio);

    GPIO_SetBits(GPIOD, GPIO_Pin_3);      // OFF (sink config: high = off)
    
    while(1)
    {

        blink_counter++;
        if(blink_counter >= blink_interval)
        {
            blink_counter = 0;
            led_state = !led_state;
            
            /* In sink mode: LOW = LED ON, HIGH = LED OFF */
            if(led_state)
            {
                GPIO_ResetBits(GPIOD, GPIO_Pin_3);  // Pull low - LED ON
            }
            else
            {
                GPIO_SetBits(GPIOD, GPIO_Pin_3);    // Release high - LED OFF
            }
        }
        
        if(i2c_rx_complete)
        {
            i2c_rx_complete = 0;
            printf("Data received via I2C\r\n");
            
            if(i2c_registers[0x00] != 0x00)
            {
                I2C_ProcessCommand();
            }
            
            printf("Registers: ");
            for(i = 0; i < 16; i++)
            {
                printf("%02X ", i2c_registers[i]);
            }
            printf("\r\n");
            
            led_state = !led_state;
            GPIO_WriteBit(GPIOD, GPIO_Pin_0, led_state);
        }
        
        if(i2c_tx_complete)
        {
            i2c_tx_complete = 0;
            printf("Data transmitted via I2C\r\n");
        }
        
        Delay_Ms(10);
    }
}

