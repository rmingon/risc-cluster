/*
 * CH32V006 Main Application (Updateable)
 * Memory Layout: 0x08001000 - 0x08008000 (28KB)
 * This is the application that gets updated via the bootloader
 */

#include "debug.h"

#define APPLICATION_VERSION    0x0102  // v1.02
#define I2C_SLAVE_ADDRESS      0x10
#define I2C_BUFFER_SIZE        128

#define APPLICATION_START      0x00002000  // Corrected from 0x08001000
#define BOOTLOADER_START       0x00000000  // Corrected from 0x08000000

// I2C State machine
typedef enum {
    I2C_STATE_IDLE = 0,
    I2C_STATE_ADDR_MATCH,
    I2C_STATE_RX_REG,
    I2C_STATE_RX_DATA,
    I2C_STATE_TX_DATA
} I2C_State_t;

// Global variables
volatile uint8_t i2c_registers[I2C_BUFFER_SIZE] = {0}; 
volatile uint8_t i2c_reg_addr = 0;
volatile I2C_State_t i2c_state = I2C_STATE_IDLE;
volatile uint8_t i2c_rx_complete = 0;
volatile uint8_t i2c_tx_complete = 0;

// Function prototypes
void Application_Init(void);
void I2C_Slave_Config(void);
void I2C_ProcessCommand(void);
void Application_Task(void);
void Jump_To_Bootloader(void);

/*********************************************************************
 * @fn      Application_Init
 *
 * @brief   Initialize main application
 *
 * @return  none
 */
void Application_Init(void)
{
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    
    printf("\r\n=== CH32V006 Main Application v%d.%02d ===\r\n", 
           APPLICATION_VERSION >> 8, APPLICATION_VERSION & 0xFF);
    printf("Application start: 0x%08X\r\n", APPLICATION_START);
    printf("System Clock: %d Hz\r\n", SystemCoreClock);
    
    i2c_registers[0x00] = 0x00;  // Command register
    i2c_registers[0x01] = 0xAA;  // Status register (different from bootloader)
    i2c_registers[0x02] = (APPLICATION_VERSION >> 8) & 0xFF;  // Version high
    i2c_registers[0x03] = APPLICATION_VERSION & 0xFF;         // Version low
    i2c_registers[0x04] = 0x01;  // Config register (app mode)
    
    I2C_Slave_Config();
    printf("I2C Slave initialized at address 0x%02X\r\n", I2C_SLAVE_ADDRESS);
    
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOD, ENABLE);
    
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin   = GPIO_Pin_3;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOD, &gpio);
    
    GPIO_SetBits(GPIOD, GPIO_Pin_3); 
    
    printf("Application initialization complete\r\n\r\n");
}

void I2C_Slave_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef  I2C_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_I2C1, ENABLE);
    
    // Configure I2C pins (PC1 = SCL, PC2 = SDA)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS << 1;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;
    
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
                    i2c_reg_addr++;
                }
                else
                {
                    I2C_ReceiveData(I2C1); // Discard data
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
                i2c_reg_addr++;
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
    }
}

void I2C1_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_ER_IRQHandler(void)
{
    if(I2C_GetITStatus(I2C1, I2C_IT_AF))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
        i2c_state = I2C_STATE_IDLE;
    }
    
    if(I2C_GetITStatus(I2C1, I2C_IT_BERR))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_BERR);
        i2c_state = I2C_STATE_IDLE;
    }
    
    if(I2C_GetITStatus(I2C1, I2C_IT_OVR))
    {
        I2C_ClearITPendingBit(I2C1, I2C_IT_OVR);
        i2c_state = I2C_STATE_IDLE;
    }
}

void Jump_To_Bootloader(void)
{
    printf("Jumping to bootloader...\r\n");
    
    __disable_irq();
    I2C_Cmd(I2C1, DISABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_I2C1, DISABLE);
    
    uint32_t* bootloader_vector = (uint32_t*)BOOTLOADER_START;
    uint32_t bootloader_stack = bootloader_vector[0];
    uint32_t bootloader_entry = bootloader_vector[1];
    
    __asm volatile(
        "mv sp, %0\n"      // Move bootloader_stack to stack pointer
        "jr %1\n"          // Jump to bootloader_entry address
        :
        : "r" (bootloader_stack), "r" (bootloader_entry)
        : "memory"
    );
    
    // Should never reach here
    while(1);
}

void I2C_ProcessCommand(void)
{
    uint8_t cmd = i2c_registers[0x00];
    
    switch(cmd)
    {
        case 0x01:  // Reset
            printf("Reset command received\r\n");
            Delay_Ms(100);
            NVIC_SystemReset();
            break;
            
        case 0x02:  // Get Status
            i2c_registers[0x01] = 0xAA;  // Application status
            break;
            
        case 0x03:  // Enter Bootloader
            printf("Enter bootloader command received\r\n");
            Delay_Ms(100);
            Jump_To_Bootloader();
            break;
            
        case 0x30:  // Start Update (redirect to bootloader)
            printf("Firmware update requested - jumping to bootloader\r\n");
            Delay_Ms(100);
            Jump_To_Bootloader();
            break;
            
        case 0x40:  // Get Device Info
            i2c_registers[0x10] = 0x32;  // '2' - CH32V006
            i2c_registers[0x11] = 0x01;  // Application variant
            i2c_registers[0x12] = 32;    // Flash size in KB
            i2c_registers[0x13] = 6;     // RAM size in KB
            i2c_registers[0x14] = (APPLICATION_VERSION >> 8) & 0xFF;
            i2c_registers[0x15] = APPLICATION_VERSION & 0xFF;
            printf("Device info requested\r\n");
            break;
            
        case 0x50:  // Application-specific command: Set LED
            if(i2c_registers[0x10] & 0x01)
            {
                GPIO_ResetBits(GPIOD, GPIO_Pin_3);  // LED ON
                printf("LED ON\r\n");
            }
            else
            {
                GPIO_SetBits(GPIOD, GPIO_Pin_3);   // LED OFF
                printf("LED OFF\r\n");
            }
            break;
            
        case 0x51:  // Application-specific command: Get GPIO state
            i2c_registers[0x10] = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3) ? 0x00 : 0x01;
            printf("GPIO state requested\r\n");
            break;
            
        case 0x52:  // Application-specific command: Blink LED
            {
                uint8_t blink_count = i2c_registers[0x10];
                if(blink_count == 0) blink_count = 3;
                
                printf("Blinking LED %d times\r\n", blink_count);
                for(uint8_t i = 0; i < blink_count; i++)
                {
                    GPIO_ResetBits(GPIOD, GPIO_Pin_3);  // ON
                    Delay_Ms(200);
                    GPIO_SetBits(GPIOD, GPIO_Pin_3);    // OFF
                    Delay_Ms(200);
                }
            }
            break;
            
        default:
            printf("Unknown command: 0x%02X\r\n", cmd);
            i2c_registers[0x0B] = 0x01;  // Error: invalid command
            break;
    }
    
    // Clear command register
    i2c_registers[0x00] = 0x00;
}

void Application_Task(void)
{
    static uint32_t heartbeat_counter = 0;
    static uint8_t led_state = 0;
    
    heartbeat_counter++;
    if(heartbeat_counter >= 100)  // 1 second at 10ms loop
    {
        heartbeat_counter = 0;
        led_state = !led_state;
        
        if(led_state)
        {
            GPIO_ResetBits(GPIOD, GPIO_Pin_3);  // LED ON
            Delay_Ms(50);
            GPIO_SetBits(GPIOD, GPIO_Pin_3);    // LED OFF
        }
        
        static uint8_t heartbeat_value = 0;
        i2c_registers[0x05] = heartbeat_value++;
    }
}

int main(void)
{
    uint8_t i;
    
    Application_Init();
    
    printf("Application running - ready for I2C commands\r\n");
    printf("Commands:\r\n");
    printf("  0x01 - Reset\r\n");
    printf("  0x02 - Get Status\r\n");
    printf("  0x03 - Enter Bootloader\r\n");
    printf("  0x30 - Start Firmware Update\r\n");
    printf("  0x40 - Get Device Info\r\n");
    printf("  0x50 - Control LED (data in reg 0x10)\r\n");
    printf("  0x51 - Get GPIO State\r\n");
    printf("  0x52 - Blink LED (count in reg 0x10)\r\n\r\n");
    
    while(1)
    {
        if(i2c_rx_complete)
        {
            i2c_rx_complete = 0;
            printf("I2C data received\r\n");
            
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
        }
        
        if(i2c_tx_complete)
        {
            i2c_tx_complete = 0;
            printf("I2C data transmitted\r\n");
        }
        
        Application_Task();
        
        Delay_Ms(10);
    }
}