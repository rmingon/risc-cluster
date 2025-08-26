/*
 * CH32V006 Bootloader Application
 * Dedicated bootloader for firmware updates via I2C
 * Memory Layout: 0x08000000 - 0x08001000 (4KB)
 */

#include "debug.h"

#define BOOTLOADER_START       0x08000000
#define BOOTLOADER_SIZE        0x1000      // 4KB
#define APPLICATION_START      0x08001000  // 4KB offset
#define APPLICATION_MAX_SIZE   0x7000      // 28KB max
#define FLASH_END_ADDR         0x08008000  // 32KB total

#define BOOTLOADER_VERSION     0x0100
#define I2C_SLAVE_ADDRESS      0x10
#define FLASH_PAGE_SIZE        64
#define BOOT_DELAY_MS          500         // Wait 500ms for update command

#define REG_COMMAND            0x00
#define REG_STATUS             0x01
#define REG_VERSION_HIGH       0x02
#define REG_VERSION_LOW        0x03
#define REG_PAGE_HIGH          0x05
#define REG_PAGE_LOW           0x06
#define REG_DATA_LENGTH        0x07
#define REG_CHECKSUM_HIGH      0x08
#define REG_CHECKSUM_LOW       0x09
#define REG_UPDATE_PROGRESS    0x0A
#define REG_ERROR_CODE         0x0B
#define REG_DATA_BUFFER        0x10

// Commands
#define CMD_RESET              0x01
#define CMD_GET_STATUS         0x02
#define CMD_ENTER_BOOTLOADER   0x03
#define CMD_START_UPDATE       0x30
#define CMD_WRITE_PAGE         0x10
#define CMD_FINISH_UPDATE      0x31
#define CMD_JUMP_TO_APP        0x50

#define STATUS_BOOTLOADER      0xBB  // Different from application (0xAA)
#define STATUS_READY           0xCC
#define STATUS_UPDATING        0xDD
#define STATUS_ERROR           0xEE

typedef enum {
    BOOT_STATE_INIT = 0,
    BOOT_STATE_WAITING,
    BOOT_STATE_UPDATING,
    BOOT_STATE_COMPLETE,
    BOOT_STATE_ERROR,
    BOOT_STATE_JUMP_APP
} BootState_t;

// Global variables
volatile uint8_t i2c_registers[128] = {0};
volatile uint8_t i2c_reg_addr = 0;
volatile uint8_t i2c_rx_complete = 0;
volatile BootState_t boot_state = BOOT_STATE_INIT;
volatile uint32_t boot_timeout = 0;
volatile uint8_t update_in_progress = 0;

// Function prototypes
void Bootloader_Init(void);
void I2C_Bootloader_Config(void);
void Bootloader_ProcessCommand(void);
uint8_t Bootloader_ValidateApplication(void);
void Bootloader_JumpToApplication(void);
uint8_t Bootloader_UpdateFirmware(void);
uint16_t Calculate_CRC16(uint8_t* data, uint16_t length);

// Flash functions (simplified for bootloader)
uint8_t Flash_ErasePage_Boot(uint32_t address);
uint8_t Flash_WritePage_Boot(uint32_t address, uint8_t* data, uint16_t length);

void Bootloader_Init(void)
{
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    
    printf("\r\n=== CH32V006 Bootloader v%d.%02d ===\r\n", 
           BOOTLOADER_VERSION >> 8, BOOTLOADER_VERSION & 0xFF);
    printf("Bootloader: 0x%08X - 0x%08X\r\n", BOOTLOADER_START, APPLICATION_START);
    printf("Application: 0x%08X - 0x%08X\r\n", APPLICATION_START, FLASH_END_ADDR);
    
    i2c_registers[REG_COMMAND] = 0x00;
    i2c_registers[REG_STATUS] = STATUS_BOOTLOADER;
    i2c_registers[REG_VERSION_HIGH] = (BOOTLOADER_VERSION >> 8) & 0xFF;
    i2c_registers[REG_VERSION_LOW] = BOOTLOADER_VERSION & 0xFF;
    i2c_registers[REG_UPDATE_PROGRESS] = 0;
    i2c_registers[REG_ERROR_CODE] = 0;
    
    I2C_Bootloader_Config();
    
    boot_state = BOOT_STATE_WAITING;
    printf("Bootloader ready. Waiting %dms for update command...\r\n", BOOT_DELAY_MS);
}

void I2C_Bootloader_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef I2C_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    
    RCC_PB2PeriphClockCmd(RCC_PB2Periph_GPIOC | RCC_PB2Periph_AFIO, ENABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_I2C1, ENABLE);
    
    // Configure I2C pins
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
    
    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

void I2C1_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_EV_IRQHandler(void)
{
    static uint8_t i2c_state = 0;  // 0=idle, 1=addr_match, 2=rx_data, 3=tx_data
    uint32_t event = I2C_GetLastEvent(I2C1);
    
    switch(event)
    {
        case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
            i2c_state = 1;
            break;
            
        case I2C_EVENT_SLAVE_BYTE_RECEIVED:
            if(i2c_state == 1) {
                i2c_reg_addr = I2C_ReceiveData(I2C1);
                i2c_state = 2;
            } else if(i2c_state == 2) {
                if(i2c_reg_addr < 128) {
                    i2c_registers[i2c_reg_addr] = I2C_ReceiveData(I2C1);
                    i2c_reg_addr++;
                } else {
                    I2C_ReceiveData(I2C1);
                }
            }
            break;
            
        case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
            i2c_state = 3;
            if(i2c_reg_addr < 128) {
                I2C_SendData(I2C1, i2c_registers[i2c_reg_addr]);
                i2c_reg_addr++;
            } else {
                I2C_SendData(I2C1, 0xFF);
            }
            break;
            
        case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
            if(i2c_reg_addr < 128) {
                I2C_SendData(I2C1, i2c_registers[i2c_reg_addr]);
                i2c_reg_addr++;
            } else {
                I2C_SendData(I2C1, 0xFF);
            }
            break;
            
        case I2C_EVENT_SLAVE_STOP_DETECTED:
            if(i2c_state == 2) {
                i2c_rx_complete = 1;
            }
            i2c_state = 0;
            I2C_ClearFlag(I2C1, I2C_FLAG_STOPF);
            break;
    }
}

uint8_t Flash_ErasePage_Boot(uint32_t address)
{
    volatile uint32_t timeout = 10000;
    
    // Unlock flash
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    
    // Wait for ready
    while(FLASH->STATR & 0x00000001 && timeout--);  // BSY
    if(timeout == 0) return 0;
    
    // Set page erase
    FLASH->CTLR |= 0x00000002;  // PER
    FLASH->ADDR = address;
    FLASH->CTLR |= 0x00000040;  // STRT
    
    // Wait completion
    timeout = 10000;
    while(FLASH->STATR & 0x00000001 && timeout--);  // BSY
    
    // Clear PER
    FLASH->CTLR &= ~0x00000002;
    
    // Lock flash
    FLASH->CTLR |= 0x00000080;  // LOCK
    
    return (timeout > 0) ? 1 : 0;
}

uint8_t Flash_WritePage_Boot(uint32_t address, uint8_t* data, uint16_t length)
{
    uint16_t i;
    volatile uint16_t *flash_ptr = (volatile uint16_t*)address;
    uint16_t *data_ptr = (uint16_t*)data;
    volatile uint32_t timeout;
    
    // Unlock flash
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    
    FLASH->CTLR |= 0x00000001;  // PG
    
    for(i = 0; i < (length + 1) / 2; i++)
    {
        *flash_ptr++ = *data_ptr++;
        
        timeout = 10000;
        while(FLASH->STATR & 0x00000001 && timeout--);  // BSY
        if(timeout == 0) {
            FLASH->CTLR &= ~0x00000001;  // Clear PG
            FLASH->CTLR |= 0x00000080;   // Lock
            return 0;
        }
    }
    
    // Clear PG and lock
    FLASH->CTLR &= ~0x00000001;  // Clear PG
    FLASH->CTLR |= 0x00000080;   // Lock
    
    return 1;
}

uint16_t Calculate_CRC16(uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    
    for(i = 0; i < length; i++)
    {
        crc ^= data[i];
        for(j = 0; j < 8; j++)
        {
            if(crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Bootloader_ProcessCommand(void)
{
    uint8_t cmd = i2c_registers[REG_COMMAND];
    
    switch(cmd)
    {
        case CMD_GET_STATUS:
            i2c_registers[REG_STATUS] = STATUS_BOOTLOADER;
            printf("Status request\r\n");
            break;
            
        case CMD_START_UPDATE:
            boot_state = BOOT_STATE_UPDATING;
            update_in_progress = 1;
            i2c_registers[REG_STATUS] = STATUS_UPDATING;
            i2c_registers[REG_UPDATE_PROGRESS] = 0;
            i2c_registers[REG_ERROR_CODE] = 0;
            printf("Firmware update started\r\n");
            break;
            
        case CMD_WRITE_PAGE:
            if(boot_state == BOOT_STATE_UPDATING)
            {
                if(Bootloader_UpdateFirmware())
                {
                    printf("Page written successfully\r\n");
                }
                else
                {
                    printf("Page write failed\r\n");
                    boot_state = BOOT_STATE_ERROR;
                    i2c_registers[REG_STATUS] = STATUS_ERROR;
                }
            }
            break;
            
        case CMD_FINISH_UPDATE:
            if(boot_state == BOOT_STATE_UPDATING)
            {
                boot_state = BOOT_STATE_COMPLETE;
                update_in_progress = 0;
                i2c_registers[REG_STATUS] = STATUS_READY;
                i2c_registers[REG_UPDATE_PROGRESS] = 100;
                printf("Firmware update completed\r\n");
                
                // Reset in 2 seconds to run new firmware
                Delay_Ms(2000);
                NVIC_SystemReset();
            }
            break;
            
        case CMD_JUMP_TO_APP:
            if(Bootloader_ValidateApplication())
            {
                printf("Jumping to application...\r\n");
                Delay_Ms(100);
                Bootloader_JumpToApplication();
            }
            else
            {
                printf("Invalid application - staying in bootloader\r\n");
                i2c_registers[REG_ERROR_CODE] = 0x10;  // Invalid app
            }
            break;
            
        case CMD_RESET:
            printf("Reset command\r\n");
            Delay_Ms(100);
            NVIC_SystemReset();
            break;
    }
    
    // Clear command
    i2c_registers[REG_COMMAND] = 0x00;
}

uint8_t Bootloader_UpdateFirmware(void)
{
    uint32_t page_addr;
    uint16_t expected_crc, calculated_crc;
    uint8_t data_len;

    page_addr = (i2c_registers[REG_PAGE_HIGH] << 8) | i2c_registers[REG_PAGE_LOW];
    page_addr *= FLASH_PAGE_SIZE;
    page_addr += APPLICATION_START;
    
    data_len = i2c_registers[REG_DATA_LENGTH];
    expected_crc = (i2c_registers[REG_CHECKSUM_HIGH] << 8) | i2c_registers[REG_CHECKSUM_LOW];
    
    if(page_addr < APPLICATION_START || page_addr >= FLASH_END_ADDR)
    {
        i2c_registers[REG_ERROR_CODE] = 0x04;  // Address out of range
        return 0;
    }
    
    if(data_len == 0 || data_len > FLASH_PAGE_SIZE)
    {
        i2c_registers[REG_ERROR_CODE] = 0x06;  // Invalid data length
        return 0;
    }

    calculated_crc = Calculate_CRC16((uint8_t*)&i2c_registers[REG_DATA_BUFFER], data_len);
    if(calculated_crc != expected_crc)
    {
        i2c_registers[REG_ERROR_CODE] = 0x03;  // Checksum mismatch
        return 0;
    }
    
    if(!Flash_ErasePage_Boot(page_addr))
    {
        i2c_registers[REG_ERROR_CODE] = 0x02;  // Flash error
        return 0;
    }
    
    if(!Flash_WritePage_Boot(page_addr, (uint8_t*)&i2c_registers[REG_DATA_BUFFER], data_len))
    {
        i2c_registers[REG_ERROR_CODE] = 0x02;  // Flash error
        return 0;
    }
    
    static uint32_t total_written = 0;
    total_written += data_len;
    i2c_registers[REG_UPDATE_PROGRESS] = (total_written * 100) / APPLICATION_MAX_SIZE;
    if(i2c_registers[REG_UPDATE_PROGRESS] > 100) i2c_registers[REG_UPDATE_PROGRESS] = 100;
    
    return 1;
}

uint8_t Bootloader_ValidateApplication(void)
{
    uint32_t* app_vector = (uint32_t*)APPLICATION_START;
    
    if(app_vector[0] < 0x20000000 || app_vector[0] > 0x20002000) {
        return 0;  // Invalid stack pointer
    }
    
    if(app_vector[1] < APPLICATION_START || app_vector[1] >= FLASH_END_ADDR) {
        return 0;  // Invalid reset vector
    }
    
    uint8_t* app_data = (uint8_t*)APPLICATION_START;
    uint8_t all_ff = 1;
    for(uint32_t i = 0; i < 256; i++) {
        if(app_data[i] != 0xFF) {
            all_ff = 0;
            break;
        }
    }
    
    if(all_ff) return 0;  // Blank flash
    
    return 1;  // Valid application
}

void Bootloader_JumpToApplication(void)
{
    uint32_t* app_vector = (uint32_t*)APPLICATION_START;
    uint32_t app_stack = app_vector[0];
    uint32_t app_entry = app_vector[1];
    
    // Disable interrupts
    __disable_irq();
    
    I2C_Cmd(I2C1, DISABLE);
    RCC_PB1PeriphClockCmd(RCC_PB1Periph_I2C1, DISABLE);
    
    // Set stack pointer and jump to application
    __asm volatile(
        "mv sp, %0\n"      // RISC-V move instruction
        "jr %1\n"          // RISC-V jump register instruction
        :
        : "r" (app_stack), "r" (app_entry)
        : "memory"
    );
    
    // Should never reach here
    while(1);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Bootloader main function
 *
 * @return  none
 */
int main(void)
{
    Bootloader_Init();
    
    // Main bootloader loop
    while(1)
    {
        // Process I2C commands
        if(i2c_rx_complete)
        {
            i2c_rx_complete = 0;
            
            if(i2c_registers[REG_COMMAND] != 0x00)
            {
                Bootloader_ProcessCommand();
            }
        }
        
        // Handle boot timeout (jump to app if no update command)
        if(boot_state == BOOT_STATE_WAITING && !update_in_progress)
        {
            boot_timeout++;
            if(boot_timeout >= BOOT_DELAY_MS)  // 500ms timeout
            {
                if(Bootloader_ValidateApplication())
                {
                    printf("Timeout - jumping to application\r\n");
                    Bootloader_JumpToApplication();
                }
                else
                {
                    printf("No valid application - staying in bootloader\r\n");
                    boot_state = BOOT_STATE_ERROR;
                    i2c_registers[REG_STATUS] = STATUS_ERROR;
                    i2c_registers[REG_ERROR_CODE] = 0x10;  // No valid app
                }
            }
        }
        
        Delay_Ms(1);
    }
}