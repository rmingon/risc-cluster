// ch32v307_master.h - Header file for CH32V307 Master Controller

#ifndef __CH32V307_MASTER_H
#define __CH32V307_MASTER_H

#include <stdint.h>

// Network configuration
#define USE_DHCP            1
#define STATIC_IP_ADDR      "192.168.1.100"
#define STATIC_NETMASK      "255.255.255.0"
#define STATIC_GATEWAY      "192.168.1.1"
#define TCP_SERVER_PORT     8080
#define UDP_SERVER_PORT     8081

// I2C Configuration
#define I2C_TIMEOUT_MS      100
#define I2C_RETRY_COUNT     3

// Programming protocol packet structure
typedef struct {
    uint8_t target_slave;   // Target slave ID (0-7)
    uint8_t command;        // Command byte
    uint16_t data_len;      // Data length
    uint8_t data[256];      // Data payload
} prog_packet_t;

// Response packet structure
typedef struct {
    uint8_t slave_id;       // Responding slave ID
    uint8_t status;         // Status code
    uint16_t data_len;      // Response data length
    uint8_t data[256];      // Response data
} resp_packet_t;

// Firmware update structure
typedef struct {
    uint32_t total_size;    // Total firmware size
    uint32_t crc32;         // CRC32 checksum
    uint16_t page_size;     // Page size for programming
    uint16_t num_pages;     // Number of pages
} fw_update_info_t;

// Extended commands for batch operations
#define CMD_BATCH_ERASE     0x10
#define CMD_BATCH_PROGRAM   0x11
#define CMD_BATCH_VERIFY    0x12
#define CMD_SCAN_SLAVES     0x13
#define CMD_SET_I2C_ADDR    0x14

// Error codes
#define ERR_NONE            0x00
#define ERR_CRC             0x10
#define ERR_TIMEOUT         0x11
#define ERR_NACK            0x12
#define ERR_BUSY            0x13
#define ERR_INVALID_CMD     0x14
#define ERR_INVALID_ADDR    0x15
#define ERR_WRITE_FAILED    0x16
#define ERR_VERIFY_FAILED   0x17

// Function prototypes for extended functionality
void Scan_I2C_Slaves(void);
uint8_t Batch_Program_Slaves(uint8_t *slave_list, uint8_t count, uint8_t *fw_data, uint32_t fw_size);
uint8_t Update_Slave_Firmware(uint8_t slave_id, const char *filename);
void Web_Server_Init(void);
void UDP_Broadcast_Status(void);

// Utility functions
uint32_t Calculate_CRC32(uint8_t *data, uint32_t len);
void Print_Slave_Status(void);
void Reset_All_Slaves(void);

#endif /* __CH32V307_MASTER_H */