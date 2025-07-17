/********************************** (C) COPYRIGHT *******************************
* File Name          : net_config.h
* Author             : WCH
* Version            : V1.30
* Date               : 2024/07/17
* Description        : Network configuration for CH32V307 Master Cluster Controller
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __NET_CONFIG_H__
#define __NET_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * Socket configuration for cluster controller
 * IPRAW + UDP + TCP + TCP_LISTEN = number of sockets
 */
#define WCHNET_NUM_IPRAW              0  /* Number of IPRAW connections */

#define WCHNET_NUM_UDP                2  /* UDP connections (broadcast + discovery) */

#define WCHNET_NUM_TCP                4  /* TCP connections (status, control, update + client) */

#define WCHNET_NUM_TCP_LISTEN         3  /* TCP listening sockets (status, control, update servers) */

/* The number of sockets, maximum is 31 */
#define WCHNET_MAX_SOCKET_NUM         (WCHNET_NUM_IPRAW+WCHNET_NUM_UDP+WCHNET_NUM_TCP+WCHNET_NUM_TCP_LISTEN)

#define WCHNET_TCP_MSS                1460  /* Size of TCP MSS */

#define WCHNET_NUM_POOL_BUF           (WCHNET_NUM_TCP*2+4)   /* Number of POOL BUFs, receive queues */

/*********************************************************************
 * MAC queue configuration
 */
#define ETH_TXBUFNB                   2    /* Number of MAC transmit descriptors */

#define ETH_RXBUFNB                   8    /* Number of MAC receive descriptors */

#ifndef ETH_MAX_PACKET_SIZE
#define ETH_RX_BUF_SZE                1520  /* MAC receive buffer length, 4-byte aligned */
#define ETH_TX_BUF_SZE                1520  /* MAC transmit buffer length, 4-byte aligned */
#else
#define ETH_RX_BUF_SZE                ETH_MAX_PACKET_SIZE
#define ETH_TX_BUF_SZE                ETH_MAX_PACKET_SIZE
#endif

/*********************************************************************
 * Functional configuration
 */
#define WCHNET_PING_ENABLE            1     /* PING enabled */

#define TCP_RETRY_COUNT               20    /* TCP retransmission count */

#define TCP_RETRY_PERIOD              10    /* TCP retransmission period (50ms units) */

#define SOCKET_SEND_RETRY             1     /* Send retry on failure */

#define HARDWARE_CHECKSUM_CONFIG      1     /* Hardware checksum enable */

#define FINE_DHCP_PERIOD              8     /* Fine DHCP period (250ms units) */

#define CFG0_TCP_SEND_COPY            1     /* TCP send buffer copy */

#define CFG0_TCP_RECV_COPY            1     /* TCP receive copy optimization */

#define CFG0_TCP_OLD_DELETE           0     /* Delete oldest TCP connection */

#define CFG0_IP_REASS_PBUFS           0     /* Number of IP reassembly PBUFs */

#define CFG0_TCP_DEALY_ACK_DISABLE    0     /* TCP delay ACK disable */

/*********************************************************************
 * Memory configuration - optimized for cluster controller
 */
/* Increased buffer size for cluster data */
#define RECE_BUF_LEN                  (WCHNET_TCP_MSS*2)   /* Socket receive buffer size */

#define WCHNET_NUM_PBUF               (WCHNET_NUM_POOL_BUF*2)   /* Number of PBUF structures */

#define WCHNET_NUM_TCP_SEG            (WCHNET_NUM_TCP*4)   /* Number of TCP segments for sending */

/* Increased heap size for cluster management */
#define WCHNET_MEM_HEAP_SIZE          (((WCHNET_TCP_MSS+0x10+54+8)*WCHNET_NUM_TCP_SEG)+ETH_TX_BUF_SZE+1024+2*0x18)

#define WCHNET_NUM_ARP_TABLE          20   /* Number of ARP table entries */

#define WCHNET_MEM_ALIGNMENT          4    /* 4-byte alignment */

#if CFG0_IP_REASS_PBUFS
#define WCHNET_NUM_IP_REASSDATA       2    /* Number of IP reassembly structures */
#define WCHNET_SIZE_POOL_BUF    (((1500 + 14 + 4) + 3) & ~3)    /* Buffer size for single packet */
#else
#define WCHNET_NUM_IP_REASSDATA       0    /* Number of IP reassembly structures */
#define WCHNET_SIZE_POOL_BUF     (((WCHNET_TCP_MSS + 40 + 14 + 4) + 3) & ~3) /* Buffer size for single packet */
#endif

/* Validation checks */
#if(WCHNET_NUM_POOL_BUF * WCHNET_SIZE_POOL_BUF < ETH_RX_BUF_SZE)
    #error "WCHNET_NUM_POOL_BUF or WCHNET_TCP_MSS Error"
    #error "Please increase WCHNET_NUM_POOL_BUF or WCHNET_TCP_MSS for sufficient receive buffer"
#endif

#if( WCHNET_NUM_TCP_LISTEN && !WCHNET_NUM_TCP )
    #error "WCHNET_NUM_TCP Error, Please configure WCHNET_NUM_TCP >= 1"
#endif

#if((WCHNET_MEM_ALIGNMENT % 4) || (WCHNET_MEM_ALIGNMENT == 0))
    #error "WCHNET_MEM_ALIGNMENT Error, Please configure WCHNET_MEM_ALIGNMENT = 4 * N, N >=1"
#endif

#if((WCHNET_TCP_MSS > 1460) || (WCHNET_TCP_MSS < 60))
    #error "WCHNET_TCP_MSS Error, Please configure WCHNET_TCP_MSS >= 60 && WCHNET_TCP_MSS <= 1460"
#endif

#if((WCHNET_NUM_ARP_TABLE > 0X7F) || (WCHNET_NUM_ARP_TABLE < 1))
    #error "WCHNET_NUM_ARP_TABLE Error, Please configure WCHNET_NUM_ARP_TABLE >= 1 && WCHNET_NUM_ARP_TABLE <= 0X7F"
#endif

#if(WCHNET_NUM_POOL_BUF < 1)
    #error "WCHNET_NUM_POOL_BUF Error, Please configure WCHNET_NUM_POOL_BUF >= 1"
#endif

#if(WCHNET_NUM_PBUF < 1)
    #error "WCHNET_NUM_PBUF Error, Please configure WCHNET_NUM_PBUF >= 1"
#endif

#if(CFG0_IP_REASS_PBUFS && ((WCHNET_NUM_IP_REASSDATA > 10) || (WCHNET_NUM_IP_REASSDATA < 1)))
    #error "WCHNET_NUM_IP_REASSDATA Error, Please configure WCHNET_NUM_IP_REASSDATA < 10 && WCHNET_NUM_IP_REASSDATA >= 1"
#endif

#if(CFG0_IP_REASS_PBUFS > WCHNET_NUM_POOL_BUF)
    #error "WCHNET_NUM_POOL_BUF Error, Please configure CFG0_IP_REASS_PBUFS < WCHNET_NUM_POOL_BUF"
#endif

#if(WCHNETTIMERPERIOD > 50)
    #error "WCHNETTIMERPERIOD Error, Please configure WCHNETTIMERPERIOD < 50"
#endif

/* Configuration value 0 */
#define WCHNET_MISC_CONFIG0    (((CFG0_TCP_SEND_COPY) << 0) |\
                               ((CFG0_TCP_RECV_COPY)  << 1) |\
                               ((CFG0_TCP_OLD_DELETE) << 2) |\
                               ((CFG0_IP_REASS_PBUFS) << 3) |\
                               ((CFG0_TCP_DEALY_ACK_DISABLE) << 8))

/* Configuration value 1 */
#define WCHNET_MISC_CONFIG1    (((WCHNET_MAX_SOCKET_NUM)<<0)|\
                               ((WCHNET_PING_ENABLE) << 13) |\
                               ((TCP_RETRY_COUNT)    << 14) |\
                               ((TCP_RETRY_PERIOD)   << 19) |\
                               ((SOCKET_SEND_RETRY)  << 25) |\
                               ((HARDWARE_CHECKSUM_CONFIG) << 26)|\
                               ((FINE_DHCP_PERIOD) << 27))

#ifdef __cplusplus
}
#endif
#endif