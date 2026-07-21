#ifndef __UART_DRIVER_H__
#define __UART_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <string.h>

#define UART1_RX_BUF_SIZE   512
#define UART1_TX_BUF_SIZE   256
#define UART2_RX_BUF_SIZE   256
#define UART2_TX_BUF_SIZE   128

typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_tx;

    uint8_t  *rx_buf;
    uint16_t  rx_buf_size;
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;
    volatile uint16_t rx_len;

    uint8_t  *tx_buf;
    uint16_t  tx_buf_size;
    volatile uint8_t  tx_busy;

    uint8_t  *dma_rx_buf;
    uint16_t  dma_rx_buf_size;
    volatile uint16_t last_dma_pos;
} UART_Driver_t;

extern UART_Driver_t g_uart1_drv;
extern UART_Driver_t g_uart2_drv;

void UART_Driver_Init(void);
void UART1_DMA_Rx_Idle_Callback(void);
void UART2_RxEvent_Callback(uint16_t size);
void UART1_TxDMA_Complete_Callback(void);
void UART2_TxDMA_Complete_Callback(void);
uint8_t UART1_RestartReceive(void);
void UART1_StopReceive(void);

uint16_t UART_Read(UART_Driver_t *drv, uint8_t *dst, uint16_t max_len);
uint16_t UART_ReadLine(UART_Driver_t *drv, uint8_t *dst, uint16_t max_len, uint32_t timeout_ms);
void UART_FlushRx(UART_Driver_t *drv);
uint8_t UART_Write(UART_Driver_t *drv, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
uint8_t UART_WriteStr(UART_Driver_t *drv, const char *str, uint32_t timeout_ms);
uint16_t UART_Available(UART_Driver_t *drv);

#ifdef __cplusplus
}
#endif

#endif
