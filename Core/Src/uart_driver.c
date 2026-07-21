#include "uart_driver.h"
#include "usart.h"
#include "dma.h"
#include <string.h>

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

static uint8_t s_uart1_rx_buf[UART1_RX_BUF_SIZE];
static uint8_t s_uart1_tx_buf[UART1_TX_BUF_SIZE];
static uint8_t s_uart1_dma_buf[256];

static uint8_t s_uart2_rx_buf[UART2_RX_BUF_SIZE];
static uint8_t s_uart2_tx_buf[UART2_TX_BUF_SIZE];
/* One maximum service frame is 75 bytes. Keep enough headroom so an entire
 * frame can be received without a full-buffer re-arm gap. */
static uint8_t s_uart2_it_buf[256];

UART_Driver_t g_uart1_drv;
UART_Driver_t g_uart2_drv;

void UART_Driver_Init(void)
{
    memset(&g_uart1_drv, 0, sizeof(g_uart1_drv));
    g_uart1_drv.huart            = &huart1;
    g_uart1_drv.hdma_tx         = &hdma_usart1_tx;
    g_uart1_drv.rx_buf          = s_uart1_rx_buf;
    g_uart1_drv.rx_buf_size     = UART1_RX_BUF_SIZE;
    g_uart1_drv.tx_buf          = s_uart1_tx_buf;
    g_uart1_drv.tx_buf_size     = UART1_TX_BUF_SIZE;
    g_uart1_drv.dma_rx_buf      = s_uart1_dma_buf;
    g_uart1_drv.dma_rx_buf_size = sizeof(s_uart1_dma_buf);

    memset(&g_uart2_drv, 0, sizeof(g_uart2_drv));
    g_uart2_drv.huart            = &huart2;
    g_uart2_drv.hdma_tx         = NULL;
    g_uart2_drv.rx_buf          = s_uart2_rx_buf;
    g_uart2_drv.rx_buf_size     = UART2_RX_BUF_SIZE;
    g_uart2_drv.tx_buf          = s_uart2_tx_buf;
    g_uart2_drv.tx_buf_size     = UART2_TX_BUF_SIZE;
    g_uart2_drv.dma_rx_buf      = s_uart2_it_buf;
    g_uart2_drv.dma_rx_buf_size = sizeof(s_uart2_it_buf);

    HAL_DMA_Abort(&hdma_usart1_rx);
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    HAL_DMA_Init(&hdma_usart1_rx);
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, s_uart1_dma_buf, sizeof(s_uart1_dma_buf));

    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_uart2_it_buf, sizeof(s_uart2_it_buf));
}

void UART1_StopReceive(void)
{
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    (void)HAL_UART_DMAStop(&huart1);
    UART_FlushRx(&g_uart1_drv);
    g_uart1_drv.last_dma_pos = 0U;
}

uint8_t UART1_RestartReceive(void)
{
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    (void)HAL_UART_DMAStop(&huart1);
    (void)HAL_DMA_Abort(&hdma_usart1_rx);
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_PEF | UART_CLEAR_FEF |
                                   UART_CLEAR_NEF | UART_CLEAR_OREF |
                                   UART_CLEAR_IDLEF);
    UART_FlushRx(&g_uart1_drv);
    g_uart1_drv.last_dma_pos = 0U;
    huart1.ErrorCode = HAL_UART_ERROR_NONE;
    if (HAL_UART_Receive_DMA(&huart1, g_uart1_drv.dma_rx_buf,
                            g_uart1_drv.dma_rx_buf_size) != HAL_OK) {
        return 0U;
    }
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    return 1U;
}

static void ring_push(UART_Driver_t *drv, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        if (drv->rx_len < drv->rx_buf_size) {
            drv->rx_buf[drv->rx_head] = data[i];
            drv->rx_head = (drv->rx_head + 1) % drv->rx_buf_size;
            drv->rx_len++;
        }
    }
}

void UART1_DMA_Rx_Idle_Callback(void)
{
    UART_Driver_t *drv = &g_uart1_drv;
    uint16_t total = drv->dma_rx_buf_size;
    uint16_t dma_pos = total - __HAL_DMA_GET_COUNTER(drv->huart->hdmarx);

    if (dma_pos == total) {
        if (drv->last_dma_pos < total) {
            ring_push(drv, drv->dma_rx_buf + drv->last_dma_pos, total - drv->last_dma_pos);
        }
        drv->last_dma_pos = 0;
        return;
    }

    if (dma_pos != drv->last_dma_pos) {
        if (dma_pos > drv->last_dma_pos) {
            ring_push(drv, drv->dma_rx_buf + drv->last_dma_pos, dma_pos - drv->last_dma_pos);
        } else {
            ring_push(drv, drv->dma_rx_buf + drv->last_dma_pos, total - drv->last_dma_pos);
            ring_push(drv, drv->dma_rx_buf, dma_pos);
        }
        drv->last_dma_pos = dma_pos;
    }
}

void UART2_RxEvent_Callback(uint16_t size)
{
    if (size > 0) {
        ring_push(&g_uart2_drv, g_uart2_drv.dma_rx_buf, size);
    }
}

void UART1_TxDMA_Complete_Callback(void)
{
    g_uart1_drv.tx_busy = 0;
}

void UART2_TxDMA_Complete_Callback(void)
{
    g_uart2_drv.tx_busy = 0;
}

uint16_t UART_Read(UART_Driver_t *drv, uint8_t *dst, uint16_t max_len)
{
    uint16_t count = 0;
    while (count < max_len && drv->rx_len > 0) {
        __disable_irq();
        if (drv->rx_len > 0) {
            dst[count++] = drv->rx_buf[drv->rx_tail];
            drv->rx_tail = (drv->rx_tail + 1) % drv->rx_buf_size;
            drv->rx_len--;
        }
        __enable_irq();
    }
    return count;
}

uint16_t UART_ReadLine(UART_Driver_t *drv, uint8_t *dst, uint16_t max_len, uint32_t timeout_ms)
{
    uint16_t count = 0;
    uint32_t start = HAL_GetTick();

    while (count < max_len - 1) {
        if (drv->rx_len > 0) {
            __disable_irq();
            uint8_t byte = drv->rx_buf[drv->rx_tail];
            drv->rx_tail = (drv->rx_tail + 1) % drv->rx_buf_size;
            drv->rx_len--;
            __enable_irq();

            dst[count++] = byte;
            if (byte == '\n') break;
        } else {
            if (HAL_GetTick() - start >= timeout_ms) break;
        }
    }
    dst[count] = '\0';
    return count;
}

void UART_FlushRx(UART_Driver_t *drv)
{
    __disable_irq();
    drv->rx_head = 0;
    drv->rx_tail = 0;
    drv->rx_len  = 0;
    __enable_irq();
}

uint8_t UART_Write(UART_Driver_t *drv, const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    if (len == 0 || len > drv->tx_buf_size) return 0;

    uint32_t start = HAL_GetTick();
    while (drv->tx_busy) {
        if (HAL_GetTick() - start >= timeout_ms) return 0;
    }

    memcpy(drv->tx_buf, data, len);
    drv->tx_busy = 1;

    if (drv->hdma_tx != NULL) {
        if (HAL_UART_Transmit_DMA(drv->huart, drv->tx_buf, len) != HAL_OK) {
            drv->tx_busy = 0;
            return 0;
        }
    } else {
        if (HAL_UART_Transmit_IT(drv->huart, drv->tx_buf, len) != HAL_OK) {
            drv->tx_busy = 0;
            return 0;
        }
    }
    return 1;
}

uint8_t UART_WriteStr(UART_Driver_t *drv, const char *str, uint32_t timeout_ms)
{
    return UART_Write(drv, (const uint8_t *)str, (uint16_t)strlen(str), timeout_ms);
}

uint16_t UART_Available(UART_Driver_t *drv)
{
    return drv->rx_len;
}
