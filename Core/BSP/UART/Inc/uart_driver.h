#ifndef __UART_DRIVER_H__
#define __UART_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <string.h>

/** @name 软件环形缓冲区容量（字节） */
/** @{ */
#define UART1_RX_BUF_SIZE   512 /**< ML307C AT/URC接收队列。 */
#define UART1_TX_BUF_SIZE   256 /**< ML307C DMA发送暂存区。 */
#define UART2_RX_BUF_SIZE   256 /**< 维护串口接收队列。 */
#define UART2_TX_BUF_SIZE   128 /**< 维护串口中断发送暂存区。 */
/** @} */

/**
 * @brief 一个UART实例的软件收发上下文。
 * @note USART1使用循环DMA接收和DMA发送；USART2使用ReceiveToIdle中断接收。
 *       rx_head/rx_tail/rx_len会同时被中断和主循环访问，因此声明为volatile。
 */
typedef struct {
    UART_HandleTypeDef *huart;   /**< STM32 HAL UART句柄。 */
    DMA_HandleTypeDef  *hdma_tx; /**< 发送DMA句柄；NULL表示使用中断发送。 */

    uint8_t  *rx_buf;            /**< 软件接收环形缓冲区。 */
    uint16_t  rx_buf_size;       /**< rx_buf容量。 */
    volatile uint16_t rx_head;   /**< 中断写入位置。 */
    volatile uint16_t rx_tail;   /**< 主循环读取位置。 */
    volatile uint16_t rx_len;    /**< 当前可读字节数。 */

    uint8_t  *tx_buf;            /**< 异步发送期间必须保持有效的暂存区。 */
    uint16_t  tx_buf_size;       /**< tx_buf容量。 */
    volatile uint8_t tx_busy;    /**< 1表示DMA/IT发送尚未完成。 */

    uint8_t  *dma_rx_buf;        /**< HAL/DMA直接写入的硬件接收区。 */
    uint16_t  dma_rx_buf_size;   /**< dma_rx_buf容量。 */
    volatile uint16_t last_dma_pos; /**< 循环DMA上次搬运到软件环形区的位置。 */
} UART_Driver_t;

/** @brief ML307C所用USART1驱动实例。 */
extern UART_Driver_t g_uart1_drv;
/** @brief CH340维护口所用USART2驱动实例。 */
extern UART_Driver_t g_uart2_drv;

/**
 * @brief 绑定HAL句柄与静态缓冲区，并启动USART1 DMA和USART2空闲中断接收。
 * @note 所有MX_USARTx_UART_Init()和MX_DMA_Init()完成后，在main()中调用一次。
 */
void UART_Driver_Init(void);

/** @brief USART1 IDLE中断入口，把循环DMA新增字节搬入软件环形缓冲区。 */
void UART1_DMA_Rx_Idle_Callback(void);

/** @brief USART2 ReceiveToIdle回调入口。@param size 本次收到的字节数。 */
void UART2_RxEvent_Callback(uint16_t size);

/** @brief USART1 DMA发送完成回调，释放tx_busy。 */
void UART1_TxDMA_Complete_Callback(void);
/** @brief USART2中断发送完成回调，释放tx_busy。 */
void UART2_TxDMA_Complete_Callback(void);

/** @brief 清错误并重新启动USART1循环DMA接收。@return 1成功，0失败。 */
uint8_t UART1_RestartReceive(void);
/** @brief Stop1前停止USART1 DMA接收并清空软件接收队列。 */
void UART1_StopReceive(void);

/**
 * @brief 非阻塞读取软件环形缓冲区。
 * @param drv UART驱动实例。
 * @param dst 输出缓冲区。
 * @param max_len 最多读取字节数。
 * @return 实际读取字节数。
 */
uint16_t UART_Read(UART_Driver_t *drv, uint8_t *dst, uint16_t max_len);

/**
 * @brief 读取到换行符、缓冲区满或超时。
 * @param drv UART驱动实例。
 * @param dst 输出字符串缓冲区。
 * @param max_len dst总容量，函数保留一个字节写`\0`。
 * @param timeout_ms 等待超时，单位ms。
 * @return 不含结尾`\0`的实际字节数。
 */
uint16_t UART_ReadLine(UART_Driver_t *drv, uint8_t *dst, uint16_t max_len, uint32_t timeout_ms);

/** @brief 原子清空指定UART的软件接收环形缓冲区。 */
void UART_FlushRx(UART_Driver_t *drv);

/**
 * @brief 通过DMA或中断异步发送二进制数据。
 * @param drv UART驱动实例。
 * @param data 待发送数据。
 * @param len 字节数，不得超过tx_buf_size。
 * @param timeout_ms 等待上一笔发送完成的最长时间。
 * @return 1已成功启动发送，0参数无效、忙超时或HAL启动失败。
 */
uint8_t UART_Write(UART_Driver_t *drv, const uint8_t *data, uint16_t len, uint32_t timeout_ms);

/** @brief UART_Write()的字符串封装。@return 1启动成功，0失败。 */
uint8_t UART_WriteStr(UART_Driver_t *drv, const char *str, uint32_t timeout_ms);

/** @brief 返回软件接收环形缓冲区当前可读字节数。 */
uint16_t UART_Available(UART_Driver_t *drv);

#ifdef __cplusplus
}
#endif

#endif
