#ifndef __SERVICE_HANDLER_H__
#define __SERVICE_HANDLER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "service_protocol.h"

/* 共享宏 — 主循环和模块均需要 */
#define SERIAL_IDLE_TIMEOUT_MS 60000U

/* 全局变量声明 */
extern uint8_t  g_serial_session_active;
extern uint32_t g_last_uart2_activity;
extern uint8_t  g_service_busy;
extern uint8_t  g_service_task_running;

/* 函数原型 */
void Handle_Service_Frame(const ServiceFrame_t *frame);
void Service_UART2_Recover(void);
void Service_UART1_Recover(void);
void Service_Task(uint8_t busy_only);
void ML307C_Background_Poll(void);
void Delay_With_Service(uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* __SERVICE_HANDLER_H__ */
