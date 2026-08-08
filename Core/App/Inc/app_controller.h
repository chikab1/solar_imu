#ifndef APP_CONTROLLER_H
#define APP_CONTROLLER_H

/**
 * @file app_controller.h
 * @brief 应用层总控入口。
 *
 * main() 完成 CubeMX 外设初始化后调用 App_Init() 一次，并在无限循环中持续调用
 * App_Run()。中断回调仍由本模块实现，但回调内只记录标志或转交协议解析。
 */

/**
 * @brief 初始化设备业务状态、IMU、Flash 事件队列和维护串口协议。
 * @note 必须在所有 MX_* 外设初始化完成后调用，且仅调用一次。
 */
void App_Init(void);

/**
 * @brief 执行一轮应用调度。
 *
 * 处理维护串口、待上报事件、维护会话超时，以及满足条件时进入 Stop1。
 * 本函数不阻塞在永久循环中，供 main() 的 while(1) 重复调用。
 */
void App_Run(void);

#endif
