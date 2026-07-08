# solar_imu

2026.6.22

整板测试：1.stm32控制ML307C模块开关机通过。

2.LSM6DS3TR模块通过加速度计和重力计互补滤波融合出角度信息。

3.IMU唤醒功能测试中

2026.6.23

整版测试：1.IMU唤醒 6D和wake-up共同唤醒stop1低功耗模式通过

2.ADC采集锂电池电压完成。

2026.7.8

第二版PCB到达，修改micro sim卡为nano sim卡，将USB 5V引入太阳能充电芯片。

完成ML307C模块通过MQTT协议发送数据到腾讯云的EMQX服务器，最终在PC端的MQTTX上接收到对应的数据。