# modebus从机功能
实现基于stm32F4的FreeModbus的功能
主要实现的功能有
```
0x01：读取线圈状态（单个或多个）。
0x02：读取输入状态（单个或多个）。
0x03：读取保持寄存器（单个或多个）。
0x04：读取输入寄存器（单个或多个）。
0x05：强制单个线圈（控制单个设备开关）。
0x06：强制单个寄存器（写入寄存器值）。
0x0F：强制多个线圈（批量控制多个设备）。
0x10：强制多个寄存器（批量写寄存器值）。
0x11：报文帧诊断。
0x13：读取设备识别。
```