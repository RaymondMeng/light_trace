# light_trace
* 基于stm32和openmv的色块追踪云台
* STM32F103C8T6作为本系统的主控芯片，在实时接收openmv传来的数据的同时控制舵机云台进行目标追踪
## 系统主控软件控制流程图
![](https://ae01.alicdn.com/kf/Uc6038aa90558428e9a1c4869d1a68b3dS.jpg)
## 设计方案
* 舵机脉冲波角度化：舵机的脉冲控制周期为0.5ms~2.5ms，控制频率在50Hz~330Hz之间。本系统采用的舵机为180°数字舵机，并以50Hz频率控制。即0.5ms为0°，2.5ms为180°，X角度需要的脉冲时间为0.5ms + x/180°* 2ms。
* 追踪算法：stm32在接收到openmv的串口数据后，即得到了坐标反馈，并以此得到了angle_error，这里采用了PID算法，分别精准控制roll_angle和pitch_angle，使得激光落点位置误差达到最小。
* Openmv程序设计：采用最传统的色块追踪，通过调用相关库函数来实现色块的追踪的功能。并通过调用pyb写入串口发送函数，将blob.cx和blob.cy实时发送至Stm32。
* 串口通信协议：串口通信协议采用的数据帧格式，即帧头、数据、校验位、帧尾。这样可以保证数据的准确性和效率性。由于数据可能大于8位，所以我采用了数据高八位和低八位分别传输，保证了数据的准确性。校验位采用数据位之和并对255进行取余。具体的程序设计中，openmv是通过右移实现发送，STM32通过左移实现接收，并且在STM32的串口程序设计中，采用了逐字节判断的方法来接收数据包，当检测到最后一个字节0xfe并且校验成功时，把坐标值传至全局变量中。
