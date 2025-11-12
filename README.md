# Robotics-System-Assessment-2

程序入口地址定义: 如果这个事件发生，请跳转到某个固定地址执行中断服务程序（ISR）
软件执行入口（CPU 跳转逻辑）”，属于**中断控制层**

| Adress | Interrupt Vector | Functionality                  |
| ------ | ---------------- | ------------------------------ |
| 003E   | TIMER3 CAPT      | Timer/Counter3 Capture Event   |
| 0040   | TIMER3 COMPA     | Timer/Counter3 Compare Match A |
| 0042   | TIMER3 COMPB     | Timer/Counter3 Compare Match B |
| 0044   | TIMER3 COMPC     | Timer/Counter3 Compare Match C |
| 0046   | TIMER3 OVF       | Timer/Counter3 Overflow        |

这一块出现在数据手册的 **I/O 寄存器地址映射表** 中。这部分是真正的“硬件存储单元”， 用来存放定时器的计数值、比较目标值、捕获值等。

| Adress | Register | Functionality                                        |
| ------ | -------- | ---------------------------------------------------- |
| 0x9D   | OCR3CH   | Timer/Counter3 - Output Compare Register C High Byte |
| 0x9C   | OCR3CL   | Timer/Counter3 - Output Compare Register C Low Byte  |
| 0x9B   | OCR3BH   | Timer/Counter3 - Output Compare Register B High Byte |
| 0x9A   | OCR3BL   | Timer/Counter3 - Output Compare Register B Low Byte  |
| 0x99   | OCR3AH   | Timer/Counter3 - Output Compare Register A High Byte |
| 0x98   | OCR3AL   | Timer/Counter3 - Output Compare Register A Low Byte  |
| 0x97   | ICR3H    | Input Capture Register High Byte                     |
| 0x96   | ICR3L    | Input Capture Register Low Byte                      |
| 0x95   | TCNT3H   | Counter Register High Byte                           |
| 0x94   | TCNT3L   | Counter Register Low Byte                            |

Timer3控制寄存器

TCCR3A

| Bit  | Name   | Functionality                          |
| ---- | ------ | -------------------------------------- |
| 7    | COM3A1 | 比较输出模式 A（控制 OC3A 引脚的行为） |
| 6    | COM3A0 | 同上, 两位组合决定模式                 |
| 5    | COM3B1 | 比较输出模式 B（控制 OC3B 引脚）       |
| 4    | COM3B0 | 同上, 两位组合决定模式                 |
| 3    | —      | 保留                                   |
| 2    | —      | 保留                                   |
| 1    | WGM31  | 波形生成模式位 1                       |
| 0    | WGM31  | 波形生成模式位 0                       |

TCCR3B

| Bit  | Name  | Functionality    |
| ---- | ----- | ---------------- |
| 7    | ICNC3 | 输入捕获滤波     |
| 6    | ICES3 | 输入捕获边沿选择 |
| 5    | —     |                  |
| 4    | WGM33 | 波形生成模式位 3 |
| 3    | WGM32 | 波形生成模式位 2 |
| 2    | CS32  | 分频控制位 2     |
| 1    | CS31  | 分频控制位 1     |
| 0    | CS30  | 分频控制位 0     |

分频配置表

| CS32 | CS31 | CS30 | 分频因子 | 含义                |
| ---- | ---- | ---- | -------- | ------------------- |
| 0    | 0    | 0    | 停止     | Timer 停止计数      |
| 0    | 0    | 1    | 1        | 不分频（16 MHz）    |
| 0    | 1    | 0    | 8        | ÷8（2 MHz）         |
| 0    | 1    | 1    | 64       | ÷64（250 kHz）      |
| 1    | 0    | 0    | 256      | ÷256（62.5 kHz）    |
| 1    | 0    | 1    | 1024     | ÷1024（15.625 kHz） |

ATmega32U4 的每个定时器都有一个“波形生成模式（WGM）”，
用来控制定时器在什么条件下清零、是否触发中断、是否生成PWM等。

波形生成模式配置表

| WGM33 | WGM33 | WGM33 | WGM33 | 含义                                     |
| ----- | ----- | ----- | ----- | ---------------------------------------- |
| 0     | 0     | 0     | 0     | Normal 模式（数到65535溢出               |
| 0     | 1     | 0     | 0     | CTC 模式（Clear Timer on Compare Match） |
| 1     | 1     | 1     | 0     | Fast PWM (ICR3为TOP)                     |
| 1     | 1     | 1     | 1     | Fast PWM (OCR3A为TOP)                    |

TIMSK3用来启用或关闭指定类型的中断寄存器

C 通道更常作为纯输出比较，不触发中断，没有控制位

| Bit  | Name   | Functionality       | Corresponding ISR Vector |
| ---- | ------ | ------------------- | ------------------------ |
| 5    | ICIE3  | 输入捕获中断使能    | TIMER3_CAPT_vect         |
| 2    | OCIE3B | 比较匹配 B 中断使能 | TIMER3_COMPB_vect        |
| 1    | OCIE3B | 比较匹配 A 中断使能 | TIMER3_COMPA_vect        |
| 0    | TOIE3  | 溢出中断使能        | TIMER3_OVF_vect          |