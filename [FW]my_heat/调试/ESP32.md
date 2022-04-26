


ESP32 的运行状态主要由 GPIO0 决定

模式	GPIO0
UART 下载模式	低
Flash 运行模式	高

### 二、接线

模组	USB-TTL
3V3	Vo/3.3V/VCC
GND	GND
TXD	RXD
RXD	TXD
IO0	DTR
EN	RST/RTS



如何确保　ESP32　模组正确的进入了下载模式：
　　1. 首先确保模组可以正常运行工作（即确保电源和串口连接正常）；

　　2. 拉低 GPIO0，打开串口工具，在波特率​115200​下观察模组的启动或复位后的打印信息；

　　3. 若出现以下红色字符则认为模组已经进入了下载模式，可以去正常进行下载。

　　　　rst:​0x10 (RTCWDT_RTC_RESET),​boot:​0x3 (DOWNLOAD_BOOT(UART0/​UART1/​SDIO_REI_REO_V2))

　　　　​waiting for download

ESP32 Flash 地址配置
ESP32 在编译时，通过 make menuconfig 来配置 Partition Table 分别支持 Single factory app, no OTA、Factory app, two OTA definitions、Custom partition table CSV

注意1：乐鑫在不同版本的 SDK 中有可能会改变这些烧录位置，以下说明仅为参考，建议以开发时的 Console 输出信息为准。

注意2：合并后的固件内已经包含了地址信息，只需烧写到 0x0 地址即可。


### EN ---> 高

反复打印
```
rst:0x3 (SW_RESET),boot:0x1b (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:2
load:0x3fff0018,len:4
load:0x3fff001c,len:1044
load:0x40078000,len:10124
load:0x40080400,len:5828
entry 0x400806a8
ets Jun  8 2016 00:22:57
```