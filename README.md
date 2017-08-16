# BMI160_AHRS

BMI160 firmware source codes.

## 2017-8-13
能够编译和运行，实现了：

* 蓝牙传输频率的修改
* acc和gyro量程的修改


## 2017-8-14
1. 写成_routing的函数都是需要定时调用的

## 8-15
1. 利用循环的数组下标的方式写了FileBuf_Data的缓冲区

## 8-16
1. 淘宝的卖家建议继续扩大缓冲区，扩充到4096 bytes试试