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
	* 测试发现会周期性的发生重复写入一整个缓缓区的现象
	* 如果缓冲区设置在100个File_Data的长度，则只发生重复写入，基本不会发生部分覆盖的现象

## 8-16
1. 淘宝的卖家建议继续扩大缓冲区，扩充到4096 bytes试试。
	* 卖家通过测试，发现TF写入文件的操作时间太长了。大部分时间在1ms左右写完信息，但有时候会超过100ms，导致原来的程序中丢失数据

	* 卖家给出了**通过UART1，调试TF写入SD卡时间的代码**：
		    void File_Save_Routing(void)
		    {
		    	uint32_t temp1, temp2;
		    	if (Data_Ready)
		    	{
		    		temp1 = micros();
		    		if (mf_write(File_SecBuf, 512) == 0){};
		    		temp2 = micros();
		    		temp1 = temp2 - temp1;
		    		if (temp1 > 2000)
		    		{
		    			UART1_Put_StringNum("TF_Time :", temp1 / 100);
		    			UART1_Put_String(" \r\n");
		    		}
		    		//if(mf_write(File_Data_buf,sizeof(File_Data))==0);
		    		// else UART1_Put_String("f_write err\r\n");
		    		Data_Ready = 0;
		    	}
		    }

2. 在`USB_check()`中	添加`FileSave_stop()`，使BMI160连接PC时关闭TIM4中断时钟，使BMI160能够当作U盘使，直接读取SD卡
	* 如果不关的话，U盘会无法读取