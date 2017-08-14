
#define acc_buffersize   10

float last_acc_g[acc_buffersize] ;
unsigned char temp_index = 0;
float acc_new_mid(float new_value){
	 float sum = 0; 
	 unsigned char i;
  	last_acc_g[temp_index] = new_value;
  	temp_index = (temp_index + 1) % acc_buffersize;
	for(i = 0;i<acc_buffersize;i++)
		sum += last_acc_g[i];
	return sum/acc_buffersize;
}

//取加速度变化值。
float get_mid(float in){
	if(in < 1.0f)return 1.0f - in;
	return in - 1.0f;
}

float Get_ACC_Blivce(float accg){
   float acc_wight;
   acc_wight = acc_new_mid(get_mid(accg));
   if(acc_wight > 0.1f){
   					acc_wight = 0.001f;
					}
	   else {
	   		acc_wight = (0.1f-acc_wight)*10.0f;
			}
   return acc_wight;
}
