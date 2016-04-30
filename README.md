# MPU9250forRfduinoWithDMP
this is the ported library from official DMP library for using on Rfduino or Arduino
this is based on offical DMP library 6.12. 

 NB: if you want to use this on Arduino, please change the callback function in inv_mpu.c 
 change the callback function called Rfduino_pinWakeCallback to AttachInterrupt with the required format in arduino.
