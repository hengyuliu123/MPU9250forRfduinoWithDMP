# MPU9250forRfduinoWithDMP
<br />this is the ported library from official DMP library for using on Rfduino or Arduino
<br />
<br />this is based on offical DMP library 6.12. 
<br />
<br />NB: if you want to use this on Arduino, please change the callback function in inv_mpu.c 
<br />
<br />____change the callback function called Rfduino_pinWakeCallback to AttachInterrupt with the required format in arduino.
<br />
<br />____and also the interrupt pin in mpu.cpp to your own interrput pin.
<br />
<br />_______________based on gregd72002 and Nav6 project's way to port and write .h files for Rfduino
<br />
<br />                                                          Felix,2016
