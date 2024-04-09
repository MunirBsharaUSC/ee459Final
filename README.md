# ee459Final

DONE Temp Sensor, interface using "void therm_read_temperature(char *);" char * is the buffer used to print, make sure to init.

DONE HeartRate Monitor, interface using "void heartbeatCalc(char *);" char * is the buffer used to print

GPS interfacing doesn't use interrupts but using USART_Transmit, USART_Recieve functions, make sure to init.

DONE LCD, lcd_init, lcd_send_command, lcd_send_data. Interfacing is done using i2c, using ADDR2.

ACCELEROMETER interfacing is done using i2c, using ADDR.
