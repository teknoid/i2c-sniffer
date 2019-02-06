# Arduino I2C Sniffer

This is a software i2c sniffer for Arduino Nano and compatible (Atmega328) boards. Connection setup:

```c
SCL --> D2
SDA --> D3
```

The Samples will be stored in internal memory and then printed on serial console
Sample output (ES9028 Control Unit): 

```c
90+ 40+ 91+ A1-                                                                 
90+ 42+ 91+ A6-                                                                 
90+ 43+ 91+ E6-                                                                 
90+ 44+ 91+ 1C-                                                                 
90+ 45+ 91+ 00-                                                                 
90+ 64+ 91+ 02- 
```