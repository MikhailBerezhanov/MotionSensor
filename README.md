# MotionSensor 
### Prototype of Bluetooth motion sensor based on STM32

Current project contains experimental firmware for __STM32F103C8__ that collects data from MEMS sensors __BMI160__ and __BMM150__ and transfers it via Bluetooth connection.
 
* Sensors are connected by SPI2 interface. 

### Embedded console menu

With USB-UART adapter it is possible to connect to USART1 (baudrate 921600) via terminal such as _PuTTy_ and use __embedded console menu__ for: 

* Sensors configuration
* Bluetooth configuraion  

Supported command list can be seen by typing `HELP`.

![console menu](https://habrastorage.org/webt/1g/pk/lz/1gpklzlyu3fm5vmrnxflchaofnk.jpeg "Menu example")

### Software filters

Experimental firmware supports different types of output data:

* __Raw__ - direct data from sensors format is: "gx\tgy\tgz\tax\tay\taz\tmx\tmy\tmz\0\n\r".
* __Madgwick algorithm filtered__
* __Mahony algorithm filtered__

_Note:_ filtered data slows data stream.  

Sensors poll period is 400 Hz.  
See `notes` folder for more details about data stream speed.

### Connection
MotionSensor prototype uses STM32 Bluepill board. 

![connection](https://habrastorage.org/webt/-v/nt/rk/-vntrkquc8akky9s1vqxjal4ieg.jpeg "Connection")