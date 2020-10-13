---
title: hFramework (deprecated)
id: hframework
---

## About ##

hFramework is a library for creating software for mechatronic devices (e.g. robots). It's completely open sorce. You can find it in the Husarion's repository on [GitHub](https://github.com/husarion/hFramework).

It has the following ports:

    STM32 port for hardware created by Husarion - CORE2, CORE2mini and RoboCORE boards
    Linux port for Raspberry Pi and Tinkerboard (experimental)
    ESP32 port (experimental)

## API reference ##

hFramework API documentation is available [here](https://husarion.com/core2/api_reference/classes.html). API it is a set of clearly defined methods of communication between various software components our API makes it easier to develop a program by providing all the building blocks. Mainly use by advanced users. 

## Using hFramework ##

The easiest way to experience hFramework is to use Husarion WebIDE or install Husarion plugin to Visual Studio Code. Each of these methods gives us different options. With WebIDE we can start programing our device right after unpacking and the controller does not have to be even close to us, if you just starting follow this [tutorial](https://github.com/husarion/docs_new/blob/hframework-deprecated/docs/tutorials/howtostart/core2-quick-start.md). Programming via VSC allows us to rebuild the default implemented library but we have to connect board with computer via USB and and depending on the system we use, install several packages. full instruction of using offline development tools you can find [here](https://husarion.com/tutorials/deprecated/offline-development-tools/).

### Building hFramework ###

If you need to change code of hFramework you have to follow the instructions from [manual](https://husarion.com/tutorials/deprecated/hframework-library-development/)). 

## Examples ##

In this chapter you can find few examples of code for CORE2. You can check other examples on GitHub:

* [hFramework examples](https://github.com/husarion/hFramework/tree/master/examples). 
* [hSensors examples](https://github.com/husarion/hSensors/tree/master/examples). 
* [Modules examples](https://github.com/husarion/modules/tree/master/examples). 

### Basics ##

```cpp
#include <hFramework.h>

void hMain()
{
	for(;;)
	{
		Serial.printf("Hello World!"); // send string via USB Serial at 460800
		sys.delay(100); 
	}
}
```

## GPIO ##

### adc ###

```cpp
#include <hFramework.h>

void hMain()
{
	hSens4.pin1.enableADC(); // enable ADC on pin1 on hSens4 port 
	hExt.pin1.enableADC(); // enable ADC on pin1 on hExt port
	for(;;)
	{
		float val1 = hSens4.pin1.analogReadVoltage(); // read analog value (voltage in [V])
		int val2 = hExt.pin1.analogReadRaw(); // read analog value (in range from 0 to 4095)
		Serial.printf("Voltage hSens4.pin1: %f\tRaw data hExt.pin1: %d\r\n", val1, val2); // send string via USB Serial at 460800
		sys.delay(100); 
	}
}
```

### gpio ###

```cpp
#include <hFramework.h>

void hMain()
{
	hExt.pin1.setOut(); // setting hExt.pin1 as the output
	hSens1.pin1.setOut();  // setting hSens.pin1 as the output
	while (true)
	{
		hSens1.pin1.toggle(); // changing hSens.pin1 GPIO to opposite voltage level 
		hExt.pin1.toggle(); // changing hSens.pin1 GPIO to opposite voltage level 
		sys.delay(300);
	}
}
```

### gpio inout ###

```cpp
#include <hFramework.h>

void hMain()
{
	hExt.pin1.setIn_pd();  // setting pin1 on hExt as the input with pull down resistor
	hExt.pin2.setOut(); // setting pin2 on hExt as the output 
	while (true)
	{
		bool value = hExt.pin1.read(); // read from pin1 on hExt
		hExt.pin2.write(value);  // write value to pin2 on hExt
		sys.delay(300);
	}
}


void hMain2()
{
	hExt.pin1.setOutOD();  // setting pin1 on hExt as the output open drain mode 
	while (true)
	{
		hExt.pin1.toggle(); // changing pin1 on hExt state between high impedance and low state
		sys.delay(500);
	}
}
```

## LED ##

### LED ###

```cpp
#include <hFramework.h>

void hMain()
{
	// turn on LEDs
	hLED1.on();
	hLED2.on();
	hLED3.on();
	sys.delay(500);
	// turn off LEDs
	hLED1.off();
	hLED2.off();
	hLED3.off();

	while (true)
	{
		hLED1.toggle();  // switch state LED1
		sys.delay(100);
		hLED2.toggle();  // switch state LED2
		sys.delay(100); 
		hLED3.toggle();  // switch state LED3
		sys.delay(100);
	}
}
```

## Interfaces ##

### buttons simple ###

```cpp
#include <hFramework.h>

void hMain()
{
	while (true)
	{
		bool state = hBtn1.isPressed();  // creating a variable containig value of hBtn1.isPressed
		printf("%d\r\n", state); 

		if (state)  // checking if the button is pressed. If it is, LEDs will be turned on
		{ 
			hLED1.on();
			hLED3.on();
		}
		else  // if it isn't, LEDs will be turned off
		{
			hLED1.off();
			hLED3.off();
		}
		sys.delay(50);
	}
}
```
## Motors ##

### motor angle ###

```cpp
#include <hFramework.h>

void hMain()
{
    hMot3.setEncoderPolarity(Polarity::Reversed);  //changing encoder polarity (Polarity::Normal is default)
    hMot3.setMotorPolarity(Polarity::Reversed);  //changing motor polarity
    char c;
    while (1)
    {  
        if (Serial.available() > 0) { // checking Serial availability
            c = Serial.getch();
            if(c == 'a') {
                hMot3.rotRel(500, 200, false, INFINITE); //rotate by 500 ticks relatively to the current position, with 20% power
                hLED1.toggle();
            }
            if(c == 's') {
                hMot3.rotRel(-500, 200, true, INFINITE); //relative rotate 500 encoder ticks left with 20% of power with blocking task
                hLED1.toggle();
            }
            if(c == 'd') {
                hMot3.rotAbs(0, 200, true, INFINITE); //rotate to "0" ticks absolute position, and block program until task finishes
                hLED1.toggle();
            }
            sys.delay(200);
        }
    }
}
```

## SPI ##

### spi ###

```cpp
#include <hFramework.h>

void hMain()
{
	hExt.spi.setSpeed(SPISpeed::Speed42000); // configure hExt.spi with baudrate == 42000
	char cmd[] = {0xaa};					 // creating message
	hExt.spi.write(cmd, 1);					 // sending message thru SPI
	for (;;)
	{
		char c[10];					 // creating 10 element long char table
		hExt.spi.read(c, sizeof(c)); // geting 10 character from SPI
		sys.delay(200);
	}
}
```

## Serial I/O ##

### serial basic ###

```cpp
#include <hFramework.h>`

void hMain()
{
	
	sys.setSysLogDev(&devNull);								//turn off system logs on Serial
	sys.setLogDev(&Serial);									// setting USB-serial as a default printf output
	Serial.init(19200, Parity::None, StopBits::One);		// configure USB-serial with baudrate == 19200, none parity and with one stop bit

	for (;;)
	{	
		uint32_t time = sys.getRefTime();
		printf("Current lifetime: %d\r\n",time);
		sys.delay(500);
	}
}
```
## System ##

### sys_mutex ###

```cpp
#include <hFramework.h>

int counter = 0;
hMutex counter_mutex;

void add_1()
{
	// modify counter in mutex to ensure that the modification is not lost
	counter_mutex.take();
	int new_value = counter + 1;
	counter = new_value;
	counter_mutex.give();
}

void adder()
{
	while (true)
	{
		add_1();
		sys.delay(500);
	}
}

void hMain()
{
	sys.taskCreate(&adder);
	while (true)
	{
		add_1();
		printf("fizz\n");
		sys.delay(1000);
	}
}
```
### sys_queue ###

```cpp
#include <hFramework.h>

hQueue<unsigned int> queue(60); // create queue with capacity for 60 elements

void consumer()
{
	while (true)
	{
		unsigned int number; // pop element from of the queue. Will block if the queue is empty.
		queue.receive(number);
		printf("consumed %d\n", number);
	}
}

void hMain()
{
	sys.taskCreate(&consumer);
	unsigned int i = 0;
	while (true)
	{
		queue.sendToBack(i); // push element to the queue. Will block if the queue is full.
		i++;
	}
}
```

### sys_task ###

```cpp
#include <hFramework.h>
#include <cmath>

void buzzer() // creating instruction for buzzer
{
	while (true)
	{
		Serial.printf("buzz\n");
		sys.delay(500);
	}
}

void motors() //creating instruction for motors
{
	for (;;)
	{
		int time = sys.getRefTime();
		float pos = sinf(time / 3000.0f * 2 * M_PI);
		hMotA.rotAbs(pos * 90.0f, 300, true); // robAbs with true as 'block' parameter blocks execution until motor reaches desired position
	}
}

void tog() //creating instruction for tog
{
	for (;;)
	{
		hLED1.toggle();
		sys.delay(50);
	}
}

void hMain()
{
	sys.taskCreate(&buzzer); // running task buzzer
	sys.taskCreate(&motors); // running task motors
	sys.taskCreate(&tog);	//runing task tog
	while (true)
	{
		Serial.printf("fizz\n");
		sys.delay(1000);
	}
}
```

## Sensors ##

### lego touch ###

```cpp
#include <hFramework.h>

#include <Lego_Touch.h>

using namespace hSensors;

void hMain(void)
{
	sys.setLogDev(&Serial);
	hLegoSensor_simple ls(hSens5);
	Lego_Touch sensor(ls);
	
	for (;;)
	{
		bool pressed = sensor.isPressed();
		LED1.toggle();
		printf("state %d\r\n", pressed);
		sys.delay_ms(10);
	}
}
```
