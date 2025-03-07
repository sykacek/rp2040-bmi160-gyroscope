# ABOUT

This project demostrates how to connect [BMI160 IMU](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160/) and [RP2040 MCU](https://www.raspberrypi.com/products/rp2040/). Data from BMI160
are written to RP2040 internal flash memory for later data processing. MCU gets the IMU data with frequency 10Hz and writes them to flash memory every 2 seconds (0.5 Hz).
Staring at offset `0x42000` there is enough memory to read for 4 hours.

## Applications

This project was intented as cheap alternative of altimeter for hobby rockets, planes etc.

## Dependencies

The program is based on [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk), so install it first.
No other dependencies are needed. To install Raspberry Pi Pico SDK please refer to [this page](https://github.com/raspberrypi/pico-sdk/blob/master/README.md).

## How to build

First, make sure that environmental variable `PICO_SDK_PATH` is pointing to your Pico SDK directory and then build as usual
CMake project:
```sh
	mkdir build
	cd build
	cmake ..
	make -j4
```
Note: `-j4` parameter is optional, is speeds up the build process using 4 threads, you can also build the project using just `make`.

## Flash the firmware

You can flash the firmware manually by copying `.uf2` file to the RP2040 mount point, or this process can be handled by CMake.
Change the line 33 in `CMakeLists.txt` to the desired mount point
```cmake
	cp ${PROJECT_NAME}.uf2 /media/user/RPI-RP2
```
To flash the firmware using make run
```sh
	make flash
```

## Get the data

To get the data out from flash memory you need to extract the memory image out of the MCU and save it to your computer. 
This can be done using [picotool](https://www.raspberrypi.com/products/rp2040/).
For building picotool please refer to [this page](https://github.com/raspberrypi/picotool/blob/master/README.md).
To save the memory image run
```sh
	picotool save -a test.bin
```
And your data will be present from offset `0x42000`

## Data format

Since RP2040 can write to internal flash only page aligned buffer (256 byte alignment), there is a 16 byte header present at each page beginning, next 240 bytes are 120 word long IMU values. In terms of C that could be presented as
```c
	struct page {
		char header[16]	= "DATA AB DATA ABC";
		int16_t accel[3];
		int16_t gyro[3];

		/* IMU data repeat 20 times */
	}
```