# Week 3 Progress Report

## Introduction

During the third week, the group started integrating an accelerometer sensor using the I<sup>2</sup>C interface. The objective is to be able to read sensor input data with bare metal at first and prepare for the next task, which is scheduling tasks under a real-time operating system (RTOS). 

## Procedure

This week’s work was primarily involved in understanding the I<sup>2</sup>C and using I<sup>2</sup>C peripheral in STM32CubeIDE. We had to find the correct accelerometer datasheets and review the addressing schemes. We have began to develop initialization and read routines for I<sup>2</sup>C communication. Our group's next plan involves implementing RTOS (FreeRTOS) to manage tasks such as GPIO control, UART communication, and sensor data handling concurrently.

## Key Code

### Sending Data / Setting Registers
```C
REGISTER = DATA_FORMAT; //Defined elsewhere as HEX address for desired register
DATA = 0x00; //Data to be written
DATA_WRITE[0] = REGISTER; //Form an array with the address to write to, along with the data
DATA_WRITE[1] = DATA;
ret = HAL_I2C_Master_Transmit(&hi2c1,ADDR,DATA_WRITE,2,HAL_MAX_DELAY); //Send data over I2C with maximum HAL timeout
```

### Recieving Data

```C
REGISTER = DATA_X0; //Set address of read start address
uint8_t RAWACCEL[6]; //Create buffer to hold read data
HAL_I2C_Mem_Read(&hi2c1, ADDR, DATA_X0, I2C_MEMADD_SIZE_8BIT, RAWACCEL, 6, HAL_MAX_DELAY); //Read 6 bytes starting at address.
```

## Results

By the end of this week, we have the framework for interacting with the accelerometer over I<sup>2</sup>C, however so far we have been unsuccessful in retrieving data from the sensor.

## References

- https://github.com/DFRobot/DFRobot_ADXL345/blob/main/DFRobot_ADXL345.h
- https://blog.embeddedexpert.io/?p=1298
- https://community.st.com/t5/stm32-mcus-products/i-don-t-understand-how-to-use-the-hal-i2c-master-transmit/td-p/582824
- https://www.st.com/resource/en/user_manual/dm00499160.pdf
- https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
- https://os.mbed.com/platforms/ST-Nucleo-H743ZI2/
