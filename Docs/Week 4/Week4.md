# Week 4 Progress Report

## Introduction

During the fourth week, the group started working with FreeRTOS. The code from previous weeks was used to define RTOS tasks and some new code was written for a control task to make the code make a decision on when to actuate the actuator based on the sensor readings.

## Procedure

1. To port the existing code to FreeRTOS, the FreeRTOS middleware package was added to the STM32CUBEIDE project. 
2. CMSIS_V2 was selected and under the tasks and queues section, and three tasks were defined.
3. One task was made for the sensor read, it had a task handle defined, and a priority of osPriorityNormal set.
4. One task was made for the control task, it had a task handle defined, and a priority of osPriorityLow set.
5. One task was made for the actuator command, it had a task handle defined, and a priority of osPriorityLow set.
6. The new code was generated, and the relevant sections of code were copied from Lab 3.
7. The first build created all sorts of errors, and after some research it was found that there is a bug with NewLib and FreeRTOS/CMSIS. This is due to using USE_NEWLIB_REENTRANT and the snprintf function.
8. To correct the bug, the steps from https://nadler.com/embedded/newlibAndFreeRTOS.html were completed. In summary, they were:
	1. Download the fixed NewLib from nadler.com.
	2. Remove the sysmem.c file from being built.
	3. Add define for ISR_STACK_LENGTH_BYTES to FreeRTOSConfig.h.
	```C
	#define ISR_STACK_LENGTH_BYTES 0x400  // 1 KB stack for interrupts)
	```
	4. Ensure USE_NEWLIB_REENTRANT is enabled and Use FW pack heap file is disabled (Middleware and Software -> FREERTOS -> Advanced Settings).
9. This fix corrected the errors related to the snprintf function and the code could now be run.
10. Upon trying to run the code, the tasks seemed to be running, as shown by the traceTaskSwitch code, but the actions contained in the tasks are not being completed.

## Key Code

```C
void traceTaskSwitch (void)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		uint8_t buf1[50];
		snprintf(buf, sizeof(buf1), "Core%d is running -> %s\r\n",
		(int)HAL_GetCurrentCPUID(), pcTaskGetName(NULL));
		HAL_UART_Transmit(&huart2, buf1, strlen(buf1), HAL_MAX_DELAY);
	}
```

## Results

As the end of this week, we have been able to switch our code to FreeRTOS, however due to remaining issues, not all tasks seem to be running properly. Troubleshooting will continue into next week.

## References

- https://github.com/DFRobot/DFRobot_ADXL345/blob/main/DFRobot_ADXL345.h
- https://blog.embeddedexpert.io/?p=1298
- https://community.st.com/t5/stm32-mcus-products/i-don-t-understand-how-to-use-the-hal-i2c-master-transmit/td-p/582824
- https://www.st.com/resource/en/user_manual/dm00499160.pdf
- https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
- https://os.mbed.com/platforms/ST-Nucleo-H743ZI2/
- https://nadler.com/embedded/newlibAndFreeRTOS.html