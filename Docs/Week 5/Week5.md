# Week 5 Progress Report

## Introduction

During the fifth week, the troubleshooting from last week was continued. Additionally, we are to add queues and semaphores to the project as well as switch some commands from the CMSIS-RTOS API to the native FreeRTOS API.

## Procedure

1. To simplify debugging, the task contents were commented out and replaced with simple LED toggles.
2. First, the three LED's on the board would not light. After consulting the manual, the pins being used by the BSP library for some of the LEDs (primarily the yellow LED) did not match. 
3. This issue was corrected by switching from the BSP pin definitions and functions to manually defined GPIO pins controlled by HAL.
4. Secondly, the task priorities were not working correctly. Despite tasks appearing to run based on the traceTaskSwitch code (see below), the debugging LED commands placed in each task would not all toggle.
5. By changing the task priorities from Normal, Low, Low to Normal, Low1, Low2, the tasks were able to run correctly.


## Key Code

```C
osThreadId_t sensorReadTaskHHandle;
const osThreadAttr_t sensorReadTaskH_attributes = {
  .name = "sensorReadTaskH",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for contTaskHandle */
osThreadId_t contTaskHandleHandle;
const osThreadAttr_t contTaskHandle_attributes = {
  .name = "contTaskHandle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for actuatorTaskH */
osThreadId_t actuatorTaskHHandle;
const osThreadAttr_t actuatorTaskH_attributes = {
  .name = "actuatorTaskH",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
```

## Results

As the end of this week, the code has finally been troubleshot. The tasks now function correctly, with LEDs toggling to indicate output from the control task, and serial printouts indicating a detected actuation condition. Queues, semaphores, and conversion to FreeRTOS API tasks have been pushed to next week.

## References

- https://github.com/DFRobot/DFRobot_ADXL345/blob/main/DFRobot_ADXL345.h
- https://blog.embeddedexpert.io/?p=1298
- https://community.st.com/t5/stm32-mcus-products/i-don-t-understand-how-to-use-the-hal-i2c-master-transmit/td-p/582824
- https://www.st.com/resource/en/user_manual/dm00499160.pdf
- https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
- https://os.mbed.com/platforms/ST-Nucleo-H743ZI2/
- https://nadler.com/embedded/newlibAndFreeRTOS.html