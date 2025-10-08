# Week 4 notes

- Download and add fixed NewLib: https://nadler.com/embedded/newlibAndFreeRTOS.html
- Remove sysmem from build
- Add define for ISR_STACK_LENGTH_BYTES to FreeRTOSConfig.h
```C
#define ISR_STACK_LENGTH_BYTES 0x400  // 1 KB stack for interrupts)
```
- Ensure USE_NEWLIB_REENTRANT is enabled and Use FW pack heap file is disabled.

# Troubleshooting

- Context switching appeared to be happening according to the traceTaskSwitch code but simple blinking of LEDs was not happening.

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

- Found pin definitions were not correct using BSP (board support package) functions, so changed to pure HAL GPIO commands with custom defined pins.
- Priority of tasks seemed to also be an issue. Setting tasks to Normal, Normal1 and Normal2 priority worked. Will have to investigate why Normal, Low, Low, was not working. Perhaps need to try Low1, Low2?