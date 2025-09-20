

# Week 2 Progress Report

## Introduction

This week, we installed the STM32 IDE, got our boards and peripherals, and got started programming the boards.
To get aquatinted to working with the boards, our task was to blink an LED and set up serial communication over
the virtual com port.

## Procedure

Since this was the first hands-on experience with STM32 hardware, our group initially faced difficulties with pin mapping and assignments. Understanding the board’s labeling and correctly configuring pins in STM32CubeIDE required extra time and effort. After resolving these issues, our group successfully configured PE1 and PE14 as GPIO outputs and was able to toggle the onboard LEDs with varying delays. Following this, we progressed to UART serial communication. Multiple UART interfaces were tested before concluding that USART3 was the most practical choice for keyboard input and serial output. A basic “Hello World” program was implemented, and the team began testing character echoing from keyboard input. Some of the challenges we faced during this week include, determining the correct placement of HAL_UART_Transmit() and HAL_UART_Receive_IT() calls, understanding buffer sizes and ensuring single-byte handling for keyboard input. At the end we were able to toggle the onboard LEDs successfully using GPIO outputs. we had our first “Hello World” message displayed via UART3. Moreover, basic keyboard input reception was achieved.

## Key Code

### Blink LED Code

```C
HAL_GPIO_TogglePin (GPIOB, GPIO_PIN_0); //Toggle a GPIO pin via HAL
BSP_LED_Toggle(LED_RED); //Toggle a predefined pin using BSP

### UART

```C
uint8_t Test[] = "Hello World !!!\n\n"; //Data to send
HAL_UART_Transmit(&huart3,Test,strlen((char*)Test),10);// Sending in normal mode

### LED and UART together

```C
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t Test[] = "Hello World !!!\n\n"; //Data to send
  HAL_UART_Transmit(&huart3,Test,strlen((char*)Test),10);// Sending in normal mode
  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (LED_STATUS) {
		  uint8_t Test[] = "Green LED is ON, Turning OFF.\n\n"; //Data to send
		  HAL_UART_Transmit(&huart3,Test,strlen((char*)Test),10);// Sending in normal mode
		  BSP_LED_Off(LED_GREEN);
		  LED_STATUS = 0;
	  } else {
		  uint8_t Test[] = "Green LED is OFF, Turning ON.\n\n"; //Data to send
		  HAL_UART_Transmit(&huart3,Test,strlen((char*)Test),10);// Sending in normal mode
		  BSP_LED_On(LED_GREEN);
		  LED_STATUS = 1;
	  }
	  HAL_Delay(1000);
  }
  /* USER CODE END 3 */
    
## Results
By the end of this week, we were able to control an LED on the board, as well as send text over UART to a terminal.

![Animated GIF showing led turning on and off, and text in a terminal window describing the LED state](https://github.com/Sciman3/MENG-4030-Industrial-Conveyor-Belt-Control/blob/47d998b4557f9fac71b9e6cb6ef74eb566dd7bd7/Results/Week%202/SerialData_LED.gif)

![Closeup of text in a terminal window describing the LED state](https://github.com/Sciman3/MENG-4030-Industrial-Conveyor-Belt-Control/blob/47d998b4557f9fac71b9e6cb6ef74eb566dd7bd7/Results/Week%202/Serial%20Data.png)