# Week 3 Progress Report

## Introduction

During the third week, the group started integrating an accelerometer sensor using the I<sup>2</sup>C interface. The objective is to be able to read sensor input data with bare metal at first and prepare for the next task, which is scheduling tasks under a real-time operating system (RTOS). 

## Procedure

This week’s work was primarily involved in understanding the I<sup>2</sup>C and using I<sup>2</sup>C peripheral in STM32CubeIDE. We had to find the correct accelerometer datasheets and review the addressing schemes. We have began to develop initialization and read routines for I<sup>2</sup>C communication. Our group's next plan involves implementing RTOS (FreeRTOS) to manage tasks such as GPIO control, UART communication, and sensor data handling concurrently.

## Key Code

- None
    
## Results
By the end of this week, we have the framework for interacting with the accelerometer over I<sup>2</sup>C, however so far we have been unsuccessful in retrieving data from the sensor.