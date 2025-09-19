# MENG-4030-Industrial-Conveyor-Belt-Control
RTOS Industrial Conveyor Belt Control assignments and code for Humber Polytechnic MENG-4030

## Summary of problem (from perspective of client)
Our conveyor belt systems are critical components in our production process. They move
materials from one stage of production to another, and any issues or downtime on these lines
can have significant consequences. That's why I'm excited to be working with an engineer (like
yourself) to design a solution that will improve the reliability and efficiency of our conveyor belt
monitoring system.
In particular, we're experiencing some problems with jamming and stalling on one of our
production lines. Our current manual intervention-based approach is not only time-consuming
but also prone to human error. We need a more automated and proactive system that can
detect issues in real-time and take corrective action before they become major problems.

## Background and purpose:
ACME Inc. is a mid-sized manufacturing company with over 50 years of experience producing
high-quality industrial equipment for industries such as automotive, aerospace, and
construction. Over the decades, the company has built a strong reputation for delivering reliable
and innovative products, and its goal is to remain a trusted partner for companies seeking
custom-made solutions tailored to their specific needs.
In recent years, ACME has experienced significant growth, bringing both new challenges and
opportunities. One of its top priorities is improving production line efficiency and minimizing
downtime. To address this, ACME has launched a project focused on designing a real-time
embedded system to monitor and control its conveyor belts.

Currently, ACME’s conveyor belt systems require a high level of manual oversight, which
increases the chance of errors, delays, and safety risks. By introducing an automated
monitoring and control system, the company aims to reduce human error, boost productivity,
and enhance overall manufacturing efficiency, ensuring it continues to deliver the level of quality
and reliability that customers expect.

## System architecture
The system hardware will be comprised of a STM32 MCU, along with IR distance, hall effect and
accelerometer sensors. The system will actuate a stepper motor and, in a fault condition, a
piezoelectric buzzer.
The system software will consist of a FreeRTOS operating system, as well as HAL drivers for
communication with the various peripherals and sensors. The RTOS will have tasks for polling
sensors, stall detection from the sensor data, actuation of the motor and buzzer, and a logging
task to keep a record of faults and the sensor that caused the fault condition.

## Requirements & Specifications

| Client requirement | Technical specification | How it will be verified? |
|---|---|---|
| Realtime reaction | 100-200ms reaction time  | Logging and timers will be used to track the amount of time between a sensor detecting a stall and the motor command to stop the conveyor |
| Minimal power impact on existing line | 10-20 watts max consumption | Voltage and current meters can be used on the prototype to gauge the power draw |

## Hardware Selections

| Part | Description |
|---|---|
| STM32H7 NUCLEO-H743ZI | STM32-based microcontroller |
| Diligent PmodACL Accelerometer | PMOD/I<sup>2</sup>C Accelerometer |
| 105990072 Seeed Technology Co., Ltd | 5V DC stepper motor + driver |
| Buzzer | Piezoelectric Buzzer |
| Hall effect sensor | 3.3V/5V hall effect current sensor |
| Sharp GP2Y0A21 IR Distance Sensor | IR distance sensor with digital output |

## Software platform:
### IDE and related tools 
- STM32CubeMX
### Programming language
- C
### Libraries and packages
- HAL and I2C libraries

## Resources
- B. Amos, Hands-on RTOS with microcontrollers : building real-time embedded systems using
FreeRTOS, STM32 MCUs, and SEGGER debug tools, 1st ed. Birmingham: Packt Publishing,2020.
