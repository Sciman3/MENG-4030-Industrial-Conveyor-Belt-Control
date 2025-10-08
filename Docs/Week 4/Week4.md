# Week 4 notes

- Download and add fixed NewLib: https://nadler.com/embedded/newlibAndFreeRTOS.html
- Remove sysmem from build
- Add define for ISR_STACK_LENGTH_BYTES to FreeRTOSConfig.h
```C
#define ISR_STACK_LENGTH_BYTES 0x400  // 1 KB stack for interrupts)
```
- Ensure USE_NEWLIB_REENTRANT is enabled and Use FW pack heap file is disabled.