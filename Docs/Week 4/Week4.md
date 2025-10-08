{\rtf1\ansi\ansicpg1252\cocoartf2865
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\fswiss\fcharset0 Helvetica;\f1\fnil\fcharset0 Menlo-Bold;\f2\fnil\fcharset0 Menlo-Regular;
}
{\colortbl;\red255\green255\blue255;\red127\green0\blue85;\red63\green127\blue95;}
{\*\expandedcolortbl;;\csgenericrgb\c49804\c0\c33333;\csgenericrgb\c24706\c49804\c37255;}
\margl1440\margr1440\vieww11520\viewh8400\viewkind0
\pard\tx566\tx1133\tx1700\tx2267\tx2834\tx3401\tx3968\tx4535\tx5102\tx5669\tx6236\tx6803\pardirnatural\partightenfactor0

\f0\fs24 \cf0 # Week 4 notes\
\
- Download and add fixed NewLib: {\field{\*\fldinst{HYPERLINK "https://nadler.com/embedded/newlibAndFreeRTOS.html"}}{\fldrslt https://nadler.com/embedded/newlibAndFreeRTOS.html}}\
- Remove sysmem from build\
- Add define for ISR_STACK_LENGTH_BYTES to FreeRTOSConfig.h\

\f1\b \cf2 ```C\
#define
\f2\b0 \cf0  ISR_STACK_LENGTH_BYTES 0x400  \cf3 // 1 KB stack for interrupts)\
```\

\f0 \cf0 - Ensure USE_NEWLIB_REENTRANT is enabled and Use FW pack heap file is disabled.}