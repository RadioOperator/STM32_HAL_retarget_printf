## STM32_HAL_retarget_printf

Retarget the printf function, sample projects for STM32 HAL under MDK/Keil, 4 methods in one ... ...

Refer to main.c file, all key-code here:

```
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//Debug mode On/Off switch 
#define DEBUG                   1 //set=0, disable all LOG lines

//select one of the following "printf" retarget methods
#define PRINTF_UART             0 //printf using UART port, 1-use this, 0-not
#define PRINTF_RTT              1 //printf using Segger RTT, 1-use this, 0-not
#define PRINTF_SWO              0 //printf using SWO port, 1-use this, 0-not
#define PRINTF_CDC              0 //printf using USBD_CDC, 1-use this, 0-not

#if ((PRINTF_UART + PRINTF_CDC + PRINTF_SWO + PRINTF_RTT) != 1)
#error "!!!!!! printf retarget function Not define or Multi-defined !!!!!!"
#endif

#if DEBUG
  #if PRINTF_RTT
    #include "SEGGER_RTT.h"
    #define LOG(format, args...)  SEGGER_RTT_printf(0, "[%s:%d] "format, __FILE__, __LINE__, ##args)
    #define printf(format, args...)  SEGGER_RTT_printf(0, format, ##args)
    
  #else //#if PRINTF_RTT
    #define LOG(format, args...)  printf("[%s:%d] "format, __FILE__, __LINE__, ##args)
    
    #if PRINTF_CDC
    #include "usbd_cdc_if.h"
    #endif
    
    int fputc(int ch, FILE *f)
    {
      #if PRINTF_UART
        HAL_UART_Transmit(&huart3, (uint8_t*)(&ch), 1, 1000); //UART3
        
      #elif PRINTF_SWO
        ITM_SendChar(ch);
        
      #elif PRINTF_CDC
        uint8_t u8Temp = 0;
        while(CDC_Transmit_FS((uint8_t*)(&ch), 1) != USBD_OK)
        {
          HAL_Delay(1);
          if (++u8Temp >= 3) break;
        }
      #endif //#if PRINTF_UART
        
      return(ch);
    }
  #endif //#if PRINTF_RTT
  
#else //#if DEBUG == 0
  #define LOG(args...) //disable all LOGs when compiling
#endif //#if DEBUG
```

Enjoy!!!
