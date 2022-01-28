/*!
\file    main.c
\brief   led spark with systick, USART print and key example

\version 2015-07-15, V1.0.0, firmware for GD32F20x
\version 2017-06-05, V2.0.0, firmware for GD32F20x
\version 2018-10-31, V2.1.0, firmware for GD32F20x
\version 2020-09-30, V2.2.0, firmware for GD32F20x
*/

/*
Copyright (c) 2020, GigaDevice Semiconductor Inc.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors 
may be used to endorse or promote products derived from this software without 
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f20x.h"
#include <stdio.h>
#include "main.h"

/*120Mhz时钟时，当ulCount为1时，函数耗时3个时钟，延时=3*1/120us=1/40us*/
/*
SystemCoreClock=120000000

us级延时,延时n微秒
SysCtlDelay(n*(SystemCoreClock/3000000));

ms级延时,延时n毫秒
SysCtlDelay(n*(SystemCoreClock/3000));

m级延时,延时n秒
SysCtlDelay(n*(SystemCoreClock/3));
*/

#if defined   (__CC_ARM) /*!< ARM Compiler */
__asm void
SysCtlDelay(unsigned long ulCount)
{
  subs    r0, #1;
  bne     SysCtlDelay;
  bx      lr;
}
#elif defined ( __ICCARM__ ) /*!< IAR Compiler */
void
SysCtlDelay(unsigned long ulCount)
{
  __asm("    subs    r0, #1\n"
        "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}

#elif defined (__GNUC__) /*!< GNU Compiler */
void __attribute__((naked))
SysCtlDelay(unsigned long ulCount)
{
  __asm("    subs    r0, #1\n"
        "    bne     SysCtlDelay\n"
          "    bx      lr");
}

#elif defined  (__TASKING__) /*!< TASKING Compiler */                           
/*无*/
#endif /* __CC_ARM */

void uart_init()
{    
  /* enable GPIO clock */
  rcu_periph_clock_enable(RCU_GPIOA);
  
  /* enable USART clock */
  rcu_periph_clock_enable(RCU_USART0);
  
  /* connect port to USARTx_Tx */
  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
  
  /* connect port to USARTx_Rx */
  gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
  
  /* USART configure */
  usart_deinit(USART0);
  usart_baudrate_set(USART0, 115200U);
  usart_receive_config(USART0, USART_RECEIVE_ENABLE);
  usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
  usart_enable(USART0);
}
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
  usart_data_transmit(USART0, (uint8_t)ch);
  while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
  return ch;
}
/*!
\brief      main function
\param[in]  none
\param[out] none
\retval     none
*/
int main(void)
{
  /* enable the led clock */
  rcu_periph_clock_enable(RCU_GPIOE);
  /* configure led GPIO port */ 
  gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_3);
  
  gpio_bit_reset(GPIOE,GPIO_PIN_3);
  uart_init();
  printf("\r\n======================================================================");
  printf("\r\n=               (C) COPYRIGHT 2022                                   =");
  printf("\r\n=                                                                    =");
  printf("\r\n=                GD32F20x USART_Printf                               =");
  printf("\r\n=                                                                    =");
  printf("\r\n=                                           By Firefly               =");
  printf("\r\n======================================================================");
  printf("\r\n\r\n");
  while(1){
    
    gpio_bit_reset(GPIOE,GPIO_PIN_3);
    SysCtlDelay(500*(SystemCoreClock/3000));
    
    gpio_bit_set(GPIOE,GPIO_PIN_3);
    SysCtlDelay(500*(SystemCoreClock/3000));
  }
}

