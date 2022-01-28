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
#include "LCD_disp.h"
#include "sdcard.h"
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

sd_card_info_struct sd_cardinfo;                            /* information of SD card */
uint32_t buf_write[512];                                    /* store the data written to the card */
uint32_t buf_read[512];                                     /* store the data read from the card */
void nvic_config(void);
sd_error_enum sd_io_init(void);
void card_info_get(void);
void gd_eval_com_init();
uint32_t line_num;
/*!
\brief      main function
\param[in]  none
\param[out] none
\retval     none
*/
int main(void)
{
  sd_error_enum sd_error;
  uint16_t i = 5;
  line_num = 0;
  /* enable the led clock */
  rcu_periph_clock_enable(RCU_GPIOE);
  /* configure led GPIO port */ 
  gpio_init(GPIOE, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_4);
  
  gpio_bit_reset(GPIOE,GPIO_PIN_4);
  
  LCD_Init();
  
  nvic_config();
  /* initialize the card */
  do{
    sd_error = sd_io_init();
  }while((SD_OK != sd_error) && (--i));
  
  if(i){
    LCD_ShowString(0,16*(line_num++),"Card init success!\r\n");
  }else{
    LCD_ShowString(0,16*(line_num++),"Card init failed!\r\n");
    while (1){
    }
  }
  
  /* get the information of the card and print it out by USART */
  card_info_get();
  
  /* init the write buffer */
  for(i=0; i<512; i++){
    buf_write[i] = i;
  }
  
  LCD_ShowString(0,16*(line_num++),"Card test:\r\n");
  
  /* single block operation test */
  sd_error = sd_block_write(buf_write, 100*512, 512);
  if(SD_OK != sd_error){
    LCD_ShowString(0,16*(line_num++),"Block write fail!\r\n");
    while (1){
    }
  }else{
    LCD_ShowString(0,16*(line_num++),"Block write success!\r\n");
  }
  sd_error = sd_block_read(buf_read, 100*512, 512);
  if(SD_OK != sd_error){
    LCD_ShowString(0,16*(line_num++),"Block read fail!\r\n");
    while (1){
    }
  }else{
    LCD_ShowString(0,16*(line_num++),"Block read success!\r\n");
  }
  
  while(1){
    
    gpio_bit_reset(GPIOE,GPIO_PIN_4);
    SysCtlDelay(500*(SystemCoreClock/3000));
    
    gpio_bit_set(GPIOE,GPIO_PIN_4);
    SysCtlDelay(500*(SystemCoreClock/3000));
  }
}


/*!
\brief      configure the NVIC
\param[in]  none
\param[out] none
\retval     none
*/
void nvic_config(void)
{
  nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
  nvic_irq_enable(SDIO_IRQn, 0, 0);
}

/*!
\brief      initialize the card, get the card information, set the bus mode and transfer mode
\param[in]  none
\param[out] none
\retval     sd_error_enum
*/
sd_error_enum sd_io_init(void)
{
  sd_error_enum status = SD_OK;
  uint32_t cardstate = 0;
  status = sd_init();
  if(SD_OK == status){
    status = sd_card_information_get(&sd_cardinfo);
  }
  if(SD_OK == status){
    status = sd_card_select_deselect(sd_cardinfo.card_rca);
  }
  status = sd_cardstatus_get(&cardstate);
  if(cardstate & 0x02000000){
    LCD_ShowString(0,16*(line_num++),"the card is locked!\r\n");
    while (1){
    }
  }
  if ((SD_OK == status) && (!(cardstate & 0x02000000)))
  {
    /* set bus mode */
    status = sd_bus_mode_config(SDIO_BUSMODE_4BIT);
    //        status = sd_bus_mode_config( SDIO_BUSMODE_1BIT );
  }
  if (SD_OK == status)
  {
    /* set data transfer mode */
    status = sd_transfer_mode_config( SD_DMA_MODE );
    //        status = sd_transfer_mode_config( SD_POLLING_MODE );
  }
  return status;
}

/*!
\brief      get the card information and print it out by USRAT
\param[in]  none
\param[out] none
\retval     none
*/
void card_info_get(void)
{
  uint8_t sd_spec, sd_spec3, sd_spec4, sd_security;
  uint32_t block_count, block_size;
  char LCD_string[200];
  uint16_t temp_ccc;
  LCD_ShowString(0,16*(line_num++),"Card information:\r\n");
  sd_spec = (sd_scr[1] & 0x0F000000) >> 24;
  sd_spec3 = (sd_scr[1] & 0x00008000) >> 15;
  sd_spec4 = (sd_scr[1] & 0x00000400) >> 10;
  if(2 == sd_spec)
  {
    if(1 == sd_spec3)
    {
      if(1 == sd_spec4) 
      {
        LCD_ShowString(0,16*(line_num++),"## Card version 4.xx ##\r\n");
      }
      else 
      {
        LCD_ShowString(0,16*(line_num++),"## Card version 3.0x ##\r\n");
      }
    }
    else 
    {
      LCD_ShowString(0,16*(line_num++),"## Card version 2.00 ##\r\n");
    }
  }
  else if(1 == sd_spec) 
  {
    LCD_ShowString(0,16*(line_num++),"## Card version 1.10 ##\r\n");
  }
  else if(0 == sd_spec) 
  {
    LCD_ShowString(0,16*(line_num++),"## Card version 1.0x ##\r\n");
  }
  
  sd_security = (sd_scr[1] & 0x00700000) >> 20;
  if(2 == sd_security) 
  {
    LCD_ShowString(0,16*(line_num++),"## SDSC card ##\r\n");
  }
  else if(3 == sd_security) 
  {   
    LCD_ShowString(0,16*(line_num++),"## SDHC card ##\r\n");
  }
  else if(4 == sd_security) 
  {
    LCD_ShowString(0,16*(line_num++),"## SDXC card ##\r\n");
  }
  
  block_count = (sd_cardinfo.card_csd.c_size + 1)*1024;
  block_size = 512;
  sprintf(LCD_string,"## Device size is %dKB ##\r\n", sd_card_capacity_get());
  LCD_ShowString(0,16*(line_num++),(uint8_t*)LCD_string);
  sprintf(LCD_string,"## Block size is %dB ##\r\n", block_size);
  LCD_ShowString(0,16*(line_num++),(uint8_t*)LCD_string);
  sprintf(LCD_string,"## Block count is %d ##\r\n", block_count);
  LCD_ShowString(0,16*(line_num++),(uint8_t*)LCD_string);
  
  if(sd_cardinfo.card_csd.read_bl_partial){
    LCD_ShowString(0,16*(line_num++),"## Partial blocks for read allowed ##\r\n");
  }
  if(sd_cardinfo.card_csd.write_bl_partial){
    LCD_ShowString(0,16*(line_num++),"## Partial blocks for write allowed ##\r\n");
  }
  temp_ccc = sd_cardinfo.card_csd.ccc;
  sprintf(LCD_string,"## CardCommandClasses is: %x ##\r\n", temp_ccc);
  LCD_ShowString(0,16*(line_num++),(uint8_t*)LCD_string);
  if((SD_CCC_BLOCK_READ & temp_ccc) && (SD_CCC_BLOCK_WRITE & temp_ccc)){
    LCD_ShowString(0,16*(line_num++),"## Block operation supported ##\r\n");
  }
  if(SD_CCC_ERASE & temp_ccc){
    LCD_ShowString(0,16*(line_num++),"## Erase supported ##\r\n");
  }
  if(SD_CCC_WRITE_PROTECTION & temp_ccc){
    LCD_ShowString(0,16*(line_num++),"## Write protection supported ##\r\n");
  }
  if(SD_CCC_LOCK_CARD & temp_ccc){
    LCD_ShowString(0,16*(line_num++),"## Lock unlock supported ##\r\n");
  }
  if(SD_CCC_APPLICATION_SPECIFIC & temp_ccc){
    LCD_ShowString(0,16*(line_num++),"## Application specific supported ##\r\n");
  }
  if(SD_CCC_IO_MODE & temp_ccc){
    LCD_ShowString(0,16*(line_num++),"## I/O mode supported ##\r\n");
  }
  if(SD_CCC_SWITCH & temp_ccc){
    LCD_ShowString(0,16*(line_num++),"## Switch function supported ##\r\n");
  }
}