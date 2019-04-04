#include "var.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_conf.h"
//#include "stdlib.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_flash.h"
#include "stdio.h"
#include "ft812.h"

GPIO_InitTypeDef GPIO;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
SPI_InitTypeDef SPI_InitStructure;
DMA_InitTypeDef DMA;
USART_InitTypeDef USART;

I2C_InitTypeDef I2C_InitStructure;

void system_init();
void SPI2_IRQHandler();
void TIM7_IRQHandler();
void USART3_IRQHandler();
void lcd_init();
//void Delay_ms(__IO uint32_t nTime);



unsigned char flag_direction = 0, res_position = 0, flag_spi_send_time = 0;
unsigned short encoder_timer = 0;
unsigned char encoder_1_state = 0, encoder_1_pstate = 0; 
unsigned char encoder_2_state = 0, encoder_2_pstate = 0; 
unsigned char encoder_3_state = 0, encoder_3_pstate = 0;
unsigned char encoder_4_state = 0, encoder_4_pstate = 0;
unsigned char encoder_5_state = 0, encoder_5_pstate = 0;
unsigned char encoder_6_state = 0, encoder_6_pstate = 0;
unsigned char encoder_7_state = 0, encoder_7_pstate = 0;
unsigned char encoder_8_state = 0, encoder_8_pstate = 0;
unsigned char encoder_9_state = 0, encoder_9_pstate = 0;
unsigned char encoder_10_state = 0, encoder_10_pstate = 0;
unsigned char encoder_11_state = 0, encoder_11_pstate = 0;
unsigned char encoder_12_state = 0, encoder_12_pstate = 0;

uint16_t encoder_1_counter = 250, encoder_2_counter = 42, encoder_3_counter = 250, encoder_4_counter = 42; 
uint16_t encoder_5_counter = 250, encoder_6_counter = 42, encoder_7_counter = 250, encoder_8_counter = 42;
uint16_t encoder_9_counter = 250, encoder_10_counter = 42, encoder_11_counter = 250, encoder_12_counter = 42; 
uint16_t encoder_1_value = 0, encoder_2_value = 0, encoder_3_value = 0, encoder_4_value = 0;
uint16_t encoder_5_value = 0, encoder_6_value = 0, encoder_7_value = 0, encoder_8_value = 0; 
uint16_t encoder_9_value = 0, encoder_10_value = 0, encoder_11_value = 0, encoder_12_value = 0;
uint16_t delay_c = 0;

uint8_t spi_data_buf[100];
uint8_t spi_cs_buf[100];
uint16_t spi_head = 0, spi_tail = 0;
uint8_t spi_queue = 0;
uint16_t spi_delay = 0;

//DAC`s SYNC as CS
GPIO_TypeDef* cs_port[3] = {GPIOE, GPIOD, GPIOC};
uint16_t cs_pin[3] = {GPIO_Pin_15, GPIO_Pin_12, GPIO_Pin_7};
//ADC CS 1&2 for each channel
GPIO_TypeDef* adc_cs_port[6] = {GPIOE, GPIOB, GPIOB, GPIOD, GPIOD, GPIOC};
uint16_t adc_cs_pin[6] = {GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_9};

uint16_t av_buf[16][20];
uint8_t head = 0, av_count = 0, average_delay = 2;
uint32_t  adc_av_buf[16];


void delay_nop(uint16_t delay_v){
  for(uint16_t del_v = 0; del_v < delay_v; del_v++)asm("nop");
  
}

void spi_custom(uint8_t cs, uint32_t dt){
/*cs - cs line number
  dt - data to slave
*/
    TIM_Cmd(TIM7, DISABLE);
    //24bit data
    GPIO_ResetBits(cs_port[cs],cs_pin[cs]);
    for(uint8_t s = 0;s<24;s++){
        if((dt<<s)&0x800000) DATA_HIGH;
        else DATA_LOW;
        delay_nop(100);
        CLK_LOW;
        delay_nop(100);

        CLK_HIGH;
        //delay_nop(100);
  
    }
    delay_nop(100);
    
    GPIO_SetBits(cs_port[cs],cs_pin[cs]);
    
    TIM_Cmd(TIM7, ENABLE);
}
uint16_t adc_data_via_spi(uint8_t cs_num, uint8_t adc_ch_num){
    //dt - data into adc, di - data from adc
    TIM_Cmd(TIM7, DISABLE);  
    uint16_t dt = 0, di = 0;
    if(adc_ch_num == 0) dt = 0x0000;
    else if(adc_ch_num == 1) dt = 0x0800;
    else if(adc_ch_num == 2) dt = 0x1000;
    else if(adc_ch_num == 3) dt = 0x1800;
    else dt = 0x00;
    GPIO_ResetBits(adc_cs_port[cs_num], adc_cs_pin[cs_num]);
    for(uint8_t s = 0; s<16; s++){
        if((dt<<s)&0x8000) DATA_HIGH;
        else DATA_LOW;  
        CLK_LOW;
        delay_nop(1000);
        di = (di << 1) | (DATA_IN & 0x01);
        CLK_HIGH;
        delay_nop(1000); 
    }
    delay_nop(100);
    
    GPIO_SetBits(adc_cs_port[cs_num],adc_cs_pin[cs_num]);
        
    TIM_Cmd(TIM7, ENABLE);  
    
    return (0x0fff & di);    
}


void adc_voltage_monitor_init(){
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    
    ADC_InitTypeDef ADC_InitStructure; 
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 1, ADC_SampleTime_480Cycles);
  
    ADC_TempSensorVrefintCmd(ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    //ADC_SoftwareStartConv(ADC1);
}


void adc_dma_init(){

    
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    
    ADC_InitTypeDef ADC_InitStructure; 
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_NbrOfConversion = 16;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_112Cycles);  
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 3, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 4, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 5, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 6, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 7, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 8, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 9, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 10, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 11, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 12, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 13, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 14, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 15, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 16, ADC_SampleTime_112Cycles);
    
/*    
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_112Cycles);  
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 3, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 4, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 5, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 6, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 7, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 8, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 9, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 10, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 11, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 12, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 13, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 14, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 15, ADC_SampleTime_112Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 16, ADC_SampleTime_112Cycles);    
*/
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);    
      
    //ADC_DiscModeCmd(ADC1, DISABLE);
    //ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);       
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    ADC_DMACmd(ADC1, ENABLE);
    //ADC_Cmd(ADC1, ENABLE);
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);   
    DMA_InitTypeDef DMA_InitStructure;     
    DMA_DeInit(DMA2_Stream0);
    DMA_InitStructure.DMA_BufferSize = 16;
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOStatus_HalfFull;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &adc_buf[0];
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream0, ENABLE);   
       
    ADC_SoftwareStartConv(ADC1);
}



uint16_t flash_read(uint32_t address)
{
  return (*(__IO uint16_t*) address);
}

void save_voltage_values(uint16_t *val){
      
        
        
        FLASH_Unlock();
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
      
        FLASH_EraseSector(FLASH_Sector_3, VoltageRange_3);
        for(uint8_t a=0; a<10; a++){
          FLASH_ProgramHalfWord(ADRESS_FLASH + 2*a, val[a]);
          watch_data[a] = val[a];
        }
        //FLASH_ProgramHalfWord(ADRESS_FLASH + 32, counter_FlashWrite + 1);//
        FLASH_Lock();  
}

void read_voltage_values(){
  
  for(uint8_t a=0; a<10;a++){
    temp_voltage_value[a] = flash_read(ADRESS_FLASH + a*2);
  } 
}






void disp_8ch_formated(uint16_t *adc_data){

  float temp_buf[16];
  
  
  for(uint8_t disp_i = 0;disp_i<8;disp_i++){
    temp_buf[2*disp_i] = 6.6*((float)adc_data[2*disp_i]/4095);//voltage channels
    temp_buf[2*disp_i + 1] = 33*((float)adc_data[2*disp_i+1]/4095);//current channels
    
  }
  for(uint8_t disp_i = 0; disp_i < 4;disp_i++){
    
    
    sprintf(string[4*disp_i],"%d: %3.2fV", (2*disp_i+1),temp_buf[4*disp_i]);//positive voltage channels
    sprintf(string[4*disp_i+2],"%d: -%3.2fV", (2*disp_i+2),temp_buf[4*disp_i+2]);//negative voltage channels                                                             
    
    if(temp_buf[4*disp_i + 1] > 10)sprintf(string[4*disp_i+1],"  %4.1fmA", temp_buf[4*disp_i+1]);//positive current channels
    else sprintf(string[4*disp_i+1],"   %3.1fmA", temp_buf[4*disp_i+1]);//positive current channels
    
    if(temp_buf[4*disp_i + 3] > 10)sprintf(string[4*disp_i+3],"  -%4.1fuA", temp_buf[4*disp_i+3]);//negative current channels
    else sprintf(string[4*disp_i+3],"   -%3.1fuA", temp_buf[4*disp_i+3]);//negative current channels                                                             
    
    
  }
  
  for(uint8_t disp_i = 0; disp_i < 4; disp_i++){
    
    asm("nop");
  }
  
}
  


int main(void)
{

  for(i=0;i<1000000;i++);
  system_init();
  //adc_dma_init();
  //for(i=0;i<100000;i++);
  //read_voltage_values();
  DAC_RESET_OFF;
  TIM_Cmd(TIM7, ENABLE);

  DAC_LDAC_HIGH;  
  spi_custom(0, 0x3f73c0);
  DAC_LDAC_LOW;
  for(uint8_t asm_delay = 0; asm_delay < 255; asm_delay++);
  DAC_LDAC_HIGH; 
      
  for(;;)
  {   
    /*
    if(average_delay == 0){
        av_buf[head++] = adc_buf[1];
        if(head >= 10) head = 0;
        if(av_count < 10)av_count++;
        
        for(uint8_t av_i = 0; av_i < av_count;av_i++){
          
          adc_av_buf[1] += av_buf[av_i];
          
        }
        adc_av_buf[1] = adc_av_buf[1] / (av_count+1);
        average_delay = 30;
    } 
    */

//save current values in flash memory of mcu    
    if(butn1_cnt != 0){
        butn1_cnt--;
        //GPIO_SetBits(GPIOE, GPIO_Pin_3);
        led_blink = 500;
        
        //save_voltage_values(temp_voltage_value);
        //st7735_fill_screen(BLUE);
        //ST7735_DrawString7x11(60,  60 , "Saved", GREEN, BLUE);
        lcd_delay = 3000;
        //while(lcd_delay != 0);
        //st7735_fill_screen(BLUE);

        
    }
    
    
    
    
    
    if(average_delay == 0){
      if(av_count < 20)av_count++;
      
      for(uint8_t adc_ch = 0; adc_ch < 16; adc_ch++){
        
        av_buf[adc_ch][head] = adc_buf[adc_ch];
        
        
        
        for(uint8_t av_i = 0; av_i < av_count;av_i++){
          
          adc_av_buf[adc_ch] += av_buf[adc_ch][av_i];
          
        }
        adc_av_buf[adc_ch] = adc_av_buf[adc_ch] / (av_count+1);
        
        
        
      }//for ch160

      head++;
      if(head >= 20) head = 0;
      average_delay = 3;
    }    
      
      if(delay_c == 0){
          
        voltage_source_data[2] = adc_av_buf[14];
        voltage_source_data[3] = adc_av_buf[15];
        voltage_source_data[0] = adc_av_buf[12];
        voltage_source_data[1] = adc_av_buf[13];
        
        voltage_source_data[6] = adc_av_buf[10];
        voltage_source_data[7] = adc_av_buf[11];
        voltage_source_data[4] = adc_av_buf[8];
        voltage_source_data[5] = adc_av_buf[9];
        
        voltage_source_data[10] = adc_av_buf[6];
        voltage_source_data[11] = adc_av_buf[7];
        voltage_source_data[8] = adc_av_buf[4];
        voltage_source_data[9] = adc_av_buf[5];
        
        voltage_source_data[14] = adc_av_buf[2];
        voltage_source_data[15] = adc_av_buf[3];
        voltage_source_data[12] = adc_av_buf[0];
        voltage_source_data[13] = adc_av_buf[1];        
        
        //disp_8ch_formated(voltage_source_data);
        
        adc_buf[0] = adc_data_via_spi(0, 0);
        adc_buf[0] = adc_data_via_spi(0, 0);
        adc_buf[1] = adc_data_via_spi(0, 1);
        adc_buf[1] = adc_data_via_spi(0, 1);
        adc_buf[2] = adc_data_via_spi(0, 2);
        adc_buf[2] = adc_data_via_spi(0, 2);
        adc_buf[3] = adc_data_via_spi(0, 3);
        adc_buf[3] = adc_data_via_spi(0, 3);
        adc_data_via_spi(1, 0);
        adc_buf[4] = adc_data_via_spi(1, 0);
        adc_data_via_spi(1, 1);
        adc_buf[5] = adc_data_via_spi(1, 1);
        adc_data_via_spi(1, 2);
        adc_buf[6] = adc_data_via_spi(1, 2);
        adc_data_via_spi(1, 3);
        adc_buf[7] = adc_data_via_spi(1, 3);
        delay_c = 500;
      } 
    
      if(spi_queue != 0){
            spi_queue--;
            //spi_via_ring_buf(spi_cs_buf[spi_tail], spi_data_buf[spi_tail]);
            //spi_custom(spi_cs_buf[spi_tail], spi_data_buf[spi_tail]);
            spi_tail++;
            if(spi_tail >= 50) spi_tail = 0;             
      }
  
      
  }//бесконечный цикл
}



void system_init(void)
{
  SystemInit(); 
  //настройка портов
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  //******************************************************************************

 
  //++  
  //LEDS
  //++
  GPIO.GPIO_Pin  = GPIO_Pin_7;
  GPIO.GPIO_Mode = GPIO_Mode_OUT;
  GPIO.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO.GPIO_OType = GPIO_OType_PP;
  GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOE, &GPIO);
  GPIO.GPIO_Pin  = GPIO_Pin_2;
  GPIO.GPIO_Mode = GPIO_Mode_OUT;
  GPIO.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO.GPIO_OType = GPIO_OType_PP;
  GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO);  
  GPIO.GPIO_Pin  = GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_Init(GPIOD, &GPIO);
  //++
  //Buttons
  //++
  GPIO.GPIO_Mode = GPIO_Mode_IN;
  GPIO.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO.GPIO_OType = GPIO_OType_PP;
  GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOE, &GPIO);   
  //++
  //ENCODERS
  //++
  GPIO.GPIO_Mode = GPIO_Mode_IN;
  GPIO.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO.GPIO_OType = GPIO_OType_PP;
  GPIO.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOE, &GPIO);   
  GPIO.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_Init(GPIOB, &GPIO);  
  GPIO.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO);   
  GPIO.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init(GPIOD, &GPIO);
  GPIO.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO);

  //CHANNEL 1 (4x OUTPUT)
  //CS1 CLK MOSI LDAC SYNC
  GPIO.GPIO_Mode = GPIO_Mode_OUT;
  GPIO.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO.GPIO_OType = GPIO_OType_PP;
  GPIO.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOE, &GPIO);
  //RESET CS2
  GPIO.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO);
  //MISO
  GPIO.GPIO_Mode = GPIO_Mode_IN;
  GPIO.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO.GPIO_OType = GPIO_OType_PP;
  GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO.GPIO_Pin = GPIO_Pin_13;
  GPIO_Init(GPIOE, &GPIO);  
 
//Display ft812 based init 
  
  //SPI1 SCK MISO MOSI
  GPIO.GPIO_Mode = GPIO_Mode_AF;
  GPIO.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO.GPIO_OType = GPIO_OType_PP;
  GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOB, &GPIO);    

  //PD INT CS
  GPIO.GPIO_Mode = GPIO_Mode_OUT;
  GPIO.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO.GPIO_OType = GPIO_OType_PP;
  GPIO.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_Init(GPIOD, &GPIO);    
  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);//SCK
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);//MISO
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);//MOSI
  
  //настройка SPI
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI_Direction_1Line_Tx;//SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//high level at no transfer (High)
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //2Edge
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //42/128 = 0.328MHz  2.625 
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE); 
  SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
  
  //настройка таймера TIM7 
  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  //тактовый сигнал на таймер 7
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE); 
  //инициализация таймера 7
  TIM_TimeBaseStructure.TIM_Period = 49; //2*42MHz (if prescaler_APB1  != 1) 100kHz // Period - 1
  TIM_TimeBaseStructure.TIM_Prescaler = 83;//(uint16_t) (84000000 / 100000) - 1;; //100kHz  //by default 0 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
  //Clear TIM7 update pending flag 
  TIM_ClearFlag(TIM7, TIM_FLAG_Update);
  //Enable TIM7 Update interrupt 
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE); 
}

void TIM7_IRQHandler()
{
 if(lcd_delay != 0)lcd_delay--;
 LedCounter++;
 //if(send_spi == 0) send_spi = 1;
 LedCounter++;
 if(spi_delay !=0) spi_delay--;
 if(delay_c!=0)delay_c--;
 if(average_delay != 0)average_delay--;
 

 
 if(led_blink != 0) led_blink--;
 else GPIO_ResetBits(GPIOE, GPIO_Pin_7);
 
 butn_tim++;
 if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8) == 1) butn1_press++;
 if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) == 1) butn2_press++;
 if(butn_tim > 1000){
   butn_tim = 0;
   if(butn1_press > 700){
     butn1_state = 1;
     if((butn1_pstate == 0) && (butn1_state == 1)) butn1_cnt++;
   }
   else butn1_state = 0;
   butn1_press = 0;
   butn1_pstate = butn1_state;
   
   if(butn2_press > 700){
     butn2_state = 1;
     if((butn2_pstate == 0) && (butn2_state == 1)) butn2_cnt++;
   }
   else butn2_state = 0;
   butn2_press = 0;
   butn2_pstate = butn2_state;
 }
 
 
 
 

 encoder_timer++;
 if(encoder_timer == 1){
  encoder_timer = 0;
  encoder_1_value = GPIOA->IDR;
 
  if(((encoder_1_value & 0x03)) == 3) encoder_1_state = 0;
  if(((encoder_1_value & 0x03)) == 2) encoder_1_state = 1;
  if(((encoder_1_value & 0x03)) == 1) encoder_1_state = 2;
  if(((encoder_1_value & 0x03)) == 0) encoder_1_state = 3;
  
  if(encoder_1_pstate != encoder_1_state){
    if((encoder_1_state == 2) && (encoder_1_pstate == 3)){
      if(encoder_1_counter < 1023) encoder_1_counter++;
      if(temp_voltage_value[0] < MAX_COUNT_DRAIN) temp_voltage_value[0]++;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[0];//encoder_1_counter;
      spi_cs_buf[spi_head++] = 0;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_1_state == 1) && (encoder_1_pstate == 3)){
      if(encoder_1_counter != 0) encoder_1_counter--;
      if(temp_voltage_value[0] != 0) temp_voltage_value[0]--;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[0];//encoder_1_counter;
      spi_cs_buf[spi_head++] = 0;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    
  }//new state

  encoder_2_value = GPIOA->IDR;//((encoders_port>>10)&0x03);
  if(((encoder_2_value & 0x0C)>>2) == 3) encoder_2_state = 0;
  if(((encoder_2_value & 0x0C)>>2) == 2) encoder_2_state = 1;
  if(((encoder_2_value & 0x0C)>>2) == 1) encoder_2_state = 2;
  if(((encoder_2_value & 0x0C)>>2) == 0) encoder_2_state = 3;
    
  
  if(encoder_2_pstate != encoder_2_state){
    if((encoder_2_state == 2) && (encoder_2_pstate == 3)){
      if(encoder_2_counter <= 1023) encoder_2_counter++;
      if(temp_voltage_value[1] < MAX_COUNT_GATE) temp_voltage_value[1]++;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[1];//encoder_2_counter;
      spi_cs_buf[spi_head++] = 1;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_2_state == 1) && (encoder_2_pstate == 3)){
      if(encoder_2_counter > 6) encoder_2_counter--;
      if(temp_voltage_value[1] != 0) temp_voltage_value[1]--;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[1];//encoder_2_counter;
      spi_cs_buf[spi_head++] = 1;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;      
    }
    
  }//new state


  encoder_3_value = GPIOA->IDR;//((encoders_port>>10)&0x03);
  if(((encoder_3_value & 0x30)>>4) == 3) encoder_3_state = 0;
  if(((encoder_3_value & 0x30)>>4) == 2) encoder_3_state = 1;
  if(((encoder_3_value & 0x30)>>4) == 1) encoder_3_state = 2;
  if(((encoder_3_value & 0x30)>>4) == 0) encoder_3_state = 3;
    
  
  if(encoder_3_pstate != encoder_3_state){
    if((encoder_3_state == 2) && (encoder_3_pstate == 3)){
      if(encoder_3_counter < 1023) encoder_3_counter++;
      if(temp_voltage_value[2] < MAX_COUNT_DRAIN) temp_voltage_value[2]++;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[2];//encoder_3_counter;
      spi_cs_buf[spi_head++] = 2;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_3_state == 1) && (encoder_3_pstate == 3)){
      if(encoder_3_counter != 0) encoder_3_counter--;
      if(temp_voltage_value[2] != 0) temp_voltage_value[2]--;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[2];//encoder_3_counter;
      spi_cs_buf[spi_head++] = 2;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;      
    }
    
  }//new state
  
  encoder_4_value = GPIOA->IDR;//((encoders_port>>10)&0x03);
  if(((encoder_4_value & 0xC0)>>6) == 3) encoder_4_state = 0;
  if(((encoder_4_value & 0xC0)>>6) == 2) encoder_4_state = 1;
  if(((encoder_4_value & 0xC0)>>6) == 1) encoder_4_state = 2;
  if(((encoder_4_value & 0xC0)>>6) == 0) encoder_4_state = 3;
    
  if(encoder_4_pstate != encoder_4_state){
    if((encoder_4_state == 2) && (encoder_4_pstate == 3)){
      if(encoder_4_counter < 1023) encoder_4_counter++;
      if(temp_voltage_value[3] < MAX_COUNT_GATE) temp_voltage_value[3]++;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[3];//encoder_4_counter;
      spi_cs_buf[spi_head++] = 3;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_4_state == 1) && (encoder_4_pstate == 3)){
      if(encoder_4_counter > 6) encoder_4_counter--;
      if(temp_voltage_value[3] != 0) temp_voltage_value[3]--;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[3];//encoder_4_counter;
      spi_cs_buf[spi_head++] = 3;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;      
    }
    
  }//new state  
  
  encoder_5_value = GPIOC->IDR;//((encoders_port>>10)&0x03);
  if(((encoder_5_value & 0x30)>>4) == 3) encoder_5_state = 0;
  if(((encoder_5_value & 0x30)>>4) == 2) encoder_5_state = 1;
  if(((encoder_5_value & 0x30)>>4) == 1) encoder_5_state = 2;
  if(((encoder_5_value & 0x30)>>4) == 0) encoder_5_state = 3;
    
  if(encoder_5_pstate != encoder_5_state){
    if((encoder_5_state == 2) && (encoder_5_pstate == 3)){
      if(encoder_5_counter < 1023) encoder_5_counter++;
      if(temp_voltage_value[4] < MAX_COUNT_DRAIN) temp_voltage_value[4]++;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[4];//encoder_5_counter;
      spi_cs_buf[spi_head++] = 4;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_5_state == 1) && (encoder_5_pstate == 3)){
      if(encoder_5_counter != 0) encoder_5_counter--;
      if(temp_voltage_value[4] != 0) temp_voltage_value[4]--;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[4];//encoder_5_counter;
      spi_cs_buf[spi_head++] = 4;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;      
    }
    
  }//new state   

  encoder_6_value = GPIOE->IDR;//((encoders_port>>10)&0x03);
  if(((encoder_6_value & 0x30)>>4) == 3) encoder_6_state = 0;
  if(((encoder_6_value & 0x30)>>4) == 2) encoder_6_state = 1;
  if(((encoder_6_value & 0x30)>>4) == 1) encoder_6_state = 2;
  if(((encoder_6_value & 0x30)>>4) == 0) encoder_6_state = 3;
  
  if(encoder_6_pstate != encoder_6_state){
    if((encoder_6_state == 2) && (encoder_6_pstate == 3)){
      if(encoder_6_counter <= 1023) encoder_6_counter++;
      if(temp_voltage_value[5] < MAX_COUNT_GATE) temp_voltage_value[5]++;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[5];//encoder_6_counter;
      spi_cs_buf[spi_head++] = 5;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_6_state == 1) && (encoder_6_pstate == 3)){
      if(encoder_6_counter > 6) encoder_6_counter--;
      if(temp_voltage_value[5] != 0) temp_voltage_value[5]--;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[5];//encoder_6_counter;
      spi_cs_buf[spi_head++] = 5;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;      
    }
    
  }//new state   

  encoder_7_value = GPIOE->IDR;//((encoders_port>>10)&0x03);
  if(((encoder_7_value & 0x0C)>>2) == 3) encoder_7_state = 0;
  if(((encoder_7_value & 0x0C)>>2) == 2) encoder_7_state = 1;
  if(((encoder_7_value & 0x0C)>>2) == 1) encoder_7_state = 2;
  if(((encoder_7_value & 0x0C)>>2) == 0) encoder_7_state = 3;
    
  if(encoder_7_pstate != encoder_7_state){
    if((encoder_7_state == 2) && (encoder_7_pstate == 3)){
      if(encoder_7_counter < 1023) encoder_7_counter++;
      if(temp_voltage_value[6] < MAX_COUNT_DRAIN) temp_voltage_value[6]++;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[6];//encoder_7_counter;
      spi_cs_buf[spi_head++] = 6;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_7_state == 1) && (encoder_7_pstate == 3)){
      if(encoder_7_counter != 0) encoder_7_counter--;
      if(temp_voltage_value[6] != 0) temp_voltage_value[6]--;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[6];//encoder_7_counter;
      spi_cs_buf[spi_head++] = 6;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;      
    }
    
  }//new state 

  encoder_8_value = GPIOE->IDR;//((encoders_port>>10)&0x03);
  if(((encoder_8_value & 0x03)) == 3) encoder_8_state = 0;
  if(((encoder_8_value & 0x03)) == 2) encoder_8_state = 1;
  if(((encoder_8_value & 0x03)) == 1) encoder_8_state = 2;
  if(((encoder_8_value & 0x03)) == 0) encoder_8_state = 3;
     
  if(encoder_8_pstate != encoder_8_state){
    if((encoder_8_state == 2) && (encoder_8_pstate == 3)){
      if(encoder_8_counter < 1023) encoder_8_counter++;
      if(temp_voltage_value[7] < MAX_COUNT_GATE) temp_voltage_value[7]++;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[7];//encoder_8_counter;
      spi_cs_buf[spi_head++] = 7;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_8_state == 1) && (encoder_8_pstate == 3)){
      if(encoder_8_counter > 6) encoder_8_counter--;
      if(temp_voltage_value[7] != 0) temp_voltage_value[7]--;
      //flag_spi_send_time = 1;
      spi_data_buf[spi_head] = temp_voltage_value[7];//encoder_8_counter;
      spi_cs_buf[spi_head++] = 7;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;      
    }
    
  }//new state 

  encoder_9_value = GPIOB->IDR;
  if(((encoder_9_value & 0x0300)>>8) == 3) encoder_9_state = 0;
  if(((encoder_9_value & 0x0300)>>8) == 2) encoder_9_state = 1;
  if(((encoder_9_value & 0x0300)>>8) == 1) encoder_9_state = 2;
  if(((encoder_9_value & 0x0300)>>8) == 0) encoder_9_state = 3;
    
  if(encoder_9_pstate != encoder_9_state){
    if((encoder_9_state == 2) && (encoder_9_pstate == 3)){
      if(encoder_9_counter < 1023) encoder_9_counter++;
      if(temp_voltage_value[8] < MAX_COUNT_DRAIN) temp_voltage_value[8]++;
      spi_data_buf[spi_head] = temp_voltage_value[8];//encoder_9_counter;
      spi_cs_buf[spi_head++] = 8;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_9_state == 1) && (encoder_9_pstate == 3)){
      if(encoder_9_counter != 0) encoder_9_counter--;
      if(temp_voltage_value[8] != 0) temp_voltage_value[8]--;
      spi_data_buf[spi_head] = temp_voltage_value[8];//encoder_9_counter;
      spi_cs_buf[spi_head++] = 8;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;      
    }
  }//new state 
  
  encoder_10_value = GPIOB->IDR;
  if(((encoder_10_value & 0x00C0)>>2) == 3) encoder_10_state = 0;
  if(((encoder_10_value & 0x00C0)>>2) == 2) encoder_10_state = 1;
  if(((encoder_10_value & 0x00C0)>>2) == 1) encoder_10_state = 2;
  if(((encoder_10_value & 0x00C0)>>2) == 0) encoder_10_state = 3;
     
  if(encoder_10_pstate != encoder_10_state){
    if((encoder_10_state == 2) && (encoder_10_pstate == 3)){
      if(encoder_10_counter < 1023) encoder_10_counter++;
      if(temp_voltage_value[9] < MAX_COUNT_GATE) temp_voltage_value[9]++;
      spi_data_buf[spi_head] = temp_voltage_value[9];//encoder_8_counter;
      spi_cs_buf[spi_head++] = 9;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_10_state == 1) && (encoder_10_pstate == 3)){
      if(encoder_10_counter > 6) encoder_10_counter--;
      if(temp_voltage_value[9] != 0) temp_voltage_value[9]--;
      spi_data_buf[spi_head] = temp_voltage_value[9];//encoder_10_counter;
      spi_cs_buf[spi_head++] = 9;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;      
    }
    
  }//new state 
    
  encoder_11_value = GPIOB->IDR;
  if(((encoder_11_value & 0x03)) == 3) encoder_11_state = 0;
  if(((encoder_11_value & 0x03)) == 2) encoder_11_state = 1;
  if(((encoder_11_value & 0x03)) == 1) encoder_11_state = 2;
  if(((encoder_11_value & 0x03)) == 0) encoder_11_state = 3;
    
  if(encoder_11_pstate != encoder_11_state){
    if((encoder_11_state == 2) && (encoder_11_pstate == 3)){
      if(encoder_11_counter < 1023) encoder_11_counter++;
      if(temp_voltage_value[10] < MAX_COUNT_DRAIN) temp_voltage_value[10]++;
      spi_data_buf[spi_head] = temp_voltage_value[10];//encoder_10_counter;
      spi_cs_buf[spi_head++] = 10;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_11_state == 1) && (encoder_11_pstate == 3)){
      if(encoder_11_counter != 0) encoder_11_counter--;
      if(temp_voltage_value[10] != 0) temp_voltage_value[10]--;
      spi_data_buf[spi_head] = temp_voltage_value[10];//encoder_10_counter;
      spi_cs_buf[spi_head++] = 10;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;      
    }
  }//new state 
  
  encoder_12_value = GPIOD->IDR;
  if(((encoder_12_value & 0x03)) == 3) encoder_12_state = 0;
  if(((encoder_12_value & 0x03)) == 2) encoder_12_state = 1;
  if(((encoder_12_value & 0x03)) == 1) encoder_12_state = 2;
  if(((encoder_12_value & 0x03)) == 0) encoder_12_state = 3;
     
  if(encoder_12_pstate != encoder_12_state){
    if((encoder_12_state == 2) && (encoder_12_pstate == 3)){
      if(encoder_12_counter < 1023) encoder_12_counter++;
      if(temp_voltage_value[11] < MAX_COUNT_GATE) temp_voltage_value[11]++;
      spi_data_buf[spi_head] = temp_voltage_value[11];//encoder_12_counter;
      spi_cs_buf[spi_head++] = 11;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;
    }
    if((encoder_12_state == 1) && (encoder_12_pstate == 3)){
      if(encoder_12_counter > 6) encoder_12_counter--;
      if(temp_voltage_value[11] != 0) temp_voltage_value[11]--;
      spi_data_buf[spi_head] = temp_voltage_value[11];//encoder_12_counter;
      spi_cs_buf[spi_head++] = 11;
      spi_queue++;
      if(spi_head >= 50) spi_head = 0;      
    }
    
  }//new state   
  
  encoder_1_pstate = encoder_1_state;
  encoder_2_pstate = encoder_2_state;
  encoder_3_pstate = encoder_3_state;
  encoder_4_pstate = encoder_4_state;
  encoder_5_pstate = encoder_5_state;
  encoder_6_pstate = encoder_6_state;
  encoder_7_pstate = encoder_7_state;
  encoder_8_pstate = encoder_8_state;
  encoder_9_pstate = encoder_9_state;
  encoder_10_pstate = encoder_10_state;
  encoder_11_pstate = encoder_11_state;
  encoder_12_pstate = encoder_12_state;
 }
 
 //сброс флага прерывания по переполнению
 TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
 //сброс флага переполения таймера
 TIM_ClearFlag(TIM7, TIM_FLAG_Update);
}











   /*
    if(send_spi == 2){
      SPI2->DR = START_ACQUISITION;
      SPI_Cmd(SPI2, ENABLE);
      SPI_BiDirectionalLineConfig(SPI2, SPI_Direction_Tx);
      for(i=0;i<100;i++);
      //SpiCmd = SPI2->DR;
             
        SPI_BiDirectionalLineConfig(SPI2, SPI_Direction_Rx);
        for(i=0;i<16;i++){
        while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
        SPI_RX_Buf[i] = SPI2->DR;
        }
        
        SPI_Cmd(SPI2, DISABLE);
        
     for(i=0;i<5000000;i++);
    
     for(i=0;i<16;i++)
     {
     uart_buf[2*i] = (uint8_t) ((SPI_RX_Buf[i]>>8)&0x00ff);
     uart_buf[(2*i)+1] = (uint8_t) ((SPI_RX_Buf[i]&0x00ff)); 
     }
     usart_send_flag = 0;
     USART_SendData(USART3, uart_buf[0]);
    
    }
    
    */

