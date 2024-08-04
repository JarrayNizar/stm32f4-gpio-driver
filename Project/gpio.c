/**
  ******************************************************************************
  * @file    Lab-Libraries/gpio.c
  * @author  CSF Team
  * @mail    formation@csf.tn
  * @Tel     (+216)92.039.433
  * @version V1.0.0
  * @date    28-04-2024
  *****************************************************************************
/* Includes ------------------------------------------------------------------*/

#include "gpio.h"


// Ajouter define pour RCC clock 
volatile unsigned short int *RCC_AHB1ENB = (unsigned short int *) 0x40023830; 
// Ajouter define pour RCC  reste 
volatile unsigned short int *RCC_AHB1RSTR = (unsigned short int *) 0x40023810;


/**
  * @brief  Enable the gpio_x peripheral clock.
  * @param  gpio_x: where x can be (A..G) to select the GPIO peripheral.
  * @retval None
  */
void GPIO_ClockEnable (unsigned int * gpio_x){
 
  // activer le clock du GPIOA  
 if(gpio_x == GPIO_A)
  {
   
    *RCC_AHB1ENB |=(1<<0);
  }
  // activer le clock du GPIOB
  else if (gpio_x == GPIO_B)
  {
    
    *RCC_AHB1ENB|=(1<<1);
  }
  // activer le clock du GPIOC
  else if (gpio_x == GPIO_C)
  {
    
    *RCC_AHB1ENB|=(1<<2);
  }
  // activer le clock du GPIOD
  else if (gpio_x == GPIO_D)
  {
    
    *RCC_AHB1ENB|=(1<<3);
  }

   // activer le clock du GPIOE
  else if (gpio_x == GPIO_E)
  {
    
    *RCC_AHB1ENB|=(1<<4);
  }
   // activer le clock du GPIOF
  else if (gpio_x == GPIO_F)
  {
    
    *RCC_AHB1ENB|=(1<<5);
  }
   // activer le clock du GPIOG
  else if (gpio_x == GPIO_G)
  {
    
    *RCC_AHB1ENB|=(1<<6);
  }
 
   // activer le clock du GPIOH
  else if (gpio_x == GPIO_H)
  {
    
    *RCC_AHB1ENB|=(1<<7);
  }
   // activer le clock du GPIOI
  else if (gpio_x == GPIO_I)
  {
    
    *RCC_AHB1ENB|=(1<<8);
  }

}


/* GPIO Resete function */
/**
* @brief Deinitializes the gpio_x peripheral registers to their default reset values.
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @retval None
*/
  void GPIO_DeInit(unsigned int * gpio_x)
  {

  
  // Resete le clock du GPIOA  
 if(gpio_x == GPIO_A)
  {
   
    *RCC_AHB1RSTR |=(1<<0);
    *RCC_AHB1RSTR &= ~(1<<0);
  }
  // Resete le clock du GPIOB
  else if (gpio_x == GPIO_B)
  {
    
    *RCC_AHB1RSTR|=(1<<1);
    *RCC_AHB1RSTR &= ~(1<<1);
  }
  // Resete le clock du GPIOC
  else if (gpio_x == GPIO_C)
  {
    
    *RCC_AHB1RSTR|=(1<<2);
    *RCC_AHB1RSTR &= ~(1<<2);
  }
  // Resete le clock du GPIOD
  else if (gpio_x == GPIO_D)
  {
    
    *RCC_AHB1RSTR|=(1<<3);
    *RCC_AHB1RSTR &= ~(1<<3);
  }

   // Resete le clock du GPIOE
  else if (gpio_x == GPIO_E)
  {
    
    *RCC_AHB1RSTR|=(1<<4);
    *RCC_AHB1RSTR &= ~(1<<4);
  }
   // Resete le clock du GPIOF
  else if (gpio_x == GPIO_F)
  {
    
    *RCC_AHB1RSTR|=(1<<5);
    *RCC_AHB1RSTR &= ~(1<<5);
  }
   // Resete le clock du GPIOG
  else if (gpio_x == GPIO_G)
  {
    
    *RCC_AHB1RSTR|=(1<<6);
    *RCC_AHB1RSTR &= ~(1<<6);
  }
 
   // Resete le clock du GPIOH
  else if (gpio_x == GPIO_H)
  {
    
    *RCC_AHB1RSTR|=(1<<7);
    *RCC_AHB1RSTR &= ~(1<<7);
  }
   // Resete le clock du GPIOI
  else if (gpio_x == GPIO_I)
  {
    
    *RCC_AHB1RSTR|=(1<<8);
    *RCC_AHB1RSTR &= ~(1<<8);
  }

}

/* Config function */
/**
* @brief Configure the gpio_x
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @param Mode: can be INPUT, OUTPUT, AF or AN
* @param typeOutput: can be PP or OD
* @param pin: can be 0...15
* @retval None
*/
void GPIO_Init(unsigned int * gpio_x, char Mode, char typeOutput, short int pin)
{ 
      
  unsigned int mask = ~(0x3<<(pin*2));
  *(gpio_x + MODER) &= mask;
  *(gpio_x + MODER) |= Mode << (pin*2);

   mask = ~(0x01<<(pin));
   //Output push-pull (reset state)
  if (typeOutput == PP)
    *(gpio_x + OTYPER) &= mask;
  //Output open-drain
  else
  *(gpio_x + OTYPER) |= ~mask;

  
  }
  


/* Read functions */
/**
* @brief Reads the specified input port pin.
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @param GPIO_Pin: specifies the port bit to read.
* This parameter can be GPIO_Pin_x where x can be (0..15).
* @retval The input port pin value.
*/
unsigned char GPIO_ReadInputDataBit(unsigned int * gpio_x, unsigned short int GPIO_Pin)
{
  unsigned  char bitstatus = 0x00;
  
  if (((*(gpio_x + IDR)) & GPIO_Pin) != 0x00)
  {
    bitstatus = 0x01;
  }
  else
  {
    bitstatus = 0x00;
  }
  return bitstatus;
}

/* Write functions */
/**
* @brief Reads the specified GPIO input data port.
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @retval GPIO input data port value.
*/
unsigned short int GPIO_ReadInputData(unsigned int * gpio_x)
{
  return (*(gpio_x + IDR));
}


//Fonction_6: Mettre la valeur 1 ou 0 dans une seul PIN
/**
* @brief Sets or clears the selected data port bit.
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @param GPIO_Pin: specifies the port bit to be written.
* This parameter can be one of GPIO_Pin_x where x can be (0..15).
* @param BitVal: specifies the value to be written to the selected bit.
* This parameter can be one of the BitAction enum values:
* @arg Bit_RESET: to clear the port pin
* @arg Bit_SET: to set the port pin
* @retval None
*/
void GPIO_WriteBit(unsigned int * gpio_x, unsigned short int GPIO_Pin, char BitVal)
{
  if (BitVal !=0x00)
  {
    *(gpio_x + ODR) |= GPIO_Pin;
  }
  else
  {
    *(gpio_x + ODR) &= (~GPIO_Pin);
  }
}

//Fonction_7: Mettre une valeur désiré dans un Port complet
/**
* @brief Writes data to the specified GPIO data port.
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @param PortVal: specifies the value to be written to the port output data register.
* @retval None
*/
void GPIO_Write(unsigned int * gpio_x, unsigned short int PortVal)
{
  *(gpio_x + ODR) = PortVal;
}



/******************* (C) COPYRIGHT 2024 CSF *****END OF FILE*******************/

