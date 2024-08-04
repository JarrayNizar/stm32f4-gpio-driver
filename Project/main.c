/**
  ******************************************************************************
  * @file    Lab-Libraries/main.c 
  * @author  CSF Team
  * @mail    formation@csf.tn
  * @Tel     (+216)92.039.433
  * @version V1.0.0
  * @date    23-04-2024
  *****************************************************************************/

#include "gpio.h"

/* Private define ------------------------------------------------------------*/
#define NOMBRE_BOUCLES 100
                                                       
/* Private function prototypes -----------------------------------------------*/
void Delay(volatile unsigned  int  nCount);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  unsigned  int counter = 0x00;  

  /* Activer l'orloge pour le Port d */
   GPIO_ClockEnable(GPIO_D);
 

  /* re-intialiser les registers de port d */
    GPIO_DeInit(GPIO_D);

  
/* --- Configure PD12 et PD13 en mode Output Push pull */  

GPIO_Init(GPIO_D, OUTPUT,  PP, 12);
GPIO_Init(GPIO_D, OUTPUT,  PP, 13);

  for(counter=0; counter<NOMBRE_BOUCLES ; counter++)
  {
   /* Turn On LD3 and LD4 */
   GPIO_WriteBit(GPIO_D, GPIO_PIN_12, 0x01)  ;

   GPIO_WriteBit(GPIO_D, GPIO_PIN_13, 0x01)  ;
   

    Delay(0x3FFFFF) ;
    
    /* Turn off LD3 and LD4 */
  GPIO_WriteBit(GPIO_D, GPIO_PIN_12, 0x00)  ;
  
  GPIO_WriteBit(GPIO_D, GPIO_PIN_13, 0x00)  ;
    
   
    Delay(0x3FFFFF) ;
           
  }
  while(1);
}
/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
void Delay(volatile unsigned  int  nCount)
{
  for(; nCount != 0; nCount--);
}

/******************* (C) COPYRIGHT 22024 CSF *****END OF FILE*******************/
