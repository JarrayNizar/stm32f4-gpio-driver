/**
  ******************************************************************************
  * @file    Lab-Libraries/gpio.h 
  * @author  CSF Team
  * @mail    formation@csf.tn
  * @Tel     (+216)92.039.433
  * @version V1.0.0
  * @date    23-04-2024
  *****************************************************************************

/* Includes ------------------------------------------------------------------*/
/*adreesse de base GPIO_X */



#define GPIO_A (unsigned int *)  (0x40020000)
#define GPIO_B (unsigned int *)  (0x40020400)
#define GPIO_C (unsigned int *)  (0x40020800)
#define GPIO_D (unsigned int *)  (0x40020C00)
#define GPIO_E (unsigned int *)  (0x40021000)
#define GPIO_F (unsigned int *)  (0x40021400)
#define GPIO_G (unsigned int *)  (0x40021800) 
#define GPIO_H (unsigned int *)  (0x40021c00)
#define GPIO_I (unsigned int *)  (0x40022000)

/*#ifdef (stm32f4i)
# define GPIO_J (unsigned int *) (0x40022400);
# define GPIO_K (unsigned int *) (0x40022800);
#endif
/*adreesse de base RCC */
#define RCC (unsigned int*) (0x40023800)
/*Les offset des differents registres RCC*/
# define AHB1RSTR 0x10  //Address offset RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
# define AHB1ENR 0x30   //Address offset RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)

/*Les offset des differents registres GPIO*/
# define MODER 0x00  //Address offset GPIO port mode register (GPIOx_MODER)  
# define OTYPER 0x01  //Address offset GPIO port output type register (GPIOx_OTYPER) 0x01 ne pas 0c04 car type  
# define OSPEEDR 0x02  //Address offset GPIO port output speed register (GPIOx_OSPEEDR)
# define PUPDR 0x03  //Address offset GPIO port pull-up/pull-down register (GPIOx_PUPDR)
# define IDR 0x04  //Address offset GPIO port input data register (GPIOx_IDR) (x = A..I/J/K)
# define ODR 0x05 //Address offset GPIO port output data register (GPIOx_ODR) (x = A..I/J/K)
# define BSRR 0x06  //Address offset GPIO port bit set/reset register (GPIOx_BSRR) (x = A..I/J/K)
# define LCKR 0x07  //Address offset GPIO port configuration lock register (GPIOx_LCKR)
# define AFRL 0x08  //Address offset GPIO alternate function low register (GPIOx_AFRL) (x = A..I/J/K)
# define AFRH 0x09  //Address offsetGPIO alternate function high register (GPIOx_AFRH) 

/*Les differents mode de fonctionement*/
// MODE GPIO port mode register (GPIOx_MODER) (x = A..I/J/K)
# define INPUT  0x00//Input (reset state)
# define OUTPUT 0x01//General purpose output mode
# define AF     0x02//Alternate function mode
# define AN     0x03//Analog mode

//MODE GPIO port pull-up/pull-down register (GPIOx_PUPDR)
# define NP 0x00//No pull-up, pull-down
# define PU 0x01//Pull-up
# define PD 0x02//Pull-down
# define RE 0x03//Reserved

//MODE GPIO port output type register (GPIOx_OTYPER)
# define PP 0x0//Output push-pull (reset state)
# define OD 0x1//Output open-drain

//GPIO port output speed register (GPIOx_OSPEEDR)
# define MODE_LS  0x00//Low speed
# define MODE_MS  0x01//Medium speed
# define MODE_HS  0x02//High speed
# define MODE_VHS 0x03//Very high speed

/*Positions des pin dans les registre de lecture et ecriture*/
# define GPIO_PIN_0  ((unsigned short)  0x0001)
# define GPIO_PIN_1  ((unsigned short)  0x0002)
# define GPIO_PIN_2  ((unsigned short)  0x0004)
# define GPIO_PIN_3  ((unsigned short)  0x0008)
# define GPIO_PIN_4  ((unsigned short)  0x0010)
# define GPIO_PIN_5  ((unsigned short)  0x0020)
# define GPIO_PIN_6  ((unsigned short)  0x0040)
# define GPIO_PIN_7  ((unsigned short)  0x0080)
# define GPIO_PIN_8  ((unsigned short)  0x0100)
# define GPIO_PIN_9  ((unsigned short)  0x0200)
# define GPIO_PIN_10 ((unsigned short)  0x0400)
# define GPIO_PIN_11 ((unsigned short)  0x0800)
# define GPIO_PIN_12 ((unsigned short)  0x1000)
# define GPIO_PIN_13 ((unsigned short)  0x2000)
# define GPIO_PIN_14 ((unsigned short)  0x4000)
# define GPIO_PIN_15 ((unsigned short)  0x8000)




/* -----------------ajouter les protopyes des fonctions----------------------*/


/* GPIO enable function */
/**
* @brief Enable clock for the gpio_x peripheral.
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @retval None
*/
void GPIO_ClockEnable (unsigned int * gpio_x);



/* GPIO Resete function */
/**
* @brief Deinitializes the gpio_x peripheral registers to their default reset values.
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @retval None
*/
void GPIO_DeInit(unsigned int * gpio_x);


/* Config function */
/**
* @brief Configure the gpio_x
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @param Mode: can be INPUT, OUTPUT, AF or AN
* @param typeOutput: can be PP or OD
* @param pin: can be 0...15
* @retval None
*/
void GPIO_Init(unsigned int * gpio_x, char Mode, char typeOutput, short int pin);

/* Read functions */
/**
* @brief Reads the specified input port pin.
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @param GPIO_Pin: specifies the port bit to read.
* This parameter can be GPIO_Pin_x where x can be (0..15).
* @retval The input port pin value.
*/
unsigned char GPIO_ReadInputDataBit(unsigned int * gpio_x, unsigned short int GPIO_Pin);

/* Write functions */
/**
* @brief Reads the specified GPIO input data port.
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @retval GPIO input data port value.
*/
unsigned short int GPIO_ReadInputData(unsigned int * gpio_x);


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
void GPIO_WriteBit(unsigned int * gpio_x, unsigned short int GPIO_Pin, char BitVal);


//Fonction_7: Mettre une valeur désiré dans un Port complet
/**
* @brief Writes data to the specified GPIO data port.
* @param gpio_x: where x can be (A..G) to select the GPIO peripheral.
* @param PortVal: specifies the value to be written to the port output data register.
* @retval None
*/
void GPIO_Write(unsigned int * gpio_x, unsigned short int PortVal);



/******************* (C) COPYRIGHT 2019 CSF *****END OF FILE*******************/