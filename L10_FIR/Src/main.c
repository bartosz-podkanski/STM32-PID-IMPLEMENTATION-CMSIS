/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/*TODO*/
/*
	*Code refactoring
	*make independent encoder from while loop == make dependent encoder from timer
	*make independent LCD from while loop == make dependent LCD from timer
*/
/*AUTHORS*/
/*
	Bartosz Podkanski && Patryk Pilarski
	February 2020 
	PID implementation on STM32 using CMSIS library 
*/ 

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "math_helper.h"
#include "bmp280.h"
#include <stdio.h>
#include "i2c-lcd.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_BUFFER_LEN 28
#define BMP280_ADDRESS_INDEX  2
#define BMP280_DATA_INDEX 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

	// UART for communication with terminal
		uint8_t temp_size;
		char buffer[100];
		char msg[3];
		float set_temp_interrupt;
	
	// temperatures values
		int32_t temp32;	// bmp
	    double temp;	
	    float set_temp=36;
	    float actual_temp=0;
		
	// PWM duties
		float duty;	// duty resistor PWM
	  	float duty_fan;	// duty fan PWM

	// PID global values
	    float set_kp = 100.0f;
	    float set_ki = 0.1f;
	    float set_kd = 0.0f;
		float error=0;	// offset
	    float pControl=0;	// pControl depends on the error
	    float prev_pControl=0;

	// encoder and LCD variables 
		uint8_t stan_obecny_ENC=0;
		uint8_t stan_poprzedni_ENC=0;
		int zlicz_przycisk=0;	// click counting
		int zlicz_poprzednie=0;	// previous state counting 
		int button_pressed=0;
	    int menu_activated=0;
	    char zdanie2 [7];	// table for sending temp to LCD
		
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/*function BMP280 sensor*/
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	HAL_StatusTypeDef status = HAL_OK ;
	int32_t iError = BMP280_OK ;
	uint8_t txarray [ SPI_BUFFER_LEN * BMP280_ADDRESS_INDEX ];
	uint8_t stringpos ;

	txarray [0] = reg_addr ;
	 for ( stringpos = 0; stringpos < length ; stringpos ++) {
	txarray [ stringpos + BMP280_DATA_INDEX ] = reg_data [ stringpos ];
	}

	HAL_GPIO_WritePin (SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_RESET );
	status = HAL_SPI_Transmit ( &hspi4 , ( uint8_t *)(& txarray ), length *2, 100);
	while ( hspi4 . State == HAL_SPI_STATE_BUSY ) {};
	HAL_GPIO_WritePin ( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_SET );

	if ( status != HAL_OK )
	{
	// The BMP280 API calls for 0 return value as a success ,
	// and -1 returned as failure
	iError = ( -1);
	 }
	return ( int8_t ) iError ;
}
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	HAL_StatusTypeDef status = HAL_OK ;
	 int32_t iError = BMP280_OK ;
	 uint8_t txarray [ SPI_BUFFER_LEN ] = {0 ,};
	uint8_t rxarray [ SPI_BUFFER_LEN ] = {0 ,};
	 uint8_t stringpos ;

	 txarray [0] = reg_addr ;

	HAL_GPIO_WritePin ( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_RESET );
	status = HAL_SPI_TransmitReceive ( &hspi4 , ( uint8_t *)(& txarray ),
	( uint8_t *)(& rxarray ), length +1, 5);
	while ( hspi4 . State == HAL_SPI_STATE_BUSY ) {};
	HAL_GPIO_WritePin ( SPI4_CS_GPIO_Port , SPI4_CS_Pin , GPIO_PIN_SET );

	for ( stringpos = 0; stringpos < length ; stringpos ++) {
	 *( reg_data + stringpos ) = rxarray [ stringpos + BMP280_DATA_INDEX ];
	 }

	 if ( status != HAL_OK )
	 {
	 // The BMP280 API calls for 0 return value as a success ,
	 // and -1 returned as failure
	 iError = ( -1);
	 }
	 return ( int8_t ) iError ;
}
/*function BMP280 sensor END*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//BMP280 sensor variables and structs
	int8_t rslt;
	struct bmp280_dev bmp;
	struct bmp280_config conf;
	struct bmp280_uncomp_data ucomp_data;
	int32_t temp_swv;
	uint8_t sp_data[200];
	uint16_t sp_size;

  /* USER CODE END 1 */
  
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_SPI4_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  lcd_init();
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

	/*BMP280 CONFIGURATION*/
	
	/* Map the delay function pointer with the function responsible for implementing the delay */
		bmp.delay_ms = HAL_Delay;
    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
		bmp.dev_id = 0;
    /* Select the interface mode as I2C */
		bmp.intf = BMP280_SPI_INTF;
    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
		bmp.read = spi_reg_read;
		bmp.write = spi_reg_write;
		rslt = bmp280_init(&bmp);
		rslt = bmp280_get_config(&conf, &bmp);
    /* configuring the temperature oversampling, filter coefficient and output data rate */
    /* Overwrite the desired settings */
		conf.filter = BMP280_FILTER_COEFF_2;
    /* Temperature oversampling set at 4x */
		conf.os_temp = BMP280_OS_4X;
	/* Pressure over sampling none (disabling pressure measurement) */
		conf.os_pres = BMP280_OS_4X;
	/* Setting the output data rate as 1HZ(1000ms) */
		conf.odr = BMP280_ODR_1000_MS;
		rslt = bmp280_set_config(&conf, &bmp);
	/* Always set the power mode after setting the configuration */
		rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
	
	/*BMP280 CONFIGURATION END*/
	/*https://github.com/BoschSensortec/BMP280_driver*/

    /* PID INIT*/
		arm_pid_instance_f32 pid;
		arm_pid_init_f32(&pid, 1);

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/* Reading the raw data from BMP280 sensor */
		rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
	/* Getting the 32 bit compensated temperature */
		rslt = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp, &bmp);
	/* Getting the compensated temperature as floating point value */
		rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);

	/* calculate error/offset */
		actual_temp = temp;	// actual temp = temp from sensor
		error = (set_temp-actual_temp);	// offset

	/* value returned by controller */
		pControl=arm_pid_f32(&pid, error);	// pControl depends on the error

	/* sending variable temp from the sensor to the terminal */
		temp_size=sprintf(&buffer, "TEMP: %f \n\r",temp);	// calculate size of the message
		HAL_UART_Transmit(&huart3, (uint8_t*)buffer, temp_size, 0xffff);	// sending temp to the terminal 
		HAL_UART_Receive_IT(&huart3, &msg, 2);	// listening terminal

	/* duty limit ( __HAL_TIM_SET_COMPARE() takes PWM duty from 0 to 1000) */
		if(pControl<0){
			pControl=0;
		}
		if(pControl >1000){
			pControl=1000;
		}
		
		duty=pControl; // duty of PWM signal for resistor

	/* fan on–off controller */
		if(error<0){
			duty_fan=1000;
		}
		else
			duty_fan=0;

	/* setting PWM */
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,duty_fan);	// PWM fan
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,duty);	// PWM resistor


	/********************LCD*****************/
	/* https://controllerstech.com/i2c-lcd-in-stm32 */

	/* encoder button filter ****see section TODO**** */
		if(zlicz_przycisk>zlicz_poprzednie){
			menu_activated++;	// every click increase menu_activated == going to next page 
			if(menu_activated>=5){
				menu_activated=0;
			}
			HAL_Delay(20);
		}
		zlicz_poprzednie=zlicz_przycisk;	//remember last encoder click *****see section HAL_GPIO_EXTI_Callback() bellow*****
	

	/* MENU MAIN PAGE */
		if(menu_activated==0)
		{
			lcd_clear();
			lcd_put_cur(0, 2);
			lcd_send_string("PID control");

			lcd_put_cur(1, 0);
			sprintf(zdanie2,"S:%.1f", set_temp);
			lcd_send_string(zdanie2);

			lcd_put_cur(1, 8);
			sprintf(zdanie2,"R:%.2f",temp);
			lcd_send_string(zdanie2);

			// main page END
		}

	/* menu first page, set temperature */
		if(menu_activated==1)
		{
			lcd_clear();
			lcd_put_cur(0, 0);
			lcd_send_string("Set temperature:");

			sprintf(zdanie2,"S:%.1f", set_temp);
			lcd_put_cur(1, 6);
			lcd_send_string(zdanie2);

			/* choosing values with encoder */

				stan_obecny_ENC=htim1.Instance->CNT;	// present encoder position
				if(stan_poprzedni_ENC<stan_obecny_ENC)	// if previous state is less than present state 
				{
					set_temp-=1;
				}
				if(stan_poprzedni_ENC>stan_obecny_ENC)	// if previous state is more than present state 
				{
					set_temp+=1;
				}
				stan_poprzedni_ENC=stan_obecny_ENC; // remember last encoder state 
			
			// first page END
		}

	/* menu second page, set kp */

		if(menu_activated==2)
		{
			lcd_clear();
			lcd_put_cur(0, 0);
			lcd_send_string("Set Kp value:");

			sprintf(zdanie2, "Kp:%.1f", pid.Kp);
			lcd_put_cur(1, 6);
			lcd_send_string(zdanie2);

			/* choosing values with encoder */

				stan_obecny_ENC=htim1.Instance->CNT;
				if(stan_poprzedni_ENC<stan_obecny_ENC)
				{
					set_kp-=10;
				}
				if(stan_poprzedni_ENC>stan_obecny_ENC)
				{
					set_kp+=10;
				}
				stan_poprzedni_ENC=stan_obecny_ENC;
		
			// second page END
		}
	/* menu third page, set ki */
	  	if(menu_activated==3)
	  	{
	  		lcd_clear();
	  		lcd_put_cur(0, 0);
	  		lcd_send_string("Set Ki value:");

	  		sprintf(zdanie2, "Ki:%.1f", pid.Ki);
	  		lcd_put_cur(1, 6);
	  		lcd_send_string(zdanie2);

	  		/* choosing values with encoder */
	  			stan_obecny_ENC=htim1.Instance->CNT;
	  			if(stan_poprzedni_ENC<stan_obecny_ENC)
	  			{
	  				set_ki-=1;
	  			}
	  			if(stan_poprzedni_ENC>stan_obecny_ENC)
	  			{
	  				set_ki+=1;
	  			}
	  			stan_poprzedni_ENC=stan_obecny_ENC;
				
			// third page END
	  	}

	/* menu fourth page, set kd */
	  	if(menu_activated==4)
	  	{
	  		lcd_clear();
	  		lcd_put_cur(0, 0);
	  		lcd_send_string("Set Kd value:");

	  		sprintf(zdanie2, "Kd:%.1f", pid.Kd);
	  		lcd_put_cur(1, 6);
	  		lcd_send_string(zdanie2);

	  	/* choosing values with encoder */
				stan_obecny_ENC=htim1.Instance->CNT;
				if(stan_poprzedni_ENC<stan_obecny_ENC)
				{
					set_kd-=1;
				}
				if(stan_poprzedni_ENC>stan_obecny_ENC)
				{
					set_kd+=1;
				}
				stan_poprzedni_ENC=stan_obecny_ENC;
			// fourth page END
	  	}
	  	
	/* set choosen values to global */

	  	pid.Kp=set_kp;
	  	pid.Kd=set_kd;
	  	pid.Ki=set_ki;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_Delay(250); // Main loop delay
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *htim)
{
	/* conversion set temperature from terminal */
	char value1[2];

	value1[0]=msg[0];
	value1[1]=msg[1];
	set_temp_interrupt=atoi(value1);

	/* set temp limit; temperature can be inflicted from 20 to 40; else set temp from terminal */
	if(set_temp_interrupt>40)
	{
		HAL_UART_Transmit(&huart3, "ERROR \n\r", 8, 0xffff);
	}
	if(set_temp_interrupt<20)
	{
		HAL_UART_Transmit(&huart3, "ERROR \n\r", 8, 0xffff);
	}
	if((set_temp_interrupt>=20)&&set_temp_interrupt<=40)
	{
	set_temp=set_temp_interrupt;
	}
}

/* interrupt from gpio */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* encoder clicks */
	if(GPIO_Pin==guzik_Pin)
	{
		zlicz_przycisk++;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  while(1)
  {
	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	  HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
