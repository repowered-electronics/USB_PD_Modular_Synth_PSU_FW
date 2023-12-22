/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "printf.h"
#ifndef PRINTF
#define PRINTF
#endif
#include "USB_PD_core.h"
#include "ina236.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ENABLE_PRIMARY_12V() HAL_GPIO_WritePin(DISABLE_PRI_12V_GPIO_Port, DISABLE_PRI_12V_Pin, 0)
#define DISABLE_PRIMARY_12V() HAL_GPIO_WritePin(DISABLE_PRI_12V_GPIO_Port, DISABLE_PRI_12V_Pin, 1)
#define ENABLE_PDGOOD_LED() HAL_GPIO_WritePin(PDGOOD_GPIO_Port, PDGOOD_Pin, 1)
#define DISABLE_PDGOOD_LED() HAL_GPIO_WritePin(PDGOOD_GPIO_Port, PDGOOD_Pin, 0)
#define ENABLE_OVRLD_LED() HAL_GPIO_WritePin(PDBAD_GPIO_Port, PDBAD_Pin, 1)
#define DISABLE_OVRLD_LED() HAL_GPIO_WritePin(PDBAD_GPIO_Port, PDBAD_Pin, 0)
#define DISABLE_5PO_SUPPLY() HAL_GPIO_WritePin(DISABLE_5P0_GPIO_Port, DISABLE_5P0_Pin, 1)
#define ENABLE_5P0_SUPPLY() HAL_GPIO_WritePin(DISABLE_5P0_GPIO_Port, DISABLE_5P0_Pin, 0)
#define ENABLE_N12_SUPPLY() HAL_GPIO_WritePin(DISABLE_N12_GPIO_Port, DISABLE_N12_Pin, 0)
#define DISABLE_N12_SUPPLY() HAL_GPIO_WritePin(DISABLE_N12_GPIO_Port, DISABLE_N12_Pin, 1)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile INA236_t ina_5V;
volatile INA236_t ina_pos_12V;
volatile INA236_t ina_neg_12V;

I2C_HandleTypeDef *hi2c[I2CBUS_MAX];
unsigned int Address;
unsigned int AddressSize = I2C_MEMADD_SIZE_8BIT;
USB_PD_I2C_PORT STUSB45DeviceConf[USBPORT_MAX];
uint32_t timer_cnt = 0;
int Flag_count = 0;
int PB_press=0;
int Time_elapse=1;
int Plim = 0;             // Power limit from find max pdo
int Plim_guardband = 18560;   // MARGIN FOR TRIPPING OVERDRAW IN mW
float ILIM_5V = 2000;        // max rail current for 5V rail in mA
float ILIM_N12V = 4000;      // max rail current for -12V rail in mA
float ILIM_12V = 8000;      // max rail current for 12V *converter* in mA

uint8_t USB_PD_Interupt_Flag[USBPORT_MAX] ;
uint8_t USB_PD_Interupt_PostponedFlag[USBPORT_MAX] ; 
uint8_t push_button_Action_Flag[USBPORT_MAX];
uint8_t Timer_Action_Flag[USBPORT_MAX];
uint8_t connection_flag[USBPORT_MAX]={1};
uint8_t HR_rcv_flag[USBPORT_MAX] = {0};
uint32_t VBUS_Current_limitation[USBPORT_MAX] = {5},Previous_VBUS_Current_limitation[USBPORT_MAX] ; 

/* PDO Variables */
extern USB_PD_StatusTypeDef PD_status[USBPORT_MAX] ;
extern USB_PD_SNK_PDO_TypeDef PDO_SNK[USBPORT_MAX][3];
extern USB_PD_SRC_PDOTypeDef PDO_FROM_SRC[USBPORT_MAX][7];
extern uint8_t PDO_FROM_SRC_Valid[USBPORT_MAX];
extern uint8_t PDO_FROM_SRC_Num_Sel[USBPORT_MAX];
extern uint8_t PDO_FROM_SRC_Num[USBPORT_MAX];
extern uint8_t Policy_Engine_State[USBPORT_MAX];
extern uint8_t Go_disable_once[USBPORT_MAX];
extern uint8_t Final_Nego_done[USBPORT_MAX] ;
extern uint8_t Core_Process_suspended ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void _putchar(char character);
extern void nvm_flash(uint8_t Usb_Port);
int usb_pd_state_machine(uint8_t usb_port);

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
  int usb_port_id = 0;
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* ENSURE EXTERNAL SUPPLIES START IN OFF STATE */
  
  ENABLE_PRIMARY_12V();
  DISABLE_OVRLD_LED();
  ENABLE_PDGOOD_LED();  
  /* SETUP USB PD STUFF */
  
  hi2c[0] = &hi2c1;
  STUSB45DeviceConf[usb_port_id].I2cBus = usb_port_id;
  STUSB45DeviceConf[usb_port_id].I2cDeviceID_7bit = 0x28;
  STUSB45DeviceConf[usb_port_id].Alert_GPIO_Bank = USBPD_ALERT_GPIO_Port;
  STUSB45DeviceConf[usb_port_id].Alert_GPIO_Pin = USBPD_ALERT_Pin;
  AddressSize = I2C_MEMADD_SIZE_8BIT; 
  USB_PD_Interupt_PostponedFlag[0] = 0; /* this flag is 1 if I2C is busy when Alert signal raise */
  USB_PD_Interupt_Flag[usb_port_id] = 1;
  Final_Nego_done[usb_port_id] = 0;
  Go_disable_once[usb_port_id] = 0;

  memset((uint32_t *)PDO_FROM_SRC[usb_port_id], 0, 7);
  memset((uint32_t *)PDO_SNK[usb_port_id], 0, 3);
  
  usb_pd_init(usb_port_id);   // after this USBPD alert line must be high 

  HAL_Delay(500); // allow STUSB4500 to initialize

  Read_SNK_PDO(usb_port_id);

  

  Print_PDO_FROM_SRC(usb_port_id);

  Plim = Find_Max_SRC_PDO(usb_port_id);

  printf("the power limit is %d \r\n", Plim);

  HAL_Delay(500);

  DISABLE_PRIMARY_12V();
  for(int i=0; i<25; i++)
  {
    __NOP();
  }
  ENABLE_PRIMARY_12V();

  Print_PDO_FROM_SRC(usb_port_id);

  push_button_Action_Flag[usb_port_id] = 0;
  Read_RDO(usb_port_id);


  Print_RDO(usb_port_id);

  connection_flag[usb_port_id] = 1;
  Previous_VBUS_Current_limitation[usb_port_id] = VBUS_Current_limitation[usb_port_id];

  /**** BEGIN SETUP INA236's ****/
 
  ina_5V.hi2c = &hi2c1;

  ina236_general_call_reset(&ina_5V); // will reset all INA236's on the bus
  HAL_Delay(30);

  // Initialzing 5v ina236 struct
  ina_5V.addr = 0x41;
  ina_5V.shunt = 0.01;
  ina_5V.int_pin = ALERT_5V_Pin;
  ina236_init(&ina_5V);

  // Initialzing p12v ina236 struct
  ina_pos_12V.hi2c = &hi2c1;
  ina_pos_12V.addr = 0x40;
  ina_pos_12V.shunt = 0.01;
  ina_pos_12V.int_pin = ALERT_POS_12V_Pin;
  ina236_init(&ina_pos_12V);

  // Initialzing n12v ina236 struct
  ina_neg_12V.hi2c = &hi2c1;
  ina_neg_12V.addr = 0x43;
  ina_neg_12V.shunt = 0.01;
  ina_neg_12V.int_pin = ALERT_NEG_12V_Pin;
  ina236_init(&ina_neg_12V);

  // Setting full scale range
  ina236_set_shunt_range(&ina_pos_12V, 0);
  ina236_set_shunt_range(&ina_5V, 0);
  ina236_set_shunt_range(&ina_neg_12V, 0);

  // Setting shunt cal register for each rail
  ina236_set_shuntcal(&ina_pos_12V);
  ina236_set_shuntcal(&ina_5V);
  ina236_set_shuntcal(&ina_neg_12V);

  // Enabling SOL Alert
  ina236_set_alertSOL(&ina_pos_12V);
  ina236_set_alertSOL(&ina_5V);
  ina236_set_alertSOL(&ina_neg_12V);

  // Setting current limits
  ina236_set_current_limit(&ina_5V, ILIM_5V);                                 // Setting constant current limit for 5V rail based on capability of converter
  ina236_set_current_limit(&ina_neg_12V, ILIM_N12V);                          // Setting constant current limit for n12V rail based on capability of converter
  ina236_set_current_limit(&ina_pos_12V, ILIM_12V);                           // Setting initial current limit for p12V rail assuming no draw from derivative rails



  /**** END SETUP INA236's ****/

  ENABLE_PRIMARY_12V(); // starting primary 
  HAL_Delay(50);
  DISABLE_PRIMARY_12V();
  HAL_Delay(1);
  ENABLE_PRIMARY_12V();
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Reading bus Voltage from INA236
    float volts_p12v = ina236_get_voltage(&ina_pos_12V);
    float volts_5v = ina236_get_voltage(&ina_5V);
    float volts_n12v = ina236_get_voltage(&ina_neg_12V);

    // Reading current from INA236
    float amps_p12v = ina236_get_current(&ina_pos_12V);
    float amps_5v = 1000*ina236_get_current(&ina_5V);
    float amps_n12v = ina236_get_current(&ina_neg_12V);

    float draw_p12v = (amps_5v + amps_n12v);                                    // Calculating draw on p12v rail 
    float ILIM_12V_dyn = (ILIM_12V - draw_p12v);                                // Updating p12v current limit based on 5v and n12v rail draw

    ina236_set_current_limit(&ina_pos_12V, ILIM_12V_dyn);                       // Setting INA236 current limit with dynamic limit based on derivative rail draw

    // Reading power from INA236
    float power_p12V = 1000*ina236_get_power(&ina_pos_12V);
    float power_n12V = 1000*ina236_get_power(&ina_neg_12V);
    float power_5V = 1000*ina236_get_power(&ina_5V);

    int tot_power = (int)(power_p12V + power_n12V + power_5V);                  // Calculating total power


    // Hiccup control
    

    // Conditional Statement for Lighting overload LED based on total power draw
    if(tot_power >= (Plim - Plim_guardband))
    {
      DISABLE_PDGOOD_LED();
      ENABLE_OVRLD_LED();
    }
    
    else if (tot_power < (Plim - Plim_guardband))
    {
      DISABLE_OVRLD_LED();
      ENABLE_PDGOOD_LED();
    }

    HAL_Delay(1000);

    
    
    
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int usb_pd_state_machine(uint8_t usb_port){
  int status = Get_Device_STATUS(usb_port);

  switch(status){
    case Not_Connected:
      break;
    case TypeC_Only:
      break;
    case Connected_Unknown_SRC_PDOs:{
      Send_Soft_reset_Message(usb_port);
      Final_Nego_done[usb_port] = 0;
      break;
    }
    case Connected_5V_PD:
      break;
    case Connected_no_Match_found:
      break;
    case Connected_Matching_ongoing:{
      Send_Soft_reset_Message(usb_port);
      Final_Nego_done[usb_port] = 0;
      break;
    }
    case Connected_Mached:{
      break;
    }
    case Not_Connected_attached_wait:
      break;
    case Hard_Reset_ongoing:{
      break;
    }
    default:
      break;
  }

  if(VBUS_Current_limitation[usb_port] != Previous_VBUS_Current_limitation[usb_port]){
    Previous_VBUS_Current_limitation[usb_port] = VBUS_Current_limitation[usb_port];
  }
}

// Hiccup control
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  INA236_t* ina;
  if(GPIO_Pin == ina_5V.int_pin){
    ina = &ina_5V;
    DISABLE_5PO_SUPPLY();
    HAL_Delay(1000);
    ENABLE_5P0_SUPPLY();
    printf("5v rail is disabled");
  }else if(GPIO_Pin == ina_pos_12V.int_pin){
    ina = &ina_pos_12V;
    DISABLE_PRIMARY_12V();
    HAL_Delay(1000);
    ENABLE_PRIMARY_12V();
    printf("P12v rail is disabled");
  }else if(GPIO_Pin == ina_neg_12V.int_pin){
    ina = &ina_neg_12V;
    DISABLE_N12_SUPPLY();
    HAL_Delay(1000);
    ENABLE_N12_SUPPLY();
    printf("N12v rail is disabled");
  }else if (GPIO_Pin == ALERT_A_Pin) {
    ALARM_MANAGEMENT(0);
  }
}

void _putchar(char character){
  HAL_UART_Transmit(&huart1, &character, 1, 1);
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
  __disable_irq();
  while (1)
  {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
