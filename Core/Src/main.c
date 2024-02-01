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
#include "u8g2/u8g2.h"
#include "display.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HICCUP_TIME_MS    1000
#define DEBUG_INTERVAL    1000
#define EFF_5V_CONV       0.9
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
#define WITHIN_RANGE(x, target, range)  (x >= (target - range) && x <= (target + range))
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
int Plim_guardband = 0;   // MARGIN FOR TRIPPING OVERDRAW IN mW
int Hiccup_5v_flag = 0;       // Setting flag for 5v rail
int Hiccup_n12v_flag = 0;     // Setting flag for n12v rail
int Hiccup_p12v_flag =0;      // Setting flag for p12v rail
uint32_t hiccup_5v_ts = 0;
uint32_t hiccup_p12v_ts = 0;
uint32_t hiccup_n12v_ts = 0;
float ILIM_5V = 3.0;        // max rail current for 5V rail in Amperes
float ILIM_N12V = 4.0;      // max rail current for -12V rail in Amperes
float ILIM_12V = 8.0;      // max rail current for 12V *converter* in Amperes
uint32_t debug_stamp = 0;

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
extern STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef Nego_RDO[USBPORT_MAX];
extern uint8_t PDO_FROM_SRC_Valid[USBPORT_MAX];
extern uint8_t PDO_FROM_SRC_Num_Sel[USBPORT_MAX];
extern uint8_t PDO_FROM_SRC_Num[USBPORT_MAX];
extern uint8_t Policy_Engine_State[USBPORT_MAX];
extern uint8_t Go_disable_once[USBPORT_MAX];
extern uint8_t Final_Nego_done[USBPORT_MAX] ;
extern uint8_t Core_Process_suspended ;

/* OLED THINGS */
u8g2_t oled; // struct to contain data for one display
display_t display = {
  .oled = &oled
}; // struct to contain more stuff about the display

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void _putchar(char character);
extern void nvm_flash(uint8_t Usb_Port);
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t gpio_and_delay_callback(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

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
  // Send_Soft_reset_Message(usb_port_id);
  // SW_reset_by_Reg(usb_port_id);
  // HAL_Delay(1000);

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

  HAL_Delay(1000); // allow STUSB4500 to initialize

  Print_PDO_FROM_SRC(usb_port_id);

  Plim = Find_Max_SRC_PDO(usb_port_id); // Returns negotiated power level in mW

  printf("the power limit is %dmW \r\n", Plim);
  Send_Soft_reset_Message(usb_port_id); // forces re-negotiation with newly updated PDO2

  HAL_Delay(500); // give time for negotiation to occur and rail to stabilize
  
  Read_RDO(usb_port_id);
  
  Print_RDO(usb_port_id);

  // we can't move this elsewhere because we need to give time for the new
  // negotiated voltage to come up and stabilize.
  float nego_voltage = (float ) PDO_FROM_SRC[usb_port_id][Nego_RDO[usb_port_id].b.Object_Pos - 1].fix.Voltage/20.0;
  if (nego_voltage >= 12.0 && nego_voltage <= 16.0){
    printf("Negotiated voltage is %.2fV - Pulsing main converter off.", nego_voltage);
    DISABLE_PRIMARY_12V();
    for(int i=0; i<25; i++)
    {
      __NOP();
    }
    ENABLE_PRIMARY_12V();
  }

  if(WITHIN_RANGE(nego_voltage, 5.0, 2.0))
  {
    HAL_GPIO_WritePin(PD5V_GPIO_Port, PD5V_Pin, 1);
  }
  else if(WITHIN_RANGE(nego_voltage, 9.0, 2.0))
  {
    HAL_GPIO_WritePin(PD9V_GPIO_Port, PD9V_Pin, 1);
  }
  else if (WITHIN_RANGE(nego_voltage, 15.0, 2.0))
  {
    HAL_GPIO_WritePin(PD15V_GPIO_Port, PD15V_Pin, 1);
  }
  else if (WITHIN_RANGE(nego_voltage, 20.0, 2.0))
  {
    HAL_GPIO_WritePin(PD20V_GPIO_Port, PD20V_Pin, 1);
  }

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
  HAL_Delay(250);

  // Setting shunt cal register for each rail
  ina236_set_shuntcal(&ina_pos_12V);
  ina236_set_shuntcal(&ina_5V);
  ina236_set_shuntcal(&ina_neg_12V);
  HAL_Delay(250);

  // Setting current limits
  ina236_set_current_limit(&ina_5V, ILIM_5V);                                 // Setting constant current limit for 5V rail based on capability of converter
  ina236_set_current_limit(&ina_neg_12V, ILIM_N12V);                          // Setting constant current limit for n12V rail based on capability of converter
  ina236_set_current_limit(&ina_pos_12V, ILIM_12V);                           // Setting initial current limit for p12V rail assuming no draw from derivative rails
  HAL_Delay(250);

  // Enabling SOL Alert
  ina236_set_alertSOL(&ina_pos_12V);
  ina236_set_alertSOL(&ina_5V);
  ina236_set_alertSOL(&ina_neg_12V);
  HAL_Delay(250);

  /**** END SETUP INA236's ****/

  /**** BEGIN SETUP OLED *****/
  u8g2_Setup_ssd1306_i2c_128x64_noname_1(&oled, U8G2_R1, u8x8_byte_hw_i2c, gpio_and_delay_callback);
  u8g2_InitDisplay(&oled); // send init sequence to the display, display is in sleep mode after this,
  u8g2_SetPowerSave(&oled, 0); // wake up display

  /**** END SETUP OLED *****/

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
    float amps_5v = ina236_get_current(&ina_5V);
    float amps_n12v = ina236_get_current(&ina_neg_12V);

    float amps_5v_in = (volts_5v*amps_5v)/(EFF_5V_CONV*volts_p12v);
    float extra_draw_p12v = (amps_5v_in + amps_n12v);                           // Calculating draw on p12v rail NOT including the +12V output
    float draw_p12v = extra_draw_p12v + amps_p12v;                              // Total draw on the +12V rail
    float ILIM_12V_dyn = (ILIM_12V - extra_draw_p12v);                          // Updating p12v current limit based on 5v and n12v rail draw
    
    ina236_set_current_limit(&ina_pos_12V, ILIM_12V_dyn);                       // Setting INA236 current limit with dynamic limit based on derivative rail draw
    HAL_Delay(100);                                                             // This delay is probably unnecessary, but adding it during ILIM debug to slow down the dynamic current limit update

    // Reading power from INA236
    float power_p12V = 1000.0*ina236_get_power(&ina_pos_12V);                   // Returns +12V bus power in mW
    float power_n12V = 1000.0*ina236_get_power(&ina_neg_12V);                   // Returns -12V bus power in mW
    float power_5V = 1000.0*ina236_get_power(&ina_5V);                          // Returns +5V bus power in mW

    int tot_power = (int)(power_p12V + power_n12V + power_5V);                  // Calculating total power in mW
    
    if (HAL_GetTick() - debug_stamp > DEBUG_INTERVAL){
      debug_stamp = HAL_GetTick();
      printf("5V Bus: %.2fV, %.2fA, %.2fmW \r\n", volts_5v, amps_5v, power_5V);
      printf("+12V Bus: %.2fV, %.2fA, %.2fmW \r\n", volts_p12v, amps_p12v, power_p12V);
      printf("-12V Bus: -%.2fV, %.2fA, %.2fmW \r\n\r\n", volts_n12v, amps_n12v, power_n12V);
      printf("Total Current from +12V Rail: %.2fA \r\n\r\n", draw_p12v);
      printf("The Updated +12V Current Limit is %.2fA \r\n\r\n", ILIM_12V_dyn);
    }else if (HAL_GetTick() < debug_stamp){
      // tick counter has wrapped around
      debug_stamp = 0;
    }

    // Hiccup control Restart

    if(Hiccup_5v_flag == 1 && HAL_GetTick() - hiccup_5v_ts > HICCUP_TIME_MS)
    {
      ENABLE_5P0_SUPPLY();
      Hiccup_5v_flag = 0;
      DISABLE_OVRLD_LED();
      ENABLE_PDGOOD_LED();
    }

    if (Hiccup_n12v_flag == 1 && HAL_GetTick() - hiccup_n12v_ts > HICCUP_TIME_MS)
    {
      ENABLE_N12_SUPPLY();
      Hiccup_n12v_flag = 0;
      DISABLE_OVRLD_LED();
      ENABLE_PDGOOD_LED();
    }

    if (Hiccup_p12v_flag == 1 && HAL_GetTick() - hiccup_p12v_ts > HICCUP_TIME_MS)
    {
      ENABLE_PRIMARY_12V();
      Hiccup_p12v_flag = 0;
      DISABLE_OVRLD_LED();
      ENABLE_PDGOOD_LED();
    }

    // Conditional Statement for Lighting overload LED based on total power draw
    if(tot_power >= (Plim - Plim_guardband))
    {
      DISABLE_PDGOOD_LED();
      ENABLE_OVRLD_LED();
      DISABLE_PRIMARY_12V();
      Hiccup_p12v_flag = 1;             // If total system power exceeds limit - guardband, hiccup entire system (primary)
      hiccup_p12v_ts = HAL_GetTick();
      printf("Total power is exceeded");
    }
    
    else if (tot_power < (Plim - Plim_guardband))
    {
      DISABLE_OVRLD_LED();
      ENABLE_PDGOOD_LED();
    }

    HAL_Delay(100);            // delay now limits rate of polling INA's
    
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

// Hiccup control
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == ina_5V.int_pin){
    DISABLE_5PO_SUPPLY();
    Hiccup_5v_flag = 1;
    hiccup_5v_ts = HAL_GetTick();
    DISABLE_PDGOOD_LED();
    ENABLE_OVRLD_LED();
  }else if(GPIO_Pin == ina_pos_12V.int_pin){
    DISABLE_PRIMARY_12V();
    Hiccup_p12v_flag = 1;
    hiccup_p12v_ts = HAL_GetTick();
  }else if(GPIO_Pin == ina_neg_12V.int_pin){
    DISABLE_N12_SUPPLY();
    Hiccup_n12v_flag = 1;
    hiccup_n12v_ts = HAL_GetTick();
  }else if (GPIO_Pin == ALERT_A_Pin) {
    ALARM_MANAGEMENT(0);
  }
}

void _putchar(char character){
  HAL_UART_Transmit(&huart1, &character, 1, 1);
}

/**
 * Handle hardware I2C comms
*/
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr){
  uint8_t *data;

  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;
      HAL_I2C_Master_Transmit(&hi2c1, u8x8_GetI2CAddress(u8x8), data, arg_int, 50);
      break;
      
    case U8X8_MSG_BYTE_INIT:
      // i2c_init(u8x8);
      break;
    case U8X8_MSG_BYTE_SET_DC:
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
      // i2c_start(u8x8);
      // i2c_write_byte(u8x8, u8x8_GetI2CAddress(u8x8));
      //i2c_write_byte(u8x8, 0x078);
      break;
    case U8X8_MSG_BYTE_END_TRANSFER:
      // i2c_stop(u8x8);
      break;
    default:
      return 0;
  }
  return 1;
}


uint8_t gpio_and_delay_callback(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr){
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8
      break;							// can be used to setup pins
    case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
      break;    
    case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
        for(int i = 0; i < arg_int*7; i++){
          __NOP();
        }
      break;
    case U8X8_MSG_DELAY_10MICRO:		// delay arg_int * 10 micro seconds
        for(int i = 0; i < arg_int*200; i++){
          __NOP();
        }
      break;
    case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
      HAL_Delay(arg_int);
      break;
    case U8X8_MSG_DELAY_I2C:				// arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
      if (arg_int == 1){
        // this should be ~very roughly 5us
        for(int i = 0; i < 100; i++){
          __NOP();
        }
      }else{
        // this should be ~very roughly 1.25us
        for(int i = 0; i < 25; i++){
          __NOP();
        }
      }
      break;							// arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
    case U8X8_MSG_GPIO_D0:				// D0 or SPI clock pin: Output level in arg_int
    //case U8X8_MSG_GPIO_SPI_CLOCK:
      break;
    case U8X8_MSG_GPIO_D1:				// D1 or SPI data pin: Output level in arg_int
    //case U8X8_MSG_GPIO_SPI_DATA:
      break;
    case U8X8_MSG_GPIO_D2:				// D2 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D3:				// D3 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D4:				// D4 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D5:				// D5 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D6:				// D6 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_D7:				// D7 pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_E:				// E/WR pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_CS:				// CS (chip select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_DC:				// DC (data/cmd, A0, register select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_RESET:			// Reset pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_CS1:				// CS1 (chip select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_CS2:				// CS2 (chip select) pin: Output level in arg_int
      break;
    case U8X8_MSG_GPIO_I2C_CLOCK:		// arg_int=0: Output low at I2C clock pin
      break;							// arg_int=1: Input dir with pullup high for I2C clock pin
    case U8X8_MSG_GPIO_I2C_DATA:			// arg_int=0: Output low at I2C data pin
      break;							// arg_int=1: Input dir with pullup high for I2C data pin
    case U8X8_MSG_GPIO_MENU_SELECT:
      // u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
      // u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
      // u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
      break;
    case U8X8_MSG_GPIO_MENU_HOME:
      // u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
      break;
    default:
      // u8x8_SetGPIOResult(u8x8, 1);			// default return value
      break;
  }
  return 1;
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
