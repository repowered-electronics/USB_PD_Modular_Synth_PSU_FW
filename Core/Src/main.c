/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  * Copyright (C) 2024  Repowered Electronics LLC
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU Lesser General Public License for more details.
  * 
  * You should have received a copy of the GNU Lesser General Public License
  * along with this program.  If not, see <https://www.gnu.org/licenses/>.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "printf.h"
#ifndef PRINTF
#define PRINTF
#endif
#include "USB_PD_core.h"
#include "ina236.h"
#include "u8g2/u8g2.h"
#include "display.h"
#include <math.h>
#include "ee.h"
#include "eeConfig.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HICCUP_TIME_MS          1000
#define DEBUG_INTERVAL          10000
#define EFF_5V_CONV             0.9
#define UART_RX_BUF_SIZE        128
#define UART_TX_BUF_SIZE        1024  // need more since we'll be printing PDO strings potentially
#define ENABLE_12V_TIMEOUT_US   1000000
#define INIT_STARTUP_DLY_MS     50
#define STUSB_INIT_DLY_MS       100
#define DUMB_CHG_IQ_MA          140   // current drawn from VBUS when it is ~5V
#define DUMB_CHG_CURR_LIM_MA    (6000 - DUMB_CHG_IQ_MA)  // max allowed to be drawn from VBUS when it's ~5V (used for Plim calc)
#define OVERLD_LED_WARN_OFF_MS  750
#define OVERLD_LED_WARN_ON_MS   250

#define OLED_UPDATE_PERIOD      1500
#define INA_UPDATE_PERIOD       100

#define CHG_TYPE_DUMB           0
#define CHG_TYPE_PD             1

#define OUTPUT_OFF              0
#define OUTPUT_ON               1

#define EFF_COEF_COUNT          2

#define IBB_IQ                  0.005


enum vbus_enum {
  VBUS_5V,
  VBUS_9V,
  VBUS_15V,
  VBUS_20V
};

enum debug_modes_enum {
  DEBUG_NORMAL = 0,
  DEBUG_DEVELOPER = 1,
  DEBUG_QUIET     = 2
};

enum debug_cmds_enum {
  CMD_NONE,
  CMD_READ_REGS,
  CMD_WRITE_REGS,
  CMD_PRINT_SRC_PDOS,
  CMD_COUNT
};

enum reg_map_enum {
  REG_DEBUG_MODE,
  REG_PLIM,           // float (W)
  REG_PLIM_GUARDBAND, // float (W)
  REG_PLIM_WARN,      // float (W)
  REG_ILIM_5V,        // float (A)
  REG_ILIM_N12V,      // float (A)
  REG_ILIM_12V,       // float (A)
  REG_NEGO_VBUS,      // float (V)
  REG_VBUS_ILIM,      // float (A)
  REG_VOLTS_5V,
  REG_VOLTS_N12V,
  REG_VOLTS_12V,
  REG_VOLTS_VBUS,
  REG_CURR_5V,
  REG_CURR_N12V,
  REG_CURR_12V,
  REG_CURR_VBUS,
  REG_POWER_VBUS,
  REG_BUCK_BST_5VIN_C0,
  REG_BUCK_BST_5VIN_C1,
  REG_BUCK_BST_9VIN_C0,
  REG_BUCK_BST_9VIN_C1,
  REG_BUCK_BST_15VIN_C0,
  REG_BUCK_BST_15VIN_C1,
  REG_BUCK_BST_20VIN_C0,
  REG_BUCK_BST_20VIN_C1,
  REG_5V_CONV_C0,
  REG_5V_CONV_C1,
  REG_IBB_5VIN_C0,
  REG_IBB_5VIN_C1,
  REG_IBB_9VIN_C0,
  REG_IBB_9VIN_C1,
  REG_IBB_15VIN_C0,
  REG_IBB_15VIN_C1,
  REG_IBB_20VIN_C0,
  REG_IBB_20VIN_C1,
  REG_DISP_PRESENT,
  REG_COUNT
};

enum rail_ids_enum {
  RAIL_NONE,
  RAIL_P12V,
  RAIL_N12V,
  RAIL_5V
};

#define UART_STATE_RX_HDDR        0
#define UART_STATE_RX_BODY        1
#define UART_STATE_TX_PACKET      2

#define UART_HDDR_SIZE            4
typedef uint32_t* regmap_t;
#define REG_MAP_DATA_SIZE         (4)
#define REG_MAP_ENTRY_SIZE        (sizeof(regmap_t))       /* size of a single register */

#define SAVE_COEFS_TO_FLASH_MASK  0x000000003FFC0000

#define LOOP_DELAY                25      /* sleep inserted in main loop */
#define OVLD_DUR_BEFORE_HICCUP    200     /* millisecs of time spent in ovld before we should hiccup everything */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ENABLE_PRIMARY_12V() HAL_GPIO_WritePin(DISABLE_PRI_12V_GPIO_Port, DISABLE_PRI_12V_Pin, 0)
#define DISABLE_PRIMARY_12V() HAL_GPIO_WritePin(DISABLE_PRI_12V_GPIO_Port, DISABLE_PRI_12V_Pin, 1)
#define PRI_12V_STATE() (((DISABLE_PRI_12V_GPIO_Port->ODR >> DISABLE_PRI_12V_Pin) & 1) ^ 1)
#define ENABLE_PDGOOD_LED() HAL_GPIO_WritePin(PDGOOD_GPIO_Port, PDGOOD_Pin, 1)
#define DISABLE_PDGOOD_LED() HAL_GPIO_WritePin(PDGOOD_GPIO_Port, PDGOOD_Pin, 0)
#define ENABLE_OVRLD_LED() HAL_GPIO_WritePin(PDBAD_GPIO_Port, PDBAD_Pin, 1)
#define DISABLE_OVRLD_LED() HAL_GPIO_WritePin(PDBAD_GPIO_Port, PDBAD_Pin, 0)
#define OVRLD_LED_STATE() ((PDBAD_GPIO_Port->ODR >> PDBAD_Pin) & 1)
#define DISABLE_5PO_SUPPLY() HAL_GPIO_WritePin(DISABLE_5P0_GPIO_Port, DISABLE_5P0_Pin, 1)
#define ENABLE_5P0_SUPPLY() HAL_GPIO_WritePin(DISABLE_5P0_GPIO_Port, DISABLE_5P0_Pin, 0)
#define ENABLE_N12_SUPPLY() HAL_GPIO_WritePin(DISABLE_N12_GPIO_Port, DISABLE_N12_Pin, 0)
#define DISABLE_N12_SUPPLY() HAL_GPIO_WritePin(DISABLE_N12_GPIO_Port, DISABLE_N12_Pin, 1)
#define WITHIN_RANGE(x, target, range)  (x >= (target - range) && x < (target + range))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
INA236_t ina_5V;
INA236_t ina_pos_12V;
INA236_t ina_neg_12V;
float volts_p12v = 12.0;
float volts_5v = 5.0;
volatile float volts_n12v = 0.0;
float amps_out_p12v = 0.0;
float amps_out_5v = 0.0;
float amps_in_n12v = 0.0;
float amps_out_n12v = 0.0;
float pwr_out_p12V = 0.0;                
float pwr_in_n12V = 0.0;                 
float pwr_out_5V = 0.0;
float pwr_out_n12V = 0.0;
float tot_idraw_p12v = 0.0;
float ILIM_12V_dyn = 0.0;
float tot_pwr_out = 0.0;


I2C_HandleTypeDef *hi2c[I2CBUS_MAX];
unsigned int Address;
unsigned int AddressSize = I2C_MEMADD_SIZE_8BIT;
USB_PD_I2C_PORT STUSB45DeviceConf[USBPORT_MAX];
uint32_t timer_cnt = 0;
int Flag_count = 0;
int PB_press=0;
int Time_elapse=1;
float Plim = 0.0;              // Power limit from find max pdo in WATTS not mW
float Plim_guardband = 0.100;  // MARGIN FOR TRIPPING OVERDRAW IN WATTS not mW
float Plim_warn = 0.250;       // Margin for triggering warning of impending overdraw in WATTS not mW
uint32_t overld_led_ts = 0;
int Hiccup_5v_flag = 0;       // Setting flag for 5v rail
int Hiccup_n12v_flag = 0;     // Setting flag for n12v rail
int Hiccup_p12v_flag = 0;     // Setting flag for p12v rail
int ina_alerts_enabled = 0;   // will be used to gate servicing of alert pin interrupts
uint32_t hiccup_5v_ts = 0;
uint32_t hiccup_p12v_ts = 0;
uint32_t hiccup_n12v_ts = 0;
int ovld_count = 0;           // how many main loop iterations have we been in overload for
float ILIM_5V = 3.0;          // max rail current for 5V rail in Amperes
float ILIM_N12V = 4.0;        // max rail current for -12V rail in Amperes
float ILIM_12V = 8.0;         // max rail current for 12V *converter* in Amperes
uint32_t debug_stamp = 0;
uint32_t update_oled_stamp = 0;
uint32_t ina_update_stamp = 0;

float nego_vbus = 0.0;    // only used in developer mode and only if non-zero
float vbus_ilim = 0;    // again, only used in developer mode and only if non-zero

regmap_t regmap[REG_COUNT];
bool writeable[REG_COUNT];
uint64_t reg_change_flags = 0;
int rx_size = 0;
uint8_t uart_state = UART_STATE_RX_HDDR;
uint8_t rx_buf[UART_RX_BUF_SIZE];
uint8_t tx_buf[UART_TX_BUF_SIZE];
bool process_uart_buf = false;
uint32_t debug_mode = DEBUG_NORMAL;
uint8_t print_src_pdos_cmd[] = "PRINT_SRC_PDOS";
float req_voltage = 5.0;
float req_current = 1.5;
int req_number = 0;

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
uint32_t disp_present = 1; // default to being present
display_t display = {
  .oled = &oled
}; // struct to contain more stuff about the display

/* ADC things */
volatile float vbus_meas = 0.0;
float amps_in_vbus = 0.0;
int32_t curr_vbus_uA = 0; /* effectively computed based on measurements */
float pwr_vbus = 0;
uint32_t adc_samples[2];
volatile int adc_sample_count = 0;
uint32_t adc_sample_sums[2];
volatile bool adc_conv_cplt = false;
volatile uint32_t micros_count = 0;
volatile uint32_t micros_count_by_32 = 0;
volatile uint8_t chg_type   = CHG_TYPE_DUMB;
volatile uint8_t output_state = OUTPUT_OFF;

/* efficiency things */
int vbus_range = VBUS_5V;                                                       /* default to 5V, will change ASAP */
float eff_coefs_buck_boost_5vin[EFF_COEF_COUNT] = {0.712, 1.057}; // {649250, 1.05682};
float eff_coefs_buck_boost_9vin[EFF_COEF_COUNT] = {0.655, 1.038}; //{0.62943, 1.03610};
float eff_coefs_buck_boost_15vin[EFF_COEF_COUNT] = {0.982, 1.035}; //{0.6272, 1.03408};
float eff_coefs_buck_boost_20vin[EFF_COEF_COUNT] = {0.671, 1.047}; //{0.7675, 1.05402};
float eff_coefs_5v_conv[EFF_COEF_COUNT] = {0.0, 1.0}; // {0.0, 1.0};
float eff_coefs_ibb_5vin[EFF_COEF_COUNT] = {-0.025, 0.883}; // {0.0, 1.0};
float eff_coefs_ibb_9vin[EFF_COEF_COUNT] = {-0.157, 0.887}; // {0.0, 1.0};
float eff_coefs_ibb_15vin[EFF_COEF_COUNT] = {-0.270, 0.886}; // {0.0, 1.0};
float eff_coefs_ibb_20vin[EFF_COEF_COUNT] = {-0.369, 0.880}; // {0.0, 1.0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void _putchar(char character);
extern void nvm_flash(uint8_t Usb_Port);
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t gpio_and_delay_callback(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void disable_ina_ints();
void enable_ina_ints();
void restart_UART_rx();
bool char_in_buffer(uint8_t* buffer, char c, int size);
void strstrip(char* str, char* loc);
void init_regmap(void);
void read_eff_coefs_from_flash(void);
void save_eff_coefs_to_flash(void);
void process_dev_packet();
void check_on_pdo_voltage(int usb_port);
void update_vbus_leds(float vbus);
void delay_us(uint32_t delay);
void set_output_state(uint8_t on_or_off);
uint32_t micros();
float compute_polynom(float* coefs, int coef_count, float x);
void enter_sleep_mode(void);
void hiccup_rail(int rail_id);

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* Start microsecond timer */
  HAL_TIM_Base_Start_IT(&htim1);

  /* ENSURE EXTERNAL SUPPLIES START IN OFF STATE */
  output_state = OUTPUT_OFF;
  set_output_state(output_state);

  DISABLE_OVRLD_LED();
  DISABLE_PDGOOD_LED(); 
  disable_ina_ints(); // start by not acting on interrupts from the INAs

  HAL_Delay(INIT_STARTUP_DLY_MS);

  /* SETUP USB PD STUFF */
  hi2c[0] = &hi2c1;
  STUSB45DeviceConf[usb_port_id].I2cBus = usb_port_id;
  STUSB45DeviceConf[usb_port_id].I2cDeviceID_7bit = 0x28;
  STUSB45DeviceConf[usb_port_id].Alert_GPIO_Bank = (uint8_t)USBPD_ALERT_GPIO_Port;
  STUSB45DeviceConf[usb_port_id].Alert_GPIO_Pin = USBPD_ALERT_Pin;
  AddressSize = I2C_MEMADD_SIZE_8BIT; 
  USB_PD_Interupt_PostponedFlag[0] = 0; /* this flag is 1 if I2C is busy when Alert signal raise */
  USB_PD_Interupt_Flag[usb_port_id] = 1;
  Final_Nego_done[usb_port_id] = 0;
  Go_disable_once[usb_port_id] = 0;

  memset((uint32_t *)PDO_FROM_SRC[usb_port_id], 0, 7*sizeof(uint32_t*));
  memset((uint32_t *)PDO_SNK[usb_port_id], 0, 3*sizeof(uint32_t*));

  usb_pd_init(usb_port_id);   // after this USBPD alert line must be high 

  HAL_Delay(100); /* delay needed for correct assertion of ATTACH */

  /* ASSESS: USB-PD CHARGER OR DUMB-CHARGER */
  if(!HAL_GPIO_ReadPin(ATTACH_GPIO_Port, ATTACH_Pin)){
    // a CC line is attached, negotiate PD
    chg_type = CHG_TYPE_PD;
    Send_Soft_reset_Message(usb_port_id); //
    
    HAL_Delay(250); //STUSB_INIT_DLY_MS); // allow STUSB4500 to Initialize

    Print_PDO_FROM_SRC(usb_port_id);

    int Plim_mW = Find_Max_SRC_PDO(usb_port_id); // Returns negotiated power level in mW
    printf("the power limit is %dmW \r\n", Plim_mW);
    Plim = Plim_mW / 1000.0; 
    
    Send_Soft_reset_Message(usb_port_id); //

    HAL_Delay(500); // give time for negotiation to occur and rail to stabilize
    
    Read_RDO(usb_port_id);
    
    Print_RDO(usb_port_id);

    // we can't move this elsewhere because we need to give time for the new
    // negotiated voltage to come up and stabilize.
    nego_vbus = (float)PDO_FROM_SRC[usb_port_id][Nego_RDO[usb_port_id].b.Object_Pos - 1].fix.Voltage/20.0;
    vbus_ilim = (float)PDO_FROM_SRC[usb_port_id][Nego_RDO[usb_port_id].b.Object_Pos - 1].fix.Max_Operating_Current/100.0;
    /* vbus LEDs get updated in the main loop if power btn is asserted */
    
    connection_flag[usb_port_id] = 1;
    Previous_VBUS_Current_limitation[usb_port_id] = VBUS_Current_limitation[usb_port_id];
  }else{
    // Dumb charger
    printf("Dumb charger detected...\r\n");
    chg_type = CHG_TYPE_DUMB;
    Plim = 5.0 * DUMB_CHG_CURR_LIM_MA;
    /* vbus LEDs get updated in the main loop if power btn is asserted */
  }

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
  HAL_Delay(5);

  // Setting shunt cal register for each rail
  ina236_set_shuntcal(&ina_pos_12V);
  ina236_set_shuntcal(&ina_5V);
  ina236_set_shuntcal(&ina_neg_12V);
  HAL_Delay(5);

  // Setting current limits
  ina236_set_current_limit(&ina_5V, ILIM_5V);                                 // Setting constant current limit for 5V rail based on capability of converter
  ina236_set_current_limit(&ina_neg_12V, ILIM_N12V);                          // Setting constant current limit for n12V rail based on capability of converter
  ina236_set_current_limit(&ina_pos_12V, ILIM_12V);                           // Setting initial current limit for p12V rail assuming no draw from derivative rails
  HAL_Delay(5);

  // Enabling SOL Alert
  ina236_set_alertSOL(&ina_pos_12V);
  ina236_set_alertSOL(&ina_5V);
  ina236_set_alertSOL(&ina_neg_12V);
  HAL_Delay(5);

  enable_ina_ints();
  /**** END SETUP INA236's ****/

  /**** BEGIN SETUP OLED *****/
  u8g2_Setup_ssd1306_i2c_128x32_univision_f(&oled, U8G2_R2, u8x8_byte_hw_i2c, gpio_and_delay_callback);
  u8g2_InitDisplay(&oled); // send init sequence to the display, display is in sleep mode after this,
  u8g2_SetPowerSave(&oled, 0); // wake up display
  u8g2_ClearDisplay(&oled);
  u8g2_SetFont(&oled, u8g2_font_6x13_tf);
  u8g2_SetFontDirection(&oled, 1);
  /**** END SETUP OLED *****/

  HAL_UART_Receive_DMA(&huart1, rx_buf, UART_HDDR_SIZE);

  HAL_ADC_Stop(&hadc1); // ensure ADC is STOPPED
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(25);

  HAL_ADC_Start_DMA(&hadc1, adc_samples, 2); //Trigger ADC DMA once, future triggers will be from the timer
  HAL_Delay(1);
  HAL_TIM_Base_Start_IT(&htim3); // start timer for actually triggering ADC conversions (2.5kHz)

  // ee_init();
  // read_eff_coefs_from_flash();
  init_regmap();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ----- Check on power switch ----- */
    int pwr_btn_state = HAL_GPIO_ReadPin(PWR_BTN_GPIO_Port, PWR_BTN_Pin);
    if (pwr_btn_state == 0 && output_state == OUTPUT_OFF) {
      /* transition to ON STATE */
      // ENABLE_PDGOOD_LED();
      output_state = OUTPUT_ON;
      set_output_state(output_state);
      u8g2_SetPowerSave(&oled, 0); // wake up display
      update_vbus_leds(vbus_meas);
    } else if (pwr_btn_state == 1 && output_state == OUTPUT_ON) {
      /* transition to OFF state*/
      output_state = OUTPUT_OFF;
      set_output_state(output_state);
      u8g2_SetPowerSave(&oled, 1); // shut down OLED display
      DISABLE_PDGOOD_LED();
      update_vbus_leds(0.0);
      /* go into sleep mode, will be woken from sleep on falling edge of PWR_BTN signal */
      enter_sleep_mode();
    }

    if (output_state == OUTPUT_OFF) {
      HAL_Delay(10);
      continue; /* skip over the rest of the main loop */
    }

    /* UPDATE INA236's -------------------------------------------------------*/
    if (HAL_GetTick() - ina_update_stamp > INA_UPDATE_PERIOD) {
      // Reading bus Voltage from INA236
      volts_p12v    = ina236_get_voltage(&ina_pos_12V);
      volts_5v      = ina236_get_voltage(&ina_5V);
      /* volts_n12v = -12V rail measurement: handled by TIM period callback */

      // Reading current from INA236
      amps_out_p12v = ina236_get_current(&ina_pos_12V);
      amps_out_5v   = ina236_get_current(&ina_5V);
      amps_in_n12v  = ina236_get_current(&ina_neg_12V);

      // Reading power from INA236
      pwr_out_p12V = ina236_get_power(&ina_pos_12V);                 // Returns +12V bus power in mW
      pwr_in_n12V = ina236_get_power(&ina_neg_12V);                  // Returns -12V bus power in mW
      pwr_out_5V = ina236_get_power(&ina_5V);                        // Returns +5V bus power in mW

      /* reflect 5V output power back to input */
      float pwr_in_5v = compute_polynom(eff_coefs_5v_conv, EFF_COEF_COUNT, pwr_out_5V);

      float* buck_boost_coefs;
      float* ibb_coefs;
      switch (vbus_range){
        default:
        case VBUS_5V:
          buck_boost_coefs = eff_coefs_buck_boost_5vin;
          ibb_coefs = eff_coefs_ibb_5vin;
          break;
        case VBUS_9V:
          buck_boost_coefs = eff_coefs_buck_boost_9vin;
          ibb_coefs = eff_coefs_ibb_9vin;
          break;
        case VBUS_15V:
          buck_boost_coefs = eff_coefs_buck_boost_15vin;
          ibb_coefs = eff_coefs_ibb_15vin;
          break;
        case VBUS_20V:
          buck_boost_coefs = eff_coefs_buck_boost_20vin;
          ibb_coefs = eff_coefs_ibb_20vin;
          break;
      };

      /* estimate output power of -12V IBB */
      if (pwr_in_n12V > vbus_meas * IBB_IQ) {                         // if pwr in is greater than approx quiescent power
        pwr_out_n12V = compute_polynom(ibb_coefs, EFF_COEF_COUNT, pwr_in_n12V);
      } else {
        pwr_out_n12V = 0.0;
      }
      amps_out_n12v = pwr_out_n12V / volts_n12v;                       // this is what should be reported to the user

      /* reflect 12V power out back to VBUS (source) */
      float pwr_p12v_total = (pwr_in_5v + pwr_out_p12V);               // total power drawn out of the 12V buck-boost
      pwr_vbus = compute_polynom(buck_boost_coefs, EFF_COEF_COUNT, pwr_p12v_total) + pwr_in_n12V;
      amps_in_vbus = pwr_vbus / vbus_meas;

      float amps_5v_in = pwr_in_5v/volts_5v;
      tot_idraw_p12v = amps_5v_in + amps_out_p12v;                     // Total draw on the +12V rail
      ILIM_12V_dyn = (ILIM_12V - amps_5v_in);                          // Updating p12v current limit based on 5v and n12v rail draw
      
      if (debug_mode == DEBUG_DEVELOPER) {
        ILIM_12V_dyn = 12.0; // override the limit for testing - make this unreasonably large
      }
      ina236_set_current_limit(&ina_pos_12V, ILIM_12V_dyn);                       // Setting INA236 current limit with dynamic limit based on derivative rail draw
    }

    /* Debug if it's time ----------------------------------------------------*/
    if (HAL_GetTick() - debug_stamp > DEBUG_INTERVAL){
      debug_stamp = HAL_GetTick();
      printf("VBUS    : %.3fV \r\n", vbus_meas);
      printf("5V Bus  : %.3fV, %.4fA, %.4fmW \r\n", volts_5v, amps_out_5v, pwr_out_5V*1000.0);
      printf("+12V Bus: %.3fV, %.4fA, %.4fmW \r\n", volts_p12v, amps_out_p12v, pwr_out_p12V*1000.0);
      printf("-12V Bus: %.3fV, %.4fA, %.4fmW \r\n\r\n", volts_n12v, amps_out_n12v, pwr_out_n12V*1000.0);
      printf("Total Current from +12V Rail: %.4fA \r\n\r\n", tot_idraw_p12v);
      printf("The Updated +12V Current Limit is %.4fA \r\n\r\n", ILIM_12V_dyn);
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
    if(pwr_vbus >= (Plim + Plim_guardband))
    {
      ovld_count += LOOP_DELAY;
      if (ovld_count >= OVLD_DUR_BEFORE_HICCUP) {
        ovld_count = 0;
        DISABLE_PDGOOD_LED();
        ENABLE_OVRLD_LED();
        hiccup_rail(RAIL_P12V);
        hiccup_rail(RAIL_N12V);
        hiccup_rail(RAIL_5V);
        printf("Total power draw (%.2f W) exceeded limit (%.2f W)", pwr_vbus, Plim);
      }
    }
    else if (pwr_vbus >= (Plim - Plim_warn))
    {
      /* total power drawn from Vbus is getting close to overload. */
      /* Flash PDBAD (OVERLOAD) LED */
      if (OVRLD_LED_STATE() == 0 && HAL_GetTick() - overld_led_ts > OVERLD_LED_WARN_OFF_MS){
        // LED is currently OFF, and it's been off long enough
        ENABLE_OVRLD_LED();
        overld_led_ts = HAL_GetTick();
      }else if(OVRLD_LED_STATE() == 1 && HAL_GetTick() - overld_led_ts > OVERLD_LED_WARN_ON_MS){
        // LED is ON now, and it's been on long enough
        DISABLE_OVRLD_LED();
        overld_led_ts = HAL_GetTick();
      }
      ovld_count = 0;
    }
    else
    {
      DISABLE_OVRLD_LED();
      ENABLE_PDGOOD_LED();
      ovld_count = 0;
    }

    /* BEGIN DISPLAY THINGS --------------------------------------------------*/
    if (HAL_GetTick() - update_oled_stamp > OLED_UPDATE_PERIOD) {
      update_oled_stamp = HAL_GetTick();
    
      u8g2_ClearBuffer(&oled);

      char buffer[10];

      //Line to Separate regulators
      u8g2_DrawLine(&oled, 90, 0, 90, 32);

      //Report +12V Stats
      int p12VTopLine = 115;
      u8g2_DrawStr(&oled, p12VTopLine, 0, "+12V");
      snprintf(buffer, 10, "%.1fV", volts_p12v);
      u8g2_DrawStr(&oled, p12VTopLine - 10, 0, buffer);
      
      if(amps_out_p12v > 0.1) {
        snprintf(buffer, 10, "%.2fA", amps_out_p12v);
      }
      else {
        snprintf(buffer, 10, "%dmA", (int)(1000*amps_out_p12v));
      }
      u8g2_DrawStr(&oled, p12VTopLine - 20, 0, buffer);
      
      //Report -12V Stats
      int n12VTopLine = 75;
      u8g2_DrawStr(&oled, n12VTopLine, 0, "-12V");
      snprintf(buffer, 10, "%.1fV", volts_n12v);
      u8g2_DrawStr(&oled, n12VTopLine - 10, 0, buffer);

      if (amps_out_n12v > 0.1){
        sprintf(buffer, "%.2fA", amps_out_n12v);
      }
      else {
        sprintf(buffer, "%dmA", (int)(1000*amps_out_n12v));
      }
      u8g2_DrawStr(&oled, n12VTopLine - 20, 0, buffer);

      //Line to Separate regulators
      u8g2_DrawLine(&oled, 50, 0, 50, 32);

      //Report +5V Stas
      int p5VTopLine = 35;
      u8g2_DrawStr(&oled, p5VTopLine, 0, "+5V");
      sprintf(buffer, "%.2fV", volts_5v);
      u8g2_DrawStr(&oled, p5VTopLine - 10, 0, buffer);
      if(amps_out_5v > 0.1) {
        sprintf(buffer, "%.2fA", amps_out_5v);
      }
      else {
        sprintf(buffer, "%dmA", (int)(1000*amps_out_5v));
      }
      u8g2_DrawStr(&oled, p5VTopLine - 20, 0, buffer);
      
      if (Hiccup_p12v_flag || Hiccup_5v_flag || Hiccup_n12v_flag){
        u8g2_DrawStr(&oled, 0, 0, "OVLD!!");
      }
      else {
        float nego_voltage = (float ) PDO_FROM_SRC[usb_port_id][Nego_RDO[usb_port_id].b.Object_Pos - 1].fix.Voltage/20.0;
        sprintf(buffer, "%.1fV", nego_voltage);
        u8g2_DrawStr(&oled, 0, 0, buffer);
      }

      u8g2_SendBuffer(&oled);
    }
    /* END DISPLAY THINGS --------------------------------------------------*/

    // UART DEBUGGING THINGS
    if (uart_state == UART_STATE_TX_PACKET) {
      uint8_t cmd = rx_buf[0];
      // uint8_t aux_value = rx_buf[1];
      uint16_t body_len = rx_buf[2] | (rx_buf[3] << 8);
      uint8_t* rx_body = rx_buf + UART_HDDR_SIZE;
      uint8_t* tx_body = tx_buf + UART_HDDR_SIZE;
      bool tx_body_rdy = false;
      int tx_body_ind = 0;
      switch (cmd) {
        default:
        case CMD_NONE:
          break;
        case CMD_READ_REGS: {
          for (int i = 0; i < body_len; i++) {
            int regind = rx_body[i];
            if (regind >= REG_COUNT) {
              continue;
            }
            tx_body[tx_body_ind++] = regind;
            memcpy(tx_body + tx_body_ind, (uint8_t*)(regmap[regind]), REG_MAP_DATA_SIZE);
            tx_body_ind += REG_MAP_DATA_SIZE;
          }
          tx_body_rdy = true;
          break;
        }
        case CMD_WRITE_REGS: {
          for (int i = 0; i < body_len; i += (1 + REG_MAP_DATA_SIZE)) {
            int regind = rx_body[i];
            if (writeable[regind]) {
              regmap_t ptr = regmap[regind];
              reg_change_flags |= (1 << regind);
              memcpy((uint8_t*)ptr, rx_body + (i + 1), REG_MAP_DATA_SIZE);
              tx_body[tx_body_ind++] = regind;
            }
          }
          tx_body_rdy = true;
          break;
        }
        case CMD_PRINT_SRC_PDOS: {
          /* special case that breaks the rules */
          debug_mode = DEBUG_NORMAL;
          Print_PDO_FROM_SRC(0); // usb port ID is always going to be 0 for our design
          debug_mode = DEBUG_DEVELOPER;
          break;
        }
      }
      if (tx_body_rdy) {
        tx_buf[0] = cmd;
        tx_buf[1] = 0; // TODO something later maybe with this
        tx_buf[2] = tx_body_ind & 0xFF;
        tx_buf[3] = (tx_body_ind >> 8) & 0xFF;
        HAL_UART_Transmit_DMA(&huart1, tx_buf, tx_body_ind + UART_HDDR_SIZE);
      }
      uart_state = UART_STATE_RX_HDDR;
      restart_UART_rx();
    }

    /* Check on special actions needed */
    if (reg_change_flags != 0) {
      if (reg_change_flags & SAVE_COEFS_TO_FLASH_MASK) {
        save_eff_coefs_to_flash();
      }
    }
    
    HAL_Delay(LOOP_DELAY);
    
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM3){
    adc_sample_sums[0] += adc_samples[0];
    adc_sample_sums[1] += adc_samples[1];
    adc_sample_count++;
    if (adc_sample_count >= 10){
      volts_n12v = (-5.0*3.3*(adc_sample_sums[0]/adc_sample_count))/4095.0;
      vbus_meas = (11.0*3.3*(adc_sample_sums[1]/adc_sample_count))/4095.0;

      if (vbus_meas < 7.0) {
        vbus_range = VBUS_5V;
      } else if (vbus_meas < 12.0) {
        vbus_range = VBUS_9V;
      } else if (vbus_meas < 17.5) {
        vbus_range = VBUS_15V;
      } else {
        vbus_range = VBUS_20V;
      }

      if (chg_type == CHG_TYPE_DUMB) {
        Plim = vbus_meas * (DUMB_CHG_CURR_LIM_MA/1000.0);
        update_vbus_leds(vbus_meas);
      }
      
      adc_sample_sums[0] = 0;
      adc_sample_sums[1] = 0;
      adc_sample_count = 0;
    }
    HAL_ADC_Start_DMA(&hadc1, adc_samples, 2); //Trigger ADC DMA
  }else if(htim->Instance == TIM1){
    micros_count_by_32++;
  }
}

// Hiccup control
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  int rail_id = RAIL_NONE;
  if(GPIO_Pin == ina_5V.int_pin && ina_alerts_enabled){
    rail_id = RAIL_5V;
  }else if(GPIO_Pin == ina_pos_12V.int_pin && ina_alerts_enabled){
    rail_id = RAIL_P12V;
  }else if(GPIO_Pin == ina_neg_12V.int_pin && ina_alerts_enabled){
    rail_id = RAIL_N12V;
  }else if (GPIO_Pin == ALERT_A_Pin) {
    ALARM_MANAGEMENT(0);
  }
  hiccup_rail(rail_id); // if rail_id remains RAIL_NONE, no action is taken
}

void _putchar(char character){
  if(debug_mode == DEBUG_NORMAL){
    HAL_UART_Transmit(&huart1, (uint8_t*)&character, 1, 1);
  }
}

/**
 * Restarts DMA-based UART reception
*/
void restart_UART_rx(){
  rx_size = 0;
  HAL_UART_Receive_DMA(&huart1, rx_buf, UART_HDDR_SIZE);
}

/**
 * Returns true if char c is in buffer
*/
bool char_in_buffer(uint8_t* buffer, char c, int size){
  for(int i = 0; i < size; i++){
    if (buffer[i] == c){
      return true;
    }
  }
  return false;
}

/**
 * strip whitespace from str storing result in loc
 * loc must have at least as much space as str
*/
void strstrip(char* str, char* loc){
  int i = 0;
  while(str[i] == ' ' || str[i] == '\n' || str[i] == '\r' || str[i] == '\t'){
    i++;
  }
  strcpy(loc, str + i); // copy over everything except leading whitespace

  i = strlen(loc) - 1; // get index of last character
  while(i >= 0 && (loc[i] == ' ' || loc[i] == '\n' || loc[i] == '\r' || loc[i] == '\t')){
    loc[i] = '\0'; // set this whitespace to null termination
    i--;
  }
}

/**
 * @brief initializes the global register map
 */
void init_regmap(void) {
  regmap[REG_DEBUG_MODE] = (regmap_t)(&debug_mode);
  regmap[REG_PLIM] = (regmap_t)(&Plim);
  regmap[REG_PLIM_GUARDBAND] = (regmap_t)(&Plim_guardband);
  regmap[REG_PLIM_WARN] = (regmap_t)(&Plim_warn);
  regmap[REG_ILIM_5V] = (regmap_t)(&ILIM_5V);
  regmap[REG_ILIM_N12V] = (regmap_t)(&ILIM_N12V);
  regmap[REG_ILIM_12V] = (regmap_t)(&ILIM_12V);
  regmap[REG_NEGO_VBUS] = (regmap_t)(&nego_vbus);
  regmap[REG_VBUS_ILIM] = (regmap_t)(&vbus_ilim);
  regmap[REG_VOLTS_5V] = (regmap_t)(&volts_5v);
  regmap[REG_VOLTS_N12V] = (regmap_t)(&volts_n12v);
  regmap[REG_VOLTS_12V] = (regmap_t)(&volts_p12v);
  regmap[REG_VOLTS_VBUS] = (regmap_t)(&vbus_meas);
  regmap[REG_CURR_5V] = (regmap_t)(&amps_out_5v);
  regmap[REG_CURR_N12V] = (regmap_t)(&amps_out_n12v);
  regmap[REG_CURR_12V] = (regmap_t)(&amps_out_p12v);
  regmap[REG_CURR_VBUS] = (regmap_t)(&amps_in_vbus);
  regmap[REG_POWER_VBUS] = (regmap_t)(&pwr_vbus);
  regmap[REG_BUCK_BST_5VIN_C0] = (regmap_t)(eff_coefs_buck_boost_5vin);
  regmap[REG_BUCK_BST_5VIN_C1] = (regmap_t)(eff_coefs_buck_boost_5vin + 1);
  regmap[REG_BUCK_BST_9VIN_C0] = (regmap_t)(eff_coefs_buck_boost_9vin);
  regmap[REG_BUCK_BST_9VIN_C1] = (regmap_t)(eff_coefs_buck_boost_9vin + 1);
  regmap[REG_BUCK_BST_15VIN_C0] = (regmap_t)(eff_coefs_buck_boost_15vin);
  regmap[REG_BUCK_BST_15VIN_C1] = (regmap_t)(eff_coefs_buck_boost_15vin + 1);
  regmap[REG_BUCK_BST_20VIN_C0] = (regmap_t)(eff_coefs_buck_boost_20vin);
  regmap[REG_BUCK_BST_20VIN_C1] = (regmap_t)(eff_coefs_buck_boost_20vin + 1);
  regmap[REG_5V_CONV_C0] = (regmap_t)(eff_coefs_5v_conv);
  regmap[REG_5V_CONV_C1] = (regmap_t)(eff_coefs_5v_conv + 1);
  regmap[REG_IBB_5VIN_C0] = (regmap_t)(&eff_coefs_ibb_5vin);
  regmap[REG_IBB_5VIN_C1] = (regmap_t)(&eff_coefs_ibb_5vin + 1);
  regmap[REG_IBB_9VIN_C0] = (regmap_t)(&eff_coefs_ibb_9vin);
  regmap[REG_IBB_9VIN_C1] = (regmap_t)(&eff_coefs_ibb_9vin + 1);
  regmap[REG_IBB_15VIN_C0] = (regmap_t)(&eff_coefs_ibb_15vin);
  regmap[REG_IBB_15VIN_C1] = (regmap_t)(&eff_coefs_ibb_15vin + 1);
  regmap[REG_IBB_20VIN_C0] = (regmap_t)(&eff_coefs_ibb_20vin);
  regmap[REG_IBB_20VIN_C1] = (regmap_t)(&eff_coefs_ibb_20vin + 1);
  regmap[REG_DISP_PRESENT] = (regmap_t)(&disp_present);
  
  writeable[REG_DEBUG_MODE] = true;
  writeable[REG_PLIM] = true;
  writeable[REG_PLIM_GUARDBAND] = true;
  writeable[REG_PLIM_WARN] = true;
  writeable[REG_ILIM_5V] = true;
  writeable[REG_ILIM_N12V] = true;
  writeable[REG_ILIM_12V] = true;
  writeable[REG_NEGO_VBUS] = true;
  writeable[REG_VBUS_ILIM] = true;

  writeable[REG_BUCK_BST_5VIN_C0] = true;
  writeable[REG_BUCK_BST_5VIN_C1] = true;
  writeable[REG_BUCK_BST_9VIN_C0] = true;
  writeable[REG_BUCK_BST_9VIN_C1] = true;
  writeable[REG_BUCK_BST_15VIN_C0] = true;
  writeable[REG_BUCK_BST_15VIN_C1] = true;
  writeable[REG_BUCK_BST_20VIN_C0] = true;
  writeable[REG_BUCK_BST_20VIN_C1] = true;
  writeable[REG_5V_CONV_C0] = true;
  writeable[REG_5V_CONV_C1] = true;
  writeable[REG_IBB_5VIN_C0] = true;
  writeable[REG_IBB_5VIN_C1] = true;
  writeable[REG_IBB_9VIN_C0] = true;
  writeable[REG_IBB_9VIN_C1] = true;
  writeable[REG_IBB_15VIN_C0] = true;
  writeable[REG_IBB_15VIN_C1] = true;
  writeable[REG_IBB_20VIN_C0] = true;
  writeable[REG_IBB_20VIN_C1] = true;
}


void read_eff_coefs_from_flash(void) {
  uint32_t flash_flag = 0;
  ee_read(0, 4, (uint8_t*)&flash_flag);
  if (flash_flag == 0xA5A5A5A5) {
    uint32_t ee_addr = 4;
    for (int regind = REG_BUCK_BST_5VIN_C0; regind <= REG_IBB_20VIN_C1; regind++) {
      ee_read(ee_addr, REG_MAP_DATA_SIZE, (uint8_t*)(regmap[regind]));
      ee_addr += 4;
    }
  }
}


void save_eff_coefs_to_flash(void) {
  uint32_t flash_flag = 0xA5A5A5A5;
  uint32_t ee_addr = 0;
  ee_write(ee_addr, 4, (uint8_t*)(&flash_flag));
  for (int regind = REG_BUCK_BST_5VIN_C0; regind <= REG_IBB_20VIN_C1; regind++) {
    ee_write(ee_addr, REG_MAP_DATA_SIZE, (uint8_t*)(regmap[regind]));
    ee_addr += 4;
  }
}

/**
 * Parses the string in rx_buf and does things based on that.
*/
void process_dev_packet(){
  if(memcmp(rx_buf, "PRINT_SRC_PDOS", rx_size) == 0){
    // printf the source PDOs
    debug_mode = DEBUG_NORMAL;
    Print_PDO_FROM_SRC(0); // usb port ID is always going to be 0 for our design
    debug_mode = DEBUG_DEVELOPER;
  }else{
    if(char_in_buffer(rx_buf, '=', rx_size)){
      // something is being set
      uint8_t* word = (uint8_t*)malloc(rx_size);
      char* token = strtok(rx_buf, "=");
      while(token != NULL){
        strstrip(token, word); // word now holds the CMD/KEY

        token = strtok(NULL, "\0"); // get the VALUE

        if (strcmp(word, "REQ_VOLTAGE") == 0){
          strstrip(token, word);
          req_voltage = atof(word);
          req_number = 0;
        }else if(strcmp(word, "REQ_CURRENT") == 0){
          strstrip(token, word);
          req_current = atof(word);
          req_number = 0;
        }else if(strcmp(word, "REQ_NUMBER") == 0){
          strstrip(token, word);
          req_number = atoi(word);
        }
      }
      free(word);
    }else if(memcmp(rx_buf, "NEGO_NEW_PDO", rx_size) == 0){
      debug_mode = DEBUG_NORMAL;
      if (req_number <= 0){
        printf("Negotiating PDO of %.1fV @ %.2fA...", req_voltage, req_current);
        int PDO_V = (int)(req_voltage*1000.0 + 0.5);
        int PDO_I = (int)(req_current*1000.0 + 0.5);
        Update_PDO(0, 2, PDO_V, PDO_I);
        Update_Valid_PDO_Number(0, 2);
      }else{
        printf("Negotiating PDO #%d...", req_number);
        Request_SRC_PDO_NUMBER(0, req_number);
      }
      Send_Soft_reset_Message(0); // forces re-negotiation
      HAL_Delay(200);
      connection_flag[0] = 1;
      Read_RDO(0);
      Print_RDO(0);
      check_on_pdo_voltage(0);
      debug_mode = DEBUG_DEVELOPER;
    }
  }
}

/**
 * Checks what voltage has been negotiated and lights LEDs accordingly.
 * Also pulses disable if voltage is around 15V.
*/
void check_on_pdo_voltage(int usb_port){
  float nego_voltage = (float)PDO_FROM_SRC[usb_port][Nego_RDO[usb_port].b.Object_Pos - 1].fix.Voltage/20.0;
  if (nego_voltage >= 12.0 && nego_voltage <= 16.0){
    printf("Negotiated voltage is %.2fV - Pulsing main converter off.", nego_voltage);
    DISABLE_PRIMARY_12V();
    for(int i=0; i<25; i++)
    {
      __NOP();
    }
    ENABLE_PRIMARY_12V();
  }
  update_vbus_leds(nego_voltage);
}


/**
 * Light the correct VBUS indicator LED depending on the vbus argument
*/
void update_vbus_leds(float vbus){

  HAL_GPIO_WritePin(PD5V_GPIO_Port, PD5V_Pin, 0);
  HAL_GPIO_WritePin(PD9V_GPIO_Port, PD9V_Pin, 0);
  HAL_GPIO_WritePin(PD15V_GPIO_Port, PD15V_Pin, 0);
  HAL_GPIO_WritePin(PD20V_GPIO_Port, PD20V_Pin, 0);

  if(WITHIN_RANGE(vbus, 5.0, 1.0))
  {
    HAL_GPIO_WritePin(PD5V_GPIO_Port, PD5V_Pin, 1);
  }
  else if(WITHIN_RANGE(vbus, 9.0, 3.0))
  {
    HAL_GPIO_WritePin(PD9V_GPIO_Port, PD9V_Pin, 1);
  }
  else if (WITHIN_RANGE(vbus, 15.0, 3.0))
  {
    HAL_GPIO_WritePin(PD15V_GPIO_Port, PD15V_Pin, 1);
  }
  else if (WITHIN_RANGE(vbus, 20.0, 2.0))
  {
    HAL_GPIO_WritePin(PD20V_GPIO_Port, PD20V_Pin, 1);
  }
}


/**
 * Delay some number of microseconds
*/
void delay_us(uint32_t delay){
  uint32_t start_us = micros();
  uint32_t tstamp = start_us;
  while (tstamp - start_us < delay){
    tstamp = micros();
    if (tstamp < start_us){
      uint32_t elapsed = 0xFFFFFFFF - start_us; // how many us passed before wrap around
      start_us = 0;
      delay -= elapsed;
    }
  }
}


/**
 * If on_or_off != 0, turn on the outputs in sequence.
 * else, turn off all 3 outputs at once
*/
void set_output_state(uint8_t on_or_off){
  if(on_or_off){
    // sequence the turn on of the rails
    ENABLE_PRIMARY_12V();
    ENABLE_N12_SUPPLY();
    int timeout = 0;
    GPIO_PinState pgood = HAL_GPIO_ReadPin(PGOOD_12V_GPIO_Port, PGOOD_12V_Pin);
    while (pgood == GPIO_PIN_RESET && timeout < ENABLE_12V_TIMEOUT_US) {
      delay_us(10);
      timeout += 10;
      pgood = HAL_GPIO_ReadPin(PGOOD_12V_GPIO_Port, PGOOD_12V_Pin);
    }
    ENABLE_5P0_SUPPLY();
  }else{
    DISABLE_N12_SUPPLY();
    DISABLE_5PO_SUPPLY();
    DISABLE_PRIMARY_12V();
  }
}

/**
 * Get a count of microseconds since program start. Will wrap around after some time.
*/
uint32_t micros(){
  uint32_t subdiv = (htim1.Instance->CNT << 5) / htim1.Instance->CNT;
  uint32_t new_micros = (micros_count_by_32 << 5) + subdiv;
  if (new_micros < micros_count){
    // micros_count_by_32 << 5 has wrapped around
    micros_count_by_32 = micros_count_by_32 - 0x07FFFFFF;
    new_micros = (micros_count_by_32 << 5) + subdiv;
  }
  return new_micros;
}

/**
 * @brief compute a polynomial from the given coefficients and x input.
 * 
 * @return y value of the polynomial
 */
float compute_polynom(float* coefs, int coef_count, float x) {
  float retval = 0.0;
  for (int coef_order = 0; coef_order < coef_count; coef_order++) {
    retval += coefs[coef_order] * powf(x, 1.0*coef_order);
  }
  return retval;
}


void enter_sleep_mode(void) {
  HAL_SuspendTick();
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFE);
  HAL_ResumeTick();
}

/**
 * Put the specified rail into hiccup and set all the necessary globals for 
 * recovering.
 */
void hiccup_rail(int rail_id) {
  switch(rail_id) {
    case RAIL_P12V:{
      DISABLE_PRIMARY_12V();
      Hiccup_p12v_flag = 1;
      hiccup_p12v_ts = HAL_GetTick();
      break;
    }
    case RAIL_N12V:{
      DISABLE_N12_SUPPLY();
      Hiccup_n12v_flag = 1;
      hiccup_n12v_ts = HAL_GetTick();
      break;
    }
    case RAIL_5V:{
      DISABLE_5PO_SUPPLY();
      Hiccup_5v_flag = 1;
      hiccup_5v_ts = HAL_GetTick();
      break;
    }
    default:
      return;
  }
}


/**
 * Callback for UART DMA reception
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if (uart_state == UART_STATE_RX_HDDR) {
    if (rx_buf[0] >= CMD_COUNT) {
      /* well this isn't right... */
      HAL_UART_Receive_DMA(&huart1, rx_buf, UART_HDDR_SIZE);
      return;
    }
    uint8_t aux_value = rx_buf[1];
    uint16_t body_len = rx_buf[2] | (rx_buf[3] << 8);
    if (body_len > 0) {
      if (body_len + UART_HDDR_SIZE > UART_RX_BUF_SIZE) {
        body_len = UART_RX_BUF_SIZE - UART_HDDR_SIZE;
      }
      uart_state = UART_STATE_RX_BODY;
      HAL_UART_Receive_DMA(&huart1, rx_buf + UART_HDDR_SIZE, body_len);
    } else {
      uart_state = UART_STATE_TX_PACKET; /* signals to the main loop that it should prepare & transmit a response packet */
    }
  }
  else if (uart_state == UART_STATE_RX_BODY) {
    uart_state = UART_STATE_TX_PACKET; /* signals to the main loop that it should prepare & transmit a response packet */
  }
  else {
    /* something's wrong, just restart the reception */
    uart_state = UART_STATE_RX_HDDR;
    HAL_UART_Receive_DMA(&huart1, rx_buf, UART_HDDR_SIZE);
  }
  
}

/**
 * Handle hardware I2C comms
*/
uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr){
  uint8_t *data;
  static uint8_t buffer[128];
  static uint8_t buf_ind;
  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;
      while(arg_int > 0){
        buffer[buf_ind++] = *data;
        data++;
        arg_int--;
      }
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
      if (disp_present != 0) {
        HAL_StatusTypeDef i2c_stat = HAL_I2C_Master_Transmit(&hi2c2, u8x8_GetI2CAddress(u8x8), buffer, buf_ind, 100);
        disp_present = (uint32_t)(i2c_stat == HAL_OK);
      }
      buf_ind = 0;
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

void disable_ina_ints(){
  ina_alerts_enabled = 0;
}

void enable_ina_ints(){
  ina_alerts_enabled = 1;
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
