/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h> // for abs()
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "comms.h"
#include "eeprom.h"
#include "sensorcoms_single.h"

#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
  #include "hd44780.h"
#endif


// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"      /* Model's header file */
#include "rtwtypes.h"

RT_MODEL rtM_Left_;               /* Real-time model */
RT_MODEL rtM_Right_;              /* Real-time model */
RT_MODEL *const rtM_Left    = &rtM_Left_;
RT_MODEL *const rtM_Right   = &rtM_Right_;

extern P rtP_Left;                /* Block parameters (auto storage) */
DW    rtDW_Left;                  /* Observable states */
ExtU  rtU_Left;                   /* External inputs */
ExtY  rtY_Left;                   /* External outputs */

P     rtP_Right;                  /* Block parameters (auto storage) */
DW    rtDW_Right;                 /* Observable states */
ExtU  rtU_Right;                  /* External inputs */
ExtY  rtY_Right;                  /* External outputs */

extern uint8_t errCode_Left;      /* Global variable to handle Motor error codes */
extern uint8_t errCode_Right;     /* Global variable to handle Motor error codes */
// ###############################################################################

void SystemClock_Config(void);
void poweroff(void);

extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
  LCD_PCF8574_HandleTypeDef lcd;
#endif
extern I2C_HandleTypeDef hi2c2;
#if defined(CONTROL_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART2) || defined(DEBUG_SERIAL_USART2) \
 || defined(CONTROL_SERIAL_USART3) || defined(FEEDBACK_SERIAL_USART3) || defined(DEBUG_SERIAL_USART3) 
  extern UART_HandleTypeDef huart2;
  extern UART_HandleTypeDef huart3;
#endif

#if defined(DEBUG_I2C_LCD) || defined(SUPPORT_LCD)
  extern uint8_t LCDerrorFlag;
#endif

#ifdef VARIANT_TRANSPOTTER
  uint8_t nunchuk_connected = 0;
  float steering;
  int feedforward;

  void saveConfig(void);

  /* Virtual address defined by the user: 0xFFFF value is prohibited */
  uint16_t VirtAddVarTab[NB_OF_VAR] = {0x1337};
  uint16_t VarDataTab[NB_OF_VAR] = {0};
  uint16_t VarValue = 0;
  uint16_t saveValue = 0;

  uint16_t counter = 0;
#else
  uint8_t nunchuk_connected = 1;
	uint16_t VirtAddVarTab[NB_OF_VAR] = {0x1300}; 	// Dummy address to avoid warnings
#endif

#if defined(CONTROL_ADC) && defined(ADC_PROTECT_ENA)
static int16_t timeoutCntADC   = 0;  // Timeout counter for ADC Protection
#endif
static uint8_t timeoutFlagADC  = 0;  // Timeout Flag for for ADC Protection: 0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)

#if defined(CONTROL_SERIAL_USART2) || defined(CONTROL_SERIAL_USART3) //|| defined(SENSOR_SERIAL_USART3)
  #ifdef CONTROL_IBUS
    static uint16_t ibus_chksum;
    static uint16_t ibus_captured_value[IBUS_NUM_CHANNELS];
    
    typedef struct{
      uint8_t  start;
      uint8_t  type; 
      uint8_t  channels[IBUS_NUM_CHANNELS*2];
      uint8_t  checksuml;
      uint8_t  checksumh;    
    } Serialcommand;
  // #elif defined(SENSOR_SERIAL_USART3)
  //   typedef struct{
  //     int16_t  speed;
  //     int16_t  speed2; 
  //     int8_t   step;
  //     int16_t   acc;
  //     int16_t   acc2;
  //     int8_t  rot;
  //     int8_t  rot2;    
  //   } Serialcommand;
  #else
    typedef struct{
      uint16_t  start; 
      int16_t   steer;
      int16_t   speed;
      uint16_t  checksum;
    } Serialcommand;
  #endif
static volatile Serialcommand command;
static int16_t timeoutCntSerial   = 0;  // Timeout counter for Rx Serial command
#endif
static uint8_t timeoutFlagSerial  = 0;  // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

#if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
typedef struct{
  uint16_t  start;
  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   speedR;
  int16_t   speedL;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  checksum;
} SerialFeedback;
static SerialFeedback Feedback;
#endif

#ifdef SUPPORT_BUTTONS
static uint8_t button1, button2;
#endif

uint8_t ctrlModReqRaw = CTRL_MOD_REQ;
uint8_t ctrlModReq    = CTRL_MOD_REQ;   // Final control mode request
static int16_t    speed;                // local variable for speed. -1000 to 1000
#ifndef VARIANT_TRANSPOTTER
	static int        cmd1;               // normalized input value. -1000 to 1000
	static int        cmd2;               // normalized input value. -1000 to 1000
  static int16_t  steer = 0;                // local variable for steering. -1000 to 1000
  static int16_t  steerRateFixdt;       // local fixed-point variable for steering rate limiter
  static int16_t  speedRateFixdt;       // local fixed-point variable for speed rate limiter
  static int32_t  steerFixdt;           // local fixed-point variable for steering low-pass filter
  static int32_t  speedFixdt;           // local fixed-point variable for speed low-pass filter
#endif
static MultipleTap MultipleTapBreak;  // define multiple tap functionality for the Break pedal

static int16_t    speedAvg;             // average measured speed
static int16_t    speedAvgAbs;          // average measured speed in absolute

extern volatile int pwml;               // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;               // global variable for pwm right. -1000 to 1000

extern uint8_t buzzerFreq;              // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern;           // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable;                  // global variable for motor enable

extern volatile uint32_t timeout;       // global variable for timeout
extern int16_t batVoltage;              // global variable for battery voltage

static uint32_t inactivity_timeout_counter;
static uint32_t main_loop_counter;

extern uint8_t nunchuk_data[6];
#ifdef CONTROL_PPM
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif


void poweroff(void) {
  //  if (abs(speed) < 20) {  // wait for the speed to drop, then shut down -> this is commented out for SAFETY reasons
        buzzerPattern = 0;
        enable = 0;
        consoleLog("-- Motors disabled --\r\n");
        for (int i = 0; i < 8; i++) {
            buzzerFreq = (uint8_t)i;
            HAL_Delay(100);
        }
        HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_RESET);
        while(1) {}
  //  }
}

void init_matlab() {
// Matlab Init
// ###############################################################################
  
  /* Set BLDC controller parameters */ 
  rtP_Left.b_selPhaABCurrMeas   = 1;            // Left motor measured current phases = {iA, iB} -> do NOT change
  rtP_Left.z_ctrlTypSel         = CTRL_TYP_SEL;
  rtP_Left.b_diagEna            = DIAG_ENA; 
  rtP_Left.i_max                = (I_MOT_MAX * A2BIT_CONV) << 4;        // fixdt(1,16,4)
  rtP_Left.n_max                = N_MOT_MAX << 4;                       // fixdt(1,16,4)
  rtP_Left.b_fieldWeakEna       = FIELD_WEAK_ENA; 
  rtP_Left.id_fieldWeakMax      = (FIELD_WEAK_MAX * A2BIT_CONV) << 4;   // fixdt(1,16,4)
  rtP_Left.a_phaAdvMax          = PHASE_ADV_MAX << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakHi        = FIELD_WEAK_HI << 4;                   // fixdt(1,16,4)
  rtP_Left.r_fieldWeakLo        = FIELD_WEAK_LO << 4;                   // fixdt(1,16,4)

  rtP_Right                     = rtP_Left;     // Copy the Left motor parameters to the Right motor parameters
  rtP_Right.b_selPhaABCurrMeas  = 0;            // Left motor measured current phases = {iB, iC} -> do NOT change

  /* Pack LEFT motor data into RTM */
  rtM_Left->defaultParam        = &rtP_Left;
  rtM_Left->dwork               = &rtDW_Left;
  rtM_Left->inputs              = &rtU_Left;
  rtM_Left->outputs             = &rtY_Left;

  /* Pack RIGHT motor data into RTM */
  rtM_Right->defaultParam       = &rtP_Right;
  rtM_Right->dwork              = &rtDW_Right;
  rtM_Right->inputs             = &rtU_Right;
  rtM_Right->outputs            = &rtY_Right;

  /* Initialize BLDC controllers */
  BLDC_controller_initialize(rtM_Left);
  BLDC_controller_initialize(rtM_Right);

// ###############################################################################
}

int main(void) {

  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  UART2_Init();
  HAL_Delay(50);
  consoleLog("UART2 init done\r\n");
 
  UART3_Init();
  HAL_Delay(50);
  consoleLog("UART3 init done\r\n");

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  init_matlab();

  sensor_init(); // sets internal structs to 0
  consoleLog("sensor init done\r\n");


  for (int i = 8; i >= 0; i--) {
    buzzerFreq = (uint8_t)i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
 

  int16_t lastCmd = 0;
  float lastSpeed = 0;
  float speedL = 0, speedR = 0;
  float accel = 0;
  float Kd, Ki, Kp;
  int32_t board_temp_adcFixdt = adc_buffer.temp << 20;  // Fixed-point filter output initialized with current ADC converted to fixed-point
  int16_t board_temp_adcFilt  = adc_buffer.temp;
  int16_t board_temp_deg_c;

  consoleLog("Enter main loop\r\n");

  while(1) {
    HAL_Delay(DELAY_IN_MAIN_LOOP); //delay in ms

    
    #ifdef CONTROL_ADC
      // ADC values range: 0-4095, see ADC-calibration in config.h
      #ifdef ADC1_MID_POT
        cmd1 = CLAMP((adc_buffer.l_tx2 - ADC1_MID) * INPUT_MAX / (ADC1_MAX - ADC1_MID), 0, INPUT_MAX) 
              -CLAMP((ADC1_MID - adc_buffer.l_tx2) * INPUT_MAX / (ADC1_MID - ADC1_MIN), 0, INPUT_MAX);    // ADC1        
      #else
        cmd1 = CLAMP((adc_buffer.l_tx2 - ADC1_MIN) * INPUT_MAX / (ADC1_MAX - ADC1_MIN), 0, INPUT_MAX);    // ADC1
      #endif

      #ifdef ADC2_MID_POT
        cmd2 = CLAMP((adc_buffer.l_rx2 - ADC2_MID) * INPUT_MAX / (ADC2_MAX - ADC2_MID), 0, INPUT_MAX)  
              -CLAMP((ADC2_MID - adc_buffer.l_rx2) * INPUT_MAX / (ADC2_MID - ADC2_MIN), 0, INPUT_MAX);    // ADC2        
      #else
        cmd2 = CLAMP((adc_buffer.l_rx2 - ADC2_MIN) * INPUT_MAX / (ADC2_MAX - ADC2_MIN), 0, INPUT_MAX);    // ADC2
      #endif

      #ifdef ADC_PROTECT_ENA
        if (adc_buffer.l_tx2 >= (ADC1_MIN - ADC_PROTECT_THRESH) && adc_buffer.l_tx2 <= (ADC1_MAX + ADC_PROTECT_THRESH) && 
            adc_buffer.l_rx2 >= (ADC2_MIN - ADC_PROTECT_THRESH) && adc_buffer.l_rx2 <= (ADC2_MAX + ADC_PROTECT_THRESH)) {
          if (timeoutFlagADC) {                         // Check for previous timeout flag  
            if (timeoutCntADC-- <= 0)                   // Timeout de-qualification
              timeoutFlagADC  = 0;                      // Timeout flag cleared           
          } else {
            timeoutCntADC     = 0;                      // Reset the timeout counter         
          }
        } else {
          if (timeoutCntADC++ >= ADC_PROTECT_TIMEOUT) { // Timeout qualification
            timeoutFlagADC    = 1;                      // Timeout detected
            timeoutCntADC     = ADC_PROTECT_TIMEOUT;    // Limit timout counter value
          }
        }

        if (timeoutFlagADC) {                           // In case of timeout bring the system to a Safe State
          ctrlModReq  = 0;                              // OPEN_MODE request. This will bring the motor power to 0 in a controlled way
          cmd1        = 0;
          cmd2        = 0;
        } else {
          ctrlModReq  = ctrlModReqRaw;                  // Follow the Mode request
        }
      #endif

      // use ADCs as button inputs:
			#ifdef SUPPORT_BUTTONS
				button1 = (uint8_t)(adc_buffer.l_tx2 > 2000);  // ADC1
				button2 = (uint8_t)(adc_buffer.l_rx2 > 2000);  // ADC2
			#endif

      timeout = 0;
    #endif

    #if defined CONTROL_SERIAL_USART2 || defined CONTROL_SERIAL_USART3

      // Handle received data validity, timeout and fix out-of-sync if necessary
      #ifdef CONTROL_IBUS
        ibus_chksum = 0xFFFF - IBUS_LENGTH - IBUS_COMMAND;
        for (uint8_t i = 0; i < (IBUS_NUM_CHANNELS * 2); i++) {
          ibus_chksum -= command.channels[i];
        }
        if (command.start == IBUS_LENGTH && command.type == IBUS_COMMAND && ibus_chksum == (uint16_t)((command.checksumh << 8) + command.checksuml)) {
          if (timeoutFlagSerial) {                      // Check for previous timeout flag  
            if (timeoutCntSerial-- <= 0)                // Timeout de-qualification
              timeoutFlagSerial = 0;                    // Timeout flag cleared           
          } else {         
            for (uint8_t i = 0; i < (IBUS_NUM_CHANNELS * 2); i+=2) {
              ibus_captured_value[(i/2)] = CLAMP(command.channels[i] + (command.channels[i+1] << 8) - 1000, 0, INPUT_MAX); // 1000-2000 -> 0-1000
            }
            cmd1              = CLAMP((ibus_captured_value[0] - INPUT_MID) * 2, INPUT_MIN, INPUT_MAX);
            cmd2              = CLAMP((ibus_captured_value[1] - INPUT_MID) * 2, INPUT_MIN, INPUT_MAX);
            command.start     = 0xFF;                   // Change the Start Frame for timeout detection in the next cycle
            timeoutCntSerial  = 0;                      // Reset the timeout counter
          }
        } else {
          if (timeoutCntSerial++ >= SERIAL_TIMEOUT) {   // Timeout qualification
            timeoutFlagSerial = 1;                      // Timeout detected
            timeoutCntSerial  = SERIAL_TIMEOUT;         // Limit timout counter value
          }
          // Check periodically the received Start Frame. If it is NOT OK, most probably we are out-of-sync. Try to re-sync by reseting the DMA
          if (main_loop_counter % 25 == 0 && command.start != IBUS_LENGTH && command.start != 0xFF) {
            HAL_UART_DMAStop(&huart);                
            HAL_UART_Receive_DMA(&huart, (uint8_t *)&command, sizeof(command));
            command.start = 0xFF;                       // Change the Start Frame to avoid entering again here if no data is received
          }
        }  
      #else
        if (command.start == START_FRAME && command.checksum == (uint16_t)(command.start ^ command.steer ^ command.speed)) {
          if (timeoutFlagSerial) {                      // Check for previous timeout flag  
            if (timeoutCntSerial-- <= 0)                // Timeout de-qualification
              timeoutFlagSerial = 0;                    // Timeout flag cleared           
          } else {
            cmd1              = CLAMP((int16_t)command.steer, INPUT_MIN, INPUT_MAX);
            cmd2              = CLAMP((int16_t)command.speed, INPUT_MIN, INPUT_MAX);
            command.start     = 0xFFFF;                 // Change the Start Frame for timeout detection in the next cycle
            timeoutCntSerial  = 0;                      // Reset the timeout counter         
          }
        } else {
          if (timeoutCntSerial++ >= SERIAL_TIMEOUT) {   // Timeout qualification
            timeoutFlagSerial = 1;                      // Timeout detected
            timeoutCntSerial  = SERIAL_TIMEOUT;         // Limit timout counter value
          }
          // Check periodically the received Start Frame. If it is NOT OK, most probably we are out-of-sync. Try to re-sync by reseting the DMA
          if (main_loop_counter % 25 == 0 && command.start != START_FRAME && command.start != 0xFFFF) {
            HAL_UART_DMAStop(&huart);                
            HAL_UART_Receive_DMA(&huart, (uint8_t *)&command, sizeof(command));
            command.start = 0xFFFF;                     // Change the Start Frame to avoid entering again here if no data is received
          }
        }
      #endif     

      if (timeoutFlagSerial) {                          // In case of timeout bring the system to a Safe State
        ctrlModReq  = 0;                                // OPEN_MODE request. This will bring the motor power to 0 in a controlled way
        cmd1        = 0;
        cmd2        = 0;
      } else {
        ctrlModReq  = ctrlModReqRaw;                    // Follow the Mode request
      }
      timeout = 0;

    #endif

    #ifdef VARIANT_HOVERBOARD
    sensor_read_data();  
    //consoleLog("# ");  
    if (sensor_data.read_timeout == 0)
      timeout ++;
    else
      timeout = 0;

    cmd1 = 0; //CLAMP((sensor_data.complete.Angle/3), INPUT_MIN, INPUT_MAX); // steer
    if (sensor_data.complete.AA_55 == 85) // foot on sensor
    {
      //int scaling = sensor_data.complete.Angle >> 6;
      //cmd2 = CLAMP(((scaling*scaling*scaling) >> 5) + (sensor_data.complete.Angle >> 1), INPUT_MIN, INPUT_MAX);  //speed
      cmd2 = CLAMP(sensor_data.complete.Angle, INPUT_MIN, INPUT_MAX);  //speed
    }
    else
    {
      cmd2 = 0;
    }
    #endif

    // Calculate measured average speed. The minus sign (-) is beacause motors spin in opposite directions
    #if   !defined(INVERT_L_DIRECTION) && !defined(INVERT_R_DIRECTION)
      speedAvg    = ( rtY_Left.n_mot - rtY_Right.n_mot) / 2;
    #elif !defined(INVERT_L_DIRECTION) &&  defined(INVERT_R_DIRECTION)
      speedAvg    = ( rtY_Left.n_mot + rtY_Right.n_mot) / 2;
    #elif  defined(INVERT_L_DIRECTION) && !defined(INVERT_R_DIRECTION)
      speedAvg    = (-rtY_Left.n_mot - rtY_Right.n_mot) / 2;
    #elif  defined(INVERT_L_DIRECTION) &&  defined(INVERT_R_DIRECTION)
      speedAvg    = (-rtY_Left.n_mot + rtY_Right.n_mot) / 2;
    #endif

    // Handle the case when SPEED_COEFFICIENT sign is negative (which is when most significant bit is 1)
    if ((SPEED_COEFFICIENT & (1 << 16)) >> 16) {
      speedAvg    = -speedAvg;
    } 
    speedAvgAbs   = abs(speedAvg);

    #ifndef VARIANT_TRANSPOTTER
      // ####### MOTOR ENABLING: only if no errors and foot switch pressed #######
      if (enable == 0 && (!errCode_Left && !errCode_Right) && (sensor_data.complete.AA_55 == 85))
      {
        // if we werent moving before, only turn on for low angles
        if (/*speedAvgAbs > 10 ||*/ (cmd2 > -90 && cmd2 < 90))
        {
          shortBeep(6);                     // make 2 beeps indicating the motor enable
          HAL_Delay(100);
          shortBeep(4); 
          consoleLog("enabeling motors\r\n");
          enable = 1;                       // enable motors
        }        
      } 

      // ####### LOW-PASS FILTER #######
      //rateLimiter16(cmd1, RATE, &steerRateFixdt);
      //rateLimiter16(cmd2, RATE, &speedRateFixdt);
      //filtLowPass32(steerRateFixdt >> 4, FILTER, &steerFixdt);
      //filtLowPass32(speedRateFixdt >> 4, FILTER, &speedFixdt);
      //steer = 0; //= (int16_t)(steerFixdt >> 20);  // convert fixed-point to integer
      //speed = (int16_t)(speedFixdt >> 20);  // convert fixed-point to integer    


      // ####### MIXER #######
      // speedR = CLAMP((int)(speed * SPEED_COEFFICIENT -  steer * STEER_COEFFICIENT), -1000, 1000);
      // speedL = CLAMP((int)(speed * SPEED_COEFFICIENT +  steer * STEER_COEFFICIENT), -1000, 1000);
      //mixerFcn(speed << 4, steer << 4, &speedR, &speedL);   // This function implements the equations above
      
      // clamp both motors to same speed
      int8_t pOnE = 0;
      if (enable == 1) {
      if (ctrlModReq==2) { // for speed mode (doesnt work well)
         Kp = 0.035;  // critical at 0.7
         Ki = 0.06;  // ncritcal at 0.08
         Kd = 0.01;
      } else // for voltage mode
      {
         Kp = 0.35; 
         Ki = 0.6;  
         Kd = 0.1;
      }
      

      int16_t input = -cmd2;
      int16_t error = cmd2;
      int16_t dinput = input - lastCmd;
      accel = Ki * error;

      if(!pOnE) accel-= Kp * dinput; 

      float output;
      if(pOnE) output = Kp * error; 
      else output = 0;

      output += accel - Kd * dinput;      
      speedL = output;
      speedR = speedL;

      } else {       
        accel = 0;   // reset PID
        speedL = 0;
        speedR = speedL;

      }

      #ifdef ADDITIONAL_CODE
        ADDITIONAL_CODE;
      #endif


      // ####### SET OUTPUTS (if the target change is less than +/- 50) #######
      if (/*(speedL > lastSpeedL-50 && speedL < lastSpeedL+50) && (speedR > lastSpeedR-50 && speedR < lastSpeedR+50) &&*/ timeout < TIMEOUT) {
        #ifdef INVERT_R_DIRECTION
          pwmr = round(speedR);
        #else
          pwmr = -speedR;
        #endif
        #ifdef INVERT_L_DIRECTION
          pwml = round(-speedL);
        #else
          pwml = speedL;
        #endif
      }
      else {
        char t[200];
        sprintf(t, "Speed change rate to high, not setting new speed %f from old speed %f\r\n", speedL, lastSpeed);
        consoleLogLowPrio(t);
      }
    #endif
    lastCmd = cmd2;
    lastSpeed = speedL;
    //lastSpeedR = speedR;


    // ####### CALC BOARD TEMPERATURE #######
    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
    board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 20);  // convert fixed-point to integer
    board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

    if (main_loop_counter % 25 == 0) {    // Send data periodically
      // ####### DEBUG SERIAL OUT #######      
      setScopeChannel(0, (int16_t)sensor_data.complete.Angle);        // 1: Angle
      setScopeChannel(1, (int16_t)sensor_data.complete.AA_55);        // 2: Step
      setScopeChannel(2, (int16_t)speedR);                    // 3: output command: [-1000, 1000]
      setScopeChannel(3, (int16_t)speedL);                    // 4: output command: [-1000, 1000]
      setScopeChannel(4, (int16_t)adc_buffer.batt1);          // 5: for battery voltage calibration
      setScopeChannel(5, (int16_t)(batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC)); // 6: for verifying battery voltage calibration
      setScopeChannel(6, (int16_t)board_temp_adcFilt);        // 7: for board temperature calibration
      setScopeChannel(7, (int16_t)board_temp_deg_c);          // 8: for verifying board temperature calibration
      consoleScope();         
    }    

    HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    // ####### POWEROFF BY POWER-BUTTON #######
    if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      consoleLog("Power button pressed\r\n");
      enable = 0;                                             // disable motors
      while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {}    // wait until button is released
      if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {               // do not power off after software reset (from a programmer/debugger)
        __HAL_RCC_CLEAR_RESET_FLAGS();                        // clear reset flags
      } else {
        poweroff();                                           // release power-latch
      }
    }


    // ####### BEEP AND EMERGENCY POWEROFF #######
    if (sensor_data.complete.AA_55 != 85) {  //foot not on sensor
      if (enable == 1){
        consoleLog("Foot raised: disabeling motors\r\n");
        enable        = 0;
        //cmd1 = cmd2 = 0;
        shortBeep(4);                     // make 2 beeps indicating the motor disable
        HAL_Delay(100);
        shortBeep(6); 
      }
    } 
    if (sensor_data.complete.Angle > 4000 || sensor_data.complete.Angle < -4000) {  //wrong way around or too steep
      if (enable == 1){
        consoleLog("Overturned: disabeling motors\r\n");
        enable        = 0;
        //cmd1 = cmd2 = 0;
        shortBeep(4);                     // make 2 beeps indicating the motor disable
        HAL_Delay(100);
        shortBeep(6); 
      }
    } 
    if (errCode_Left || errCode_Right) {    // disable motors and beep in case of Motor error - fast beep
      if (enable){
        consoleLog("Motor Error\r\n");
        enable        = 0;
        buzzerFreq    = 8;
        buzzerPattern = 1;
      }
    } else if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) || (batVoltage < BAT_LOW_DEAD && speedAvgAbs < 20)) {  // poweroff before mainboard burns OR low bat 3
      consoleLog("Overtemp or battery low\r\n");
      poweroff();
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {  // beep if mainboard gets hot
      consoleLog("Temp Warning\r\n");
      buzzerFreq    = 4;
      buzzerPattern = 1;
    } else if (batVoltage < BAT_LOW_LVL1 && batVoltage >= BAT_LOW_LVL2 && BAT_LOW_LVL1_ENABLE) {  // low bat 1: slow beep
      consoleLog("Low Battery Warning 1\r\n");
      buzzerFreq    = 5;
      buzzerPattern = 42;
    } else if (batVoltage < BAT_LOW_LVL2 && batVoltage >= BAT_LOW_DEAD && BAT_LOW_LVL2_ENABLE) {  // low bat 2: fast beep
      consoleLog("Very Low Battery Warning 2\r\n");
      buzzerFreq    = 5;
      buzzerPattern = 6;
    } else if (timeoutFlagADC || timeoutFlagSerial) {  // beep in case of ADC or Serial timeout - fast beep      
      consoleLog("ADC/Serail timeout\r\n");
      buzzerFreq    = 24;
      buzzerPattern = 1;
    // } else if (BEEPS_BACKWARD && ((speed < -50 && speedAvg < 0) || MultipleTapBreak.b_multipleTap)) {  // backward beep
    //   buzzerFreq    = 5;
    //   buzzerPattern = 1;
    } else {  // do not beep
      buzzerFreq    = 0;
      buzzerPattern = 0;
    }


    // ####### INACTIVITY TIMEOUT #######
    if (abs(speedL) > 50 || abs(speedR) > 50) {
      inactivity_timeout_counter = 0;
    } else {
      inactivity_timeout_counter ++;
    }
    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // rest of main loop needs maybe 1ms
      consoleLog("Timeout Poweroff\r\n");
      poweroff();
    }

    main_loop_counter++;
    timeout++;
  }
}

#ifdef VARIANT_TRANSPOTTER
  void saveConfig() {
    HAL_FLASH_Unlock();
    EE_WriteVariable(VirtAddVarTab[0], saveValue);
    HAL_FLASH_Lock();
  }
#endif

void longBeep(uint8_t freq){
    buzzerFreq = freq;
    HAL_Delay(500);
    buzzerFreq = 0;
}

void shortBeep(uint8_t freq){
    buzzerFreq = freq;
    HAL_Delay(100);
    buzzerFreq = 0;
}

// ===========================================================
  /* Low pass filter fixed-point 32 bits: fixdt(1,32,20)
  * Max:  2047.9375
  * Min: -2048
  * Res:  0.0625
  * 
  * Inputs:       u     = int16
  * Outputs:      y     = fixdt(1,32,20)
  * Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  * 
  * Example: 
  * If coef = 0.8 (in floating point), then coef = 0.8 * 2^16 = 52429 (in fixed-point)
  * filtLowPass16(u, 52429, &y);
  * yint = (int16_t)(y >> 20); // the integer output is the fixed-point ouput shifted by 20 bits
  */
void filtLowPass32(int16_t u, uint16_t coef, int32_t *y)
{
  int32_t tmp;
  
  tmp = (int16_t)(u << 4) - (*y >> 16);  
  tmp = CLAMP(tmp, -32768, 32767);  // Overflow protection  
  *y  = coef * tmp + (*y);
}

// ===========================================================
  /* mixerFcn(rtu_speed, rtu_steer, &rty_speedR, &rty_speedL); 
  * Inputs:       rtu_speed, rtu_steer                  = fixdt(1,16,4)
  * Outputs:      rty_speedR, rty_speedL                = int16_t
  * Parameters:   SPEED_COEFFICIENT, STEER_COEFFICIENT  = fixdt(0,16,14)
  */
void mixerFcn(int16_t rtu_speed, int16_t rtu_steer, int16_t *rty_speedR, int16_t *rty_speedL)
{
  int16_t prodSpeed;
  int16_t prodSteer;
  int32_t tmp;

  prodSpeed   = (int16_t)((rtu_speed * (int16_t)SPEED_COEFFICIENT) >> 14);
  prodSteer   = (int16_t)((rtu_steer * (int16_t)STEER_COEFFICIENT) >> 14);

  tmp         = prodSpeed - prodSteer;  
  tmp         = CLAMP(tmp, -32768, 32767);  // Overflow protection
  *rty_speedR = (int16_t)(tmp >> 4);        // Convert from fixed-point to int 
  *rty_speedR = CLAMP(*rty_speedR, INPUT_MIN, INPUT_MAX);

  tmp         = prodSpeed + prodSteer;
  tmp         = CLAMP(tmp, -32768, 32767);  // Overflow protection
  *rty_speedL = (int16_t)(tmp >> 4);        // Convert from fixed-point to int
  *rty_speedL = CLAMP(*rty_speedL, INPUT_MIN, INPUT_MAX);
}

// ===========================================================
  /* rateLimiter16(int16_t u, int16_t rate, int16_t *y);
  * Inputs:       u     = int16
  * Outputs:      y     = fixdt(1,16,4)
  * Parameters:   rate  = fixdt(1,16,4) = [0, 32767] Do NOT make rate negative (>32767)
  */
void rateLimiter16(int16_t u, int16_t rate, int16_t *y)
{
  int16_t q0;
  int16_t q1;

  q0 = (u << 4)  - *y;

  if (q0 > rate) {
    q0 = rate;
  } else {
    q1 = -rate;
    if (q0 < q1) {
      q0 = q1;
    }
  }

  *y = q0 + *y;
}

// ===========================================================
  /* multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x)
  * This function detects multiple tap presses, such as double tapping, triple tapping, etc.
  * Inputs:       u = int16_t (input signal); timeNow = uint32_t (current time)  
  * Outputs:      x->b_multipleTap (get the output here)
  */
void multipleTapDet(int16_t u, uint32_t timeNow, MultipleTap *x)
{
  uint8_t 	b_timeout;
  uint8_t 	b_hyst;
  uint8_t 	b_pulse;
  uint8_t 	z_pulseCnt;
  uint8_t   z_pulseCntRst;
  uint32_t 	t_time; 

  // Detect hysteresis
  if (x->b_hysteresis) {
    b_hyst = (u > MULTIPLE_TAP_LO);
  } else {
    b_hyst = (u > MULTIPLE_TAP_HI);
  }

  // Detect pulse
  b_pulse = (b_hyst != x->b_hysteresis);

  // Save time when first pulse is detected
  if (b_hyst && b_pulse && (x->z_pulseCntPrev == 0)) {
    t_time = timeNow;
  } else {
    t_time = x->t_timePrev;
  }

  // Create timeout boolean
  b_timeout = (timeNow - t_time > MULTIPLE_TAP_TIMEOUT);

  // Create pulse counter
  if ((!b_hyst) && (x->z_pulseCntPrev == 0)) {
    z_pulseCnt = 0U;
  } else {
    z_pulseCnt = b_pulse;
  }

  // Reset counter if we detected complete tap presses OR there is a timeout
  if ((x->z_pulseCntPrev >= MULTIPLE_TAP_NR) || b_timeout) {
    z_pulseCntRst = 0U;
  } else {
    z_pulseCntRst = x->z_pulseCntPrev;
  }
  z_pulseCnt = z_pulseCnt + z_pulseCntRst;

  // Check if complete tap presses are detected AND no timeout
  if ((z_pulseCnt >= MULTIPLE_TAP_NR) && (!b_timeout)) {
    x->b_multipleTap = !x->b_multipleTap;	// Toggle output
  }

  // Update states
  x->z_pulseCntPrev = z_pulseCnt;
  x->b_hysteresis 	= b_hyst;
  x->t_timePrev 	  = t_time;
}

// ===========================================================

/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  // PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;  // 16 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);

  // enable the DWT counter for cycle timing
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
