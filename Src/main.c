#include <stdint.h>
#include <stdbool.h>

#define STM32G474xx
#include "adc.h"
#include "kobuki.h"
#include "stm32g4xx.h"
#include "uart.h"
#include "ultrasonic.h"

/* Kobuki button values */
#define BUTTON_0_START 0x01U
#define BUTTON_1_STOP 0x02U
#define BUTTON_2_ROTATE 0x04U

// Logic
#define NULL 0
#define TRUE 1

// Main FSM states (Figure 5)
typedef enum { IDLE, SEARCH, ORIENT, PATHFIND, STORAGE, DEPOSIT } Main_State_Enum;

// Result of the ORIENT state: success -> PATHFIND, fail -> back to SEARCH
typedef enum { ORIENT_PENDING, ORIENT_SUCCESS, ORIENT_FAIL } Orient_Result_Enum;

// The course that Search detects
typedef enum { UNKNOWN, A, B } Current_Course;


// Main FSM transition flags, set by each state's sub-FSM when it finishes.
// B0, B1 and wheeldrop are read directly from the kobuki struct, not stored here.
static bool search_done        = false;
static bool pathfind_complete  = false;
static bool storage_complete   = false;
static bool deposit_complete   = false;

static Main_State_Enum state = IDLE;
static Orient_Result_Enum orient_success = ORIENT_PENDING;
static Current_Course course = UNKNOWN;

// Search Sub FSM
void sub_FSM_Search(void){
  // tbd
}

void sub_FSM_Orient(void){
  // tbd, please use orient_success and Orient_Result_Enum
}

void sub_FSM_Pathfind(void){
  // tbd
}

void sub_FSM_Storage(void){
  
}

void sub_FSM_Deposit(void){
  // tbd
}

// Stop the motors and clear every transition flag so the next run starts fresh
static void Main_FSM_Reset(void) {
  Kobuki_Drive(STOP);
  search_done       = false;
  pathfind_complete = false;
  storage_complete  = false;
  deposit_complete  = false;
  orient_success    = ORIENT_PENDING;
}

// Overall FSM (Figure 5)
void Main_FSM(Kobuki_Typedef kobuki) {
  bool B0 = kobuki.button & BUTTON_0_START;
  bool B1 = kobuki.button & BUTTON_1_STOP;

  // wheeldrop|B1: every active state returns to IDLE
  if (state != IDLE && (kobuki.wheeldrop || B1)) {
    Main_FSM_Reset();
    state = IDLE;
    return;
  }

  switch (state) {
    case IDLE:
      if (B0) {
        state = SEARCH;
      } else {
        Kobuki_Drive(STOP);  // !B0/stop
      }
      break;

    case SEARCH:
      if (search_done) {
        search_done = false;
        state = ORIENT;
      } else {
        sub_FSM_Search();
      }
      break;

    case ORIENT:
      if (orient_success == ORIENT_PENDING) {
        sub_FSM_Orient();
      } else {
        state = (orient_success == ORIENT_SUCCESS) ? PATHFIND : SEARCH;
        orient_success = ORIENT_PENDING;
      }
      break;

    case PATHFIND:
      if (pathfind_complete) {
        pathfind_complete = false;
        state = STORAGE;
      } else {
        sub_FSM_Pathfind();
      }
      break;

    case STORAGE:
      if (storage_complete) {
        storage_complete = false;
        state = DEPOSIT;
      } else {
        sub_FSM_Storage();
      }
      break;

    case DEPOSIT:
      if (deposit_complete) {
        Main_FSM_Reset();
        state = IDLE;
      } else {
        sub_FSM_Deposit();
      }
      break;

    default:
      Main_FSM_Reset();
      state = IDLE;
      break;
  }
}

int main(void) {
  // Both of these only for testing purposes. Not required
  volatile uint16_t ultrasonic_read;
  volatile uint16_t adc_read;

  Kobuki_Typedef kobuki;

  US_Typedef object;

  Kobuki_LED_Enum led_1, led_2;

  UART_Init();

  ADC_Init();

  Ultrasonic_Init();

  state = IDLE;
  led_1 = OFF;
  led_2 = OFF;

  while (1) {
    Kobuki_Read(&kobuki);

    Ultrasonic_Detect(&object);

    // Both of these only for testing purposes. Not required
    ultrasonic_read = Ultrasonic_Read();
    adc_read = ADC_Read(ADC_CHANNEL_1);

    Kobuki_Update_LEDs(led_1, led_2);

    Main_FSM(kobuki);
  }
}