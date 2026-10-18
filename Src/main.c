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
void sub_FSM_Search(Kobuki_Typedef kobuki){
  // tbd
}

void sub_FSM_Orient(Kobuki_Typedef kobuki){
  // tbd, please use orient_success and Orient_Result_Enum
}

void sub_FSM_Pathfind(Kobuki_Typedef kobuki){
  // tbd
}

typedef enum { STORAGE_ROTATE_SET, STORAGE_ROTATE, STORAGE_DRIVE_SET, STORAGE_DRIVE } Storage_Step_Enum;
static Storage_Step_Enum storage_step = STORAGE_ROTATE_SET;

void sub_FSM_Storage(Kobuki_Typedef kobuki){
  switch (storage_step) {
    case STORAGE_ROTATE_SET:
      if (course == A) {
        Kobuki_Rotate_Angle_Setpoint(kobuki.angle, 135);   // CCW rotation
      } else if (course == B) {
        Kobuki_Rotate_Angle_Setpoint(kobuki.angle, -135);  // CW rotation
      } else {
        return;                         // course unknown, don't move
      }
      storage_step = STORAGE_ROTATE;
      break;

    case STORAGE_ROTATE:
      if (kobuki.rotation_complete) {
        Kobuki_Drive(STOP);
        storage_step = STORAGE_DRIVE_SET;
      } else {
        Kobuki_Rotate_Angle(kobuki.angle);   // Rotates every cycle until done
      }
      break;

    case STORAGE_DRIVE_SET:
      Kobuki_Drive_Distance_Setpoint(kobuki.distance, 900);
      storage_step = STORAGE_DRIVE;
      break;

    case STORAGE_DRIVE:
      if (kobuki.distance_complete) {
        Kobuki_Drive(STOP);
        storage_step = STORAGE_ROTATE_SET;  // ready for next time
        storage_complete = true;            // tells Main_FSM to go to DEPOSIT
      } else {
        Kobuki_Drive_Distance(kobuki.distance);
      }
      break;
  }
}

void sub_FSM_Deposit(Kobuki_Typedef kobuki){
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
        sub_FSM_Search(kobuki);
      }
      break;

    case ORIENT:
      if (orient_success == ORIENT_PENDING) {
        sub_FSM_Orient(kobuki);
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
        sub_FSM_Pathfind(kobuki);
      }
      break;

    case STORAGE:
      if (storage_complete) {
        storage_complete = false;
        state = DEPOSIT;
      } else {
        sub_FSM_Storage(kobuki);
      }
      break;

    case DEPOSIT:
      if (deposit_complete) {
        Main_FSM_Reset();
        state = IDLE;
      } else {
        sub_FSM_Deposit(kobuki);
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