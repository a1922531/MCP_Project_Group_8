//#include "Ultrasonic/ultrasonic.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

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
#define TRUE 1

// Drive Distances and Rotations
#define HORIZONTAL_DISTANCE 1150 //mm
#define VERTICAL_DISTANCE 1350 //mm
#define CW (-90) 
#define CCW 90

// Main FSM states (Figure 5)
typedef enum { IDLE, SEARCH, ORIENT, PATHFIND, STORAGE, DEPOSIT } Main_State_Enum;

// The course that Search detects
typedef enum { UNKNOWN, A, B } Current_Course;

// Result of the ORIENT state: success -> PATHFIND, fail -> back to SEARCH
typedef enum { ORIENT_PENDING, ORIENT_SUCCESS, ORIENT_FAIL } Orient_Result_Enum;

// Main FSM transition flags, set by each state's sub-FSM when it finishes.
// B0, B1 and wheeldrop are read directly from the kobuki struct, not stored here.
static bool search_done = false;
static bool pathfind_complete = false;
static bool storage_complete = false;
static bool deposit_complete = false;

static bool pathfind_started = false;
static bool orient_started = false;

static Main_State_Enum state = IDLE;
static Orient_Result_Enum orient_success = ORIENT_PENDING;
static Current_Course course = UNKNOWN;

// The following LEDs are not required, but may be useful for debugging.
// led_1: search progress (YELLOW finding, GREEN sweeping, RED fault)
// led_2: layout determination (GREEN = A, YELLOW = B, RED = not a bollard)
static Kobuki_LED_Enum led_1 = OFF;
static Kobuki_LED_Enum led_2 = OFF;

// Angle helper function to ensure angle is always between -180 and 180 degrees and
// given as a centidegree (this is what the Kobuki uses for its angle)
static int32_t wrap_angle(int32_t angle) {
  if (angle > 18000)  return angle - 36000;
  if (angle < -18000) return angle + 36000;
  return angle;
}

// Centidegrees -> whole degrees, rounded
static int16_t Cdeg_To_Deg(int32_t cdeg) {
  return (int16_t)((cdeg >= 0) ? ((cdeg + 50) / 100) : ((cdeg - 50) / 100));
}

// Search Sub FSM State Definitions
typedef enum {
  SEARCH_INIT,
  SEARCH_LEFT,
  SEARCH_RIGHT,
  SEARCH_FACE,
  SEARCH_FAULT
} Search_State_Enum;

static Search_State_Enum search_state = SEARCH_INIT;

typedef enum {
  PATHFIND_FIRST,
  PATHFIND_ROTATE,
  PATHFIND_SECOND,
  PATHFIND_FAULT
} Pathfind_State_Enum;

static Pathfind_State_Enum pathfind_state = PATHFIND_FIRST;

// variable definitions for variables used in FSM to keep it clean
static bool edge_seen = false; // sweep reached bollard
static uint8_t edge_count = 0; // debounce counter
static int32_t prev_angle = 0; // previous angle
static int32_t turned_angle = 0; // total turned angle
static int32_t theta_left = 0; // CCW edge of bollard
static int32_t theta_right = 0; // CW edge of bollard
static int32_t theta_mid = 0; // midpoint of bollard
static bool search_retry = false; // set by ORIENT on FAIL
static uint8_t attempts = 0; // number of attempts to find bollard

// function to move states (clears required variables for next state)
static void next_state(Search_State_Enum next, int32_t angle) {
  search_state = next;
  edge_seen = false;
  edge_count = 0;
  turned_angle = 0;
  prev_angle = angle;  
}

// Function to read true once sensor has detected an edge for 3 loops
static bool Edge_Confirmed(bool condition) {
  if (condition) {
    edge_count++;
  } else {
    edge_count = 0;
  }
  return (edge_count >= 3);
}


// Search Sub FSM - also called by storage and deposit state
void sub_FSM_Search(const Kobuki_Typedef *kobuki, const US_Typedef *object){
  int32_t angle = kobuki->angle; // store angle
  bool bollard_detected = object->detected; // range < ULTRASONIC_DETECT_RANGE_MM

  turned_angle += abs(wrap_angle(angle - prev_angle)); // store turned angle
  prev_angle = angle; // update previous angle

  // Switch statement to handle the different states of the search sub-FSM
  switch (search_state) {

    case SEARCH_INIT: // initialisation state, performs no movement
      course = UNKNOWN;
      led_2 = YELLOW;
      attempts++;

      if (attempts > 3) { next_state(SEARCH_FAULT, angle); // give up after 3 tries
      } else if (search_retry) { // if retry flag high
        search_retry = false; // reset retry flag
        next_state(SEARCH_LEFT, angle); // SEARCH_LEFT state to find edge of the bollard
      } else {
        next_state(SEARCH_LEFT, angle); // move to next state otherwise
      }
    break;

    case SEARCH_LEFT: // Turn CCW to find left edge
      if (!edge_seen){  
        Kobuki_Rotate(SEARCH_SPEED); // begin CCW rotation

          if (Edge_Confirmed(bollard_detected)) { // detect edge and apply actions
            edge_seen = true;
            edge_count = 0;
            led_2 = GREEN;
          }
      } else {
        Kobuki_Rotate(MIN_SPEED); // once edge detected, rotate slower
          if (Edge_Confirmed(!bollard_detected)) { // edge no longer detected (left side of bollard)
            theta_left = angle; // record angle of left edge
            next_state(SEARCH_RIGHT, angle); // next state
            break;
          }
      }

      if (turned_angle >= 36000) { // fault detection
        next_state(SEARCH_FAULT, angle); // nothing found in a full turn
      }
    break;

    case SEARCH_RIGHT: // Turn CW to find right edge
      Kobuki_Rotate(-MIN_SPEED); // begin CW slow rotation
      if (!edge_seen){
        if (Edge_Confirmed(bollard_detected)) { // detect edge and apply actions
          edge_seen = true;
          edge_count = 0;
        } else if (turned_angle >= 9000) { // fault detection
          next_state(SEARCH_INIT, angle); // lost it within 90 deg: start again
        }
      } else if (Edge_Confirmed(!bollard_detected)) { // one edge no longer detected (right side of bollard)
            theta_right = angle; // record angle of right edge
            int32_t span = (theta_left - theta_right + 36000) % 36000; // calculate span
            theta_mid = wrap_angle(angle + span / 2); // calculate center of bollard

            Kobuki_Rotate(STOP); // stop robot
            Kobuki_Rotate_Angle_Setpoint(kobuki->angle, Cdeg_To_Deg(wrap_angle(theta_mid - angle)));
            next_state(SEARCH_FACE, angle);
      } else if (turned_angle >= 36000) { // fault detection
        next_state(SEARCH_FAULT, angle); // change to fault state
      }
    break;

    case SEARCH_FACE:
      if (kobuki->rotation_complete) {
        Kobuki_Rotate(STOP);
        search_done = true; // flag used in MAIN FSM to mark transition from SEARCH -> ORIENT state
        next_state(SEARCH_INIT, angle); // loop back to start fresh search
      } else {
        Kobuki_Rotate_Angle(kobuki->angle);
      }
    break;

    case SEARCH_FAULT:
    default:
      // Nothing found: stop and wait for B1
      Kobuki_Rotate(STOP);
      led_2 = RED;
      search_state = SEARCH_FAULT;
    break;
  }
}

void sub_FSM_Orient(Kobuki_Typedef* kobuki, const US_Typedef* object) {
  if (!orient_started){
    //uint16_t range = object->range_mm;
    if (object->range_mm <= 550){
      course = B;
      led_1 = YELLOW;
      Kobuki_Rotate_Angle_Setpoint(kobuki->angle, Cdeg_To_Deg(wrap_angle(theta_mid + CCW * 100 - kobuki->angle)));
    }
    else if (object->range_mm <= 900){
      course = A;
      led_1 = GREEN;
      Kobuki_Rotate_Angle_Setpoint(kobuki->angle, Cdeg_To_Deg(wrap_angle(theta_mid + CW * 100 - kobuki->angle)));
    }
    else{
      orient_success = ORIENT_FAIL;
      course = UNKNOWN;
      led_1 = RED;
      return;
    }
    orient_started = true;
    return;
  }

  if (kobuki->rotation_complete){
    Kobuki_Rotate(STOP);
    orient_started = false;
    orient_success = ORIENT_SUCCESS;
  } else {
    Kobuki_Rotate_Angle(kobuki->angle);
  }
}

void sub_FSM_Pathfind(Kobuki_Typedef* kobuki){
  static int16_t Path[3];
  //Could add path_initialised to stop Path[] being initialised every time 
    if (course == A){
      Path[0] = HORIZONTAL_DISTANCE; Path[1] = CCW; Path[2] = VERTICAL_DISTANCE; 
    }
    else {
      Path[0] = VERTICAL_DISTANCE; Path[1] = CW; Path[2] = HORIZONTAL_DISTANCE; 
    }
    switch (pathfind_state){
      case PATHFIND_FIRST:
        if (!pathfind_started){
          Kobuki_Drive_Distance_Setpoint(kobuki->distance, Path[0]);
          pathfind_started = true;
          break;
        }

        Kobuki_Drive_Distance(kobuki->distance);

        if (kobuki->distance_complete){
          Kobuki_Drive(STOP);
          pathfind_state = PATHFIND_ROTATE;
          pathfind_started = false;
          }
        break;
        
      case PATHFIND_ROTATE:
        if (!pathfind_started){
          Kobuki_Rotate_Angle_Setpoint(kobuki->angle, Path[1]);
          pathfind_started = true;
          break;
        }
        Kobuki_Rotate_Angle(kobuki->angle);

        if (kobuki->rotation_complete){
          Kobuki_Rotate(STOP);
          pathfind_state = PATHFIND_SECOND;
          pathfind_started = false;
        }
        break;

      case PATHFIND_SECOND:
        if (!pathfind_started){
          Kobuki_Drive_Distance_Setpoint(kobuki->distance, Path[2]);
          pathfind_started = true;
          break;
        }
        
        Kobuki_Drive_Distance(kobuki->distance);

        if (kobuki->distance_complete){
          Kobuki_Drive(STOP);
          pathfind_complete = true;   
          pathfind_started = false;   
          pathfind_state = PATHFIND_FIRST; 
        }
        break;
      case PATHFIND_FAULT:
        //used for course correction if needed
        break;
    }
}

typedef enum { STORAGE_ROTATE_SET, STORAGE_ROTATE, STORAGE_DRIVE_SET, STORAGE_DRIVE } Storage_Step_Enum;
static Storage_Step_Enum storage_step = STORAGE_ROTATE_SET;

void sub_FSM_Storage(Kobuki_Typedef* kobuki) {
  switch (storage_step) {
    case STORAGE_ROTATE_SET:
      if (course == A) {
        Kobuki_Rotate_Angle_Setpoint(kobuki->angle, 135);  // CCW rotation
      } else if (course == B) {
        Kobuki_Rotate_Angle_Setpoint(kobuki->angle, -135);  // CW rotation
      } else {
        return;  // course unknown, don't move
      }
      storage_step = STORAGE_ROTATE;
      break;

    case STORAGE_ROTATE:
      if (kobuki->rotation_complete) {
        Kobuki_Drive(STOP);
        storage_step = STORAGE_DRIVE_SET;
      } else {
        Kobuki_Rotate_Angle(kobuki->angle);  // Rotates every cycle until done
      }
      break;

    case STORAGE_DRIVE_SET:
      Kobuki_Drive_Distance_Setpoint(kobuki->distance, 900);
      storage_step = STORAGE_DRIVE;
      break;

    case STORAGE_DRIVE:
      if (kobuki->distance_complete) {
        Kobuki_Drive(STOP);
        storage_step = STORAGE_ROTATE_SET;  // ready for next time
        storage_complete = true;            // tells Main_FSM to go to DEPOSIT
      } else {
        Kobuki_Drive_Distance(kobuki->distance);
      }
      break;
  }
}

#define DEPOSIT_DETECT_MM ((uint16_t)500)  // deposit marker counts as "seen" inside this range
#define DEPOSIT_STOP_MM   ((uint16_t)125)  // stop this far from the deposit marker

typedef enum {
  DEPOSIT_SCAN_ENTRY,   // rotate CCW until the marker enters the beam
  DEPOSIT_SCAN_EXIT,    // keep rotating until the marker leaves the beam
  DEPOSIT_CENTRE_SET,   // turn back by half the sweep to face the marker's centre
  DEPOSIT_CENTRE,
  DEPOSIT_DRIVE
} Deposit_Step_Enum;

static Deposit_Step_Enum deposit_step = DEPOSIT_SCAN_ENTRY;
static int16_t entry_tick = 0;  // kobuki.angle (0.01 deg) when the marker entered the beam
static int16_t exit_tick = 0;   // kobuki.angle (0.01 deg) when the marker left the beam

void sub_FSM_Deposit(Kobuki_Typedef* kobuki, US_Typedef* object) {
  int16_t sweep_deg;

  switch (deposit_step) {
    case DEPOSIT_SCAN_ENTRY:
      if (object->range_mm <= DEPOSIT_DETECT_MM) {
        entry_tick = kobuki->angle;
        deposit_step = DEPOSIT_SCAN_EXIT;
      }
      Kobuki_Rotate(SEARCH_SPEED);  // constant-speed CCW sweep
      break;

    case DEPOSIT_SCAN_EXIT:
      if (object->range_mm > DEPOSIT_DETECT_MM) {
        exit_tick = kobuki->angle;
        Kobuki_Drive(STOP);
        deposit_step = DEPOSIT_CENTRE_SET;
      } else {
        Kobuki_Rotate(SEARCH_SPEED);
      }
      break;

    case DEPOSIT_CENTRE_SET:
      // Angle swept across the marker, wrapped to [-180, 180] deg
      sweep_deg = (int16_t)((exit_tick - entry_tick) / 100);
      if (sweep_deg > 180) sweep_deg -= 360;
      if (sweep_deg < -180) sweep_deg += 360;

      Kobuki_Rotate_Angle_Setpoint(kobuki->angle, -sweep_deg / 2);  // relative turn back to the centre
      deposit_step = DEPOSIT_CENTRE;
      break;

    case DEPOSIT_CENTRE:
      if (kobuki->rotation_complete) {
        Kobuki_Drive(STOP);
        deposit_step = DEPOSIT_DRIVE;
      } else {
        Kobuki_Rotate_Angle(kobuki->angle);
      }
      break;

    case DEPOSIT_DRIVE:
      if (object->range_mm <= 125) {
        Kobuki_Drive(STOP);
        deposit_step = DEPOSIT_SCAN_ENTRY;  // ready for next time
        deposit_complete = true;            // tells Main_FSM to go to IDLE
      } else {
        Kobuki_Drive(MIN_SPEED);
      }
      break;

    default:
      break;
  }
}

// Stop the motors and clear every transition flag so the next run starts fresh
static void Main_FSM_Reset(void) {
  Kobuki_Drive(STOP);
  led_1 = OFF;
  led_2 = OFF;
  search_done = false;
  pathfind_complete = false;
  storage_complete = false;
  deposit_complete = false;
  orient_success = ORIENT_PENDING;
  storage_step = STORAGE_ROTATE_SET;
  deposit_step = DEPOSIT_SCAN_ENTRY;
  pathfind_started = false;
  orient_started = false;
  pathfind_state = PATHFIND_FIRST;
  attempts = 0;
  search_state = SEARCH_INIT;
  search_retry = false;
}

// Overall FSM (Figure 5)
void Main_FSM(Kobuki_Typedef* kobuki, US_Typedef* object) {
  bool B0 = kobuki->button & BUTTON_0_START;
  bool B1 = kobuki->button & BUTTON_1_STOP;

  // wheeldrop|B1: every active state returns to IDLE
  if (state != IDLE && (kobuki->wheeldrop || B1)) {
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
        sub_FSM_Search(kobuki, object);
      }
      break;

    case ORIENT:
      if (orient_success == ORIENT_PENDING) {
        sub_FSM_Orient(kobuki, object);
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
        sub_FSM_Deposit(kobuki, object);
      }
      break;

    default:
      Main_FSM_Reset();
      state = IDLE;
      break;
  }
}

int main(void) {
  Kobuki_Typedef kobuki;

  US_Typedef object;

  UART_Init();

  ADC_Init();

  Ultrasonic_Init();

  while (1) {
    Kobuki_Read(&kobuki);

    Ultrasonic_Detect(&object);

    Kobuki_Update_LEDs(led_1, led_2);

    Main_FSM(&kobuki, &object);
  }
}