// #include "Ultrasonic/ultrasonic.h"
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

// Rotations (used by ORIENT and PATHFIND)
#define CW (-90)
#define CCW 90

// ---- Shared between FSMs ----

// The course that Search detects
typedef enum { UNKNOWN, A, B } Current_Course;
static Current_Course course =
    UNKNOWN; // set by ORIENT, used by PATHFIND and STORAGE

// Progress of a sub-FSM: returned by sub_FSM_Search, stored by ORIENT, STORAGE
// and DEPOSIT
typedef enum { STATUS_PENDING, STATUS_DONE, STATUS_FAIL } Status_Enum;

// The following LEDs are not required, but may be useful for debugging.
// led_1: search progress (YELLOW finding, GREEN sweeping, RED fault)
// led_2: layout determination (GREEN = A, YELLOW = B, RED = not a bollard)
static Kobuki_LED_Enum led_1 = OFF;
static Kobuki_LED_Enum led_2 = OFF;

// Angle helper function to ensure angle is always between -180 and 180 degrees
// and given as a centidegree (this is what the Kobuki uses for its angle)
static int32_t wrap_angle(int32_t angle) {
  if (angle > 18000)
    return angle - 36000;
  if (angle < -18000)
    return angle + 36000;
  return angle;
}

// Centidegrees -> whole degrees, rounded
static int16_t Cdeg_To_Deg(int32_t cdeg) {
  return (int16_t)((cdeg >= 0) ? ((cdeg + 50) / 100) : ((cdeg - 50) / 100));
}

// Search Sub FSM State
typedef enum {
  SEARCH_INIT,
  SEARCH_LEFT,
  SEARCH_RIGHT,
  SEARCH_FACE,
  SEARCH_FAULT
} Search_State_Enum;

static Search_State_Enum search_state = SEARCH_INIT;

// variable definitions for variables used in FSM to keep it clean
static bool edge_seen = false;    // sweep reached bollard
static uint8_t edge_count = 0;    // debounce counter
static int32_t prev_angle = 0;    // previous angle
static int32_t turned_angle = 0;  // total turned angle
static int32_t theta_left = 0;    // CCW edge of bollard
static int32_t theta_right = 0;   // CW edge of bollard
static int32_t theta_mid = 0;     // midpoint of bollard
static bool search_retry = false; // set by ORIENT on FAIL
static uint8_t attempts = 0;      // number of attempts to find bollard

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

// Call once when a state begins a new search, then call sub_FSM_Search every
// loop
static void Search_Start(void) {
  attempts = 0;
  search_state = SEARCH_INIT;
}

// Search Sub FSM - also called by storage and deposit state
Status_Enum sub_FSM_Search(const Kobuki_Typedef *kobuki,
                           const US_Typedef *object) {
  int32_t angle = kobuki->angle; // store angle
  bool bollard_detected =
      object->detected; // range < ULTRASONIC_DETECT_RANGE_MM

  turned_angle += abs(wrap_angle(angle - prev_angle)); // store turned angle
  prev_angle = angle;                                  // update previous angle

  // Switch statement to handle the different states of the search sub-FSM
  switch (search_state) {

  case SEARCH_INIT: // initialisation state, performs no movement
    led_2 = YELLOW;
    attempts++;

    if (attempts > 3) {
      next_state(SEARCH_FAULT, angle); // give up after 3 tries
    } else if (search_retry) {         // if retry flag high
      search_retry = false;            // reset retry flag
      next_state(SEARCH_LEFT,
                 angle); // SEARCH_LEFT state to find edge of the bollard
    } else {
      next_state(SEARCH_LEFT, angle); // move to next state otherwise
    }
    break;

  case SEARCH_LEFT: // Turn CCW to find left edge
    if (!edge_seen) {
      Kobuki_Rotate(SEARCH_SPEED); // begin CCW rotation

      if (Edge_Confirmed(bollard_detected)) { // detect edge and apply actions
        edge_seen = true;
        edge_count = 0;
        led_2 = GREEN;
      }
    } else {
      Kobuki_Rotate(MIN_SPEED); // once edge detected, rotate slower
      if (Edge_Confirmed(!bollard_detected)) { // edge no longer detected (left
                                               // side of bollard)
        theta_left = angle;                    // record angle of left edge
        next_state(SEARCH_RIGHT, angle);       // next state
        break;
      }
    }

    if (turned_angle >= 36000) {       // fault detection
      next_state(SEARCH_FAULT, angle); // nothing found in a full turn
    }
    break;

  case SEARCH_RIGHT:           // Turn CW to find right edge
    Kobuki_Rotate(-MIN_SPEED); // begin CW slow rotation
    if (!edge_seen) {
      if (Edge_Confirmed(bollard_detected)) { // detect edge and apply actions
        edge_seen = true;
        edge_count = 0;
      } else if (turned_angle >= 9000) { // fault detection
        next_state(SEARCH_INIT, angle);  // lost it within 90 deg: start again
      }
    } else if (Edge_Confirmed(
                   !bollard_detected)) { // one edge no longer detected (right
                                         // side of bollard)
      theta_right = angle;               // record angle of right edge
      int32_t span =
          (theta_left - theta_right + 36000) % 36000; // calculate span
      theta_mid = wrap_angle(angle + span / 2); // calculate center of bollard

      Kobuki_Rotate(STOP); // stop robot
      Kobuki_Rotate_Angle_Setpoint(kobuki->angle,
                                   Cdeg_To_Deg(wrap_angle(theta_mid - angle)));
      next_state(SEARCH_FACE, angle);
    } else if (turned_angle >= 36000) { // fault detection
      next_state(SEARCH_FAULT, angle);  // change to fault state
    }
    break;

  case SEARCH_FACE:
    if (kobuki->rotation_complete) {
      Kobuki_Rotate(STOP);
      next_state(SEARCH_INIT, angle); // loop back to start fresh search
      return STATUS_DONE; // tells the caller the robot is facing the bollard
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
    return STATUS_FAIL;
  }

  return STATUS_PENDING;
}

// ORIENT result: DONE -> PATHFIND, FAIL -> back to SEARCH
static Status_Enum orient_success = STATUS_PENDING;
static bool orient_started = false;

void sub_FSM_Orient(Kobuki_Typedef *kobuki, const US_Typedef *object) {
  if (!orient_started) {
    // uint16_t range = object->range_mm;
    if (object->range_mm <= 600) {
      course = B;
      led_1 = YELLOW;
      Kobuki_Rotate_Angle_Setpoint(
          kobuki->angle,
          Cdeg_To_Deg(wrap_angle(theta_mid + CCW * 100 - kobuki->angle)));
    } else if (object->range_mm <= 900) {
      course = A;
      led_1 = GREEN;
      Kobuki_Rotate_Angle_Setpoint(
          kobuki->angle,
          Cdeg_To_Deg(wrap_angle(theta_mid + CW * 100 - kobuki->angle)));
    } else {
      orient_success = STATUS_FAIL;
      course = UNKNOWN;
      led_1 = RED;
      return;
    }
    orient_started = true;
    return;
  }

  if (kobuki->rotation_complete) {
    Kobuki_Rotate(STOP);
    orient_started = false;
    orient_success = STATUS_DONE;
  } else {
    Kobuki_Rotate_Angle(kobuki->angle);
  }
}

// Drive Distances
#define HORIZONTAL_DISTANCE 1150 // mm
#define VERTICAL_DISTANCE 1350   // mm
#define REVERSE 50               // mm
#define REVERSE_ANGLE 3
#define speed 150

typedef enum {
  PATHFIND_FIRST,
  PATHFIND_ROTATE,
  PATHFIND_SECOND,
  PATHFIND_REVERSE,
  PATHFIND_REVERSE_ANGLE,
  PATHFIND_CENTER,
  PATHFIND_FAULT
} Pathfind_State_Enum;

static Pathfind_State_Enum pathfind_state = PATHFIND_FIRST;
static Pathfind_State_Enum pathfind_state_prev = PATHFIND_FIRST;
static bool pathfind_started = false;
static bool pathfind_complete = false; // tells Main_FSM to go to STORAGE
static bool reverse_started = false;   // Temp variable/ffunctionality
static int corb = 0;                   // 1 = cliff, 2= bumper, 3 = front cliff
static int pathfind_count = 0;

void sub_FSM_Pathfind(Kobuki_Typedef *kobuki) {
  static int16_t Path[3];
  // Could add path_initialised to stop Path[] being initialised every time
  if (course == A) {
    Path[0] = HORIZONTAL_DISTANCE;
    Path[1] = CCW;
    Path[2] = VERTICAL_DISTANCE;
  } else {
    Path[0] = VERTICAL_DISTANCE;
    Path[1] = CW;
    Path[2] = HORIZONTAL_DISTANCE;
  }
  switch (pathfind_state) {
  case PATHFIND_FIRST:
    if (pathfind_count == 3) {
      Kobuki_Drive(STOP);
      pathfind_started = false;
      pathfind_complete = true;
      pathfind_state = PATHFIND_FIRST;
    } else {
      Kobuki_Drive(speed);
    }
    break;

  /*
  case PATHFIND_FIRST:
    if (!pathfind_started){
      Kobuki_Drive_Distance_Setpoint(kobuki->distance, Path[0]);
      pathfind_started = true;
      break;
    }
    if (kobuki->distance_complete){
      Kobuki_Drive(STOP);
      pathfind_state = PATHFIND_ROTATE;
      pathfind_started = false;
      }
      else{
        Kobuki_Drive_Distance(kobuki->distance);
      }
    break;

  case PATHFIND_ROTATE:
    if (!pathfind_started){
      Kobuki_Rotate_Angle_Setpoint(kobuki->angle, Path[1]);
      pathfind_started = true;
      break;
    }

    if (kobuki->rotation_complete){
      Kobuki_Rotate(STOP);
      pathfind_state = PATHFIND_SECOND;
      pathfind_started = false;
      kobuki->rotation_complete = 0;
    }
    else{
      Kobuki_Rotate_Angle(kobuki->angle);
    }
    break;

  case PATHFIND_SECOND:
    if (!pathfind_started){
      Kobuki_Drive_Distance_Setpoint(kobuki->distance, Path[2]);
      pathfind_started = true;
      break;
    }

    if (kobuki->distance_complete){
      Kobuki_Drive(STOP);
      pathfind_complete = true;
      pathfind_started = false;
      pathfind_state = PATHFIND_FIRST;
    }
    else{
      Kobuki_Drive_Distance(kobuki->distance);
    }
    break;
    */
  case PATHFIND_REVERSE:
    if (!reverse_started) {
      Kobuki_Drive_Distance_Setpoint(kobuki->distance, -REVERSE);
      reverse_started = true;
      break;
    }

    if (kobuki->distance_complete) {
      Kobuki_Drive(STOP);
      kobuki->distance_complete = 0;
      pathfind_state = PATHFIND_REVERSE_ANGLE;
      reverse_started = false;
    } else {
      Kobuki_Drive_Distance(kobuki->distance);
    }
    break;
  case PATHFIND_REVERSE_ANGLE:
    if (!reverse_started) {
      if (corb == 3) {
        pathfind_count++;
        if (pathfind_count == 3) {
          pathfind_started = false;
          pathfind_complete = true;
          pathfind_state = PATHFIND_FIRST;
          corb = 0;
        }
      }
      if (course == A && corb == 1) {
        Kobuki_Rotate_Angle_Setpoint(kobuki->angle, REVERSE_ANGLE);
      } else if (course == A && corb == 2) {
        Kobuki_Rotate_Angle_Setpoint(kobuki->angle, -REVERSE_ANGLE);
      } else if (course == B && corb == 1) {
        Kobuki_Rotate_Angle_Setpoint(kobuki->angle, -REVERSE_ANGLE);
      } else if (course == B && corb == 2) {
        Kobuki_Rotate_Angle_Setpoint(kobuki->angle, REVERSE_ANGLE);
      } else if (course == A && corb == 3) {
        if (pathfind_count == 2) {
          Kobuki_Rotate_Angle_Setpoint(kobuki->angle, CW);
        } else {
          Kobuki_Rotate_Angle_Setpoint(kobuki->angle, CCW);
        }

      } else if (course == B && corb == 3) {
        if (pathfind_count == 2) {
          Kobuki_Rotate_Angle_Setpoint(kobuki->angle, CCW);
        } else {
          Kobuki_Rotate_Angle_Setpoint(kobuki->angle, CW);
        }
      }
      reverse_started = true;
      break;
    }

    if (kobuki->rotation_complete) {
      Kobuki_Rotate(STOP);
      kobuki->rotation_complete = 0;
      if (corb == 3 && pathfind_count == 2) {
        pathfind_complete = true;
      }
      pathfind_state = PATHFIND_FIRST;
      reverse_started = false;
      corb = 0;
    } else {
      Kobuki_Rotate_Angle(kobuki->angle);
    }
    break;
  case PATHFIND_FAULT:
    // used for course correction if needed
    break;
  }
}

typedef enum {
  STORAGE_ROTATE_SET,
  STORAGE_ROTATE,
  STORAGE_DRIVE_SET,
  STORAGE_DRIVE
  STORAGE_BUMPER, 
} Storage_Step_Enum;
static Storage_Step_Enum storage_step = STORAGE_DETECT;
static Status_Enum storage_status = STATUS_PENDING;
static bool storage_complete = false; // tells Main_FSM to go to DEPOSIT

void sub_FSM_Storage(Kobuki_Typedef *kobuki, US_Typedef *object) {
  switch (storage_step) {
  case STORAGE_ROTATE_SET:
    if (object->detected) {
      if (object->range_mm <= 900) {
        if (course == A) {
          Kobuki_Rotate_Angle_Setpoint(kobuki->angle, 135);
        } else {
          Kobuki_Rotate_Angle_Setpoint(kobuki->angle, -135);
        }
        storage_step = STORAGE_ROTATE;
        break;
      }
    }
    storage_status = STATUS_FAIL; // not facing bollard 2 after the search
    led_1 = RED;
    break;
  case STORAGE_ROTATE:
    if (kobuki->rotation_complete) {
      Kobuki_Rotate(STOP);
      storage_step = STORAGE_DRIVE_SET;
    } else {
      Kobuki_Rotate_Angle(kobuki->angle);
    }

    break;
  case STORAGE_DRIVE_SET:
    Kobuki_Drive_Distance_Setpoint(kobuki->distance, 900);
    storage_step = STORAGE_DRIVE;
    break;

  case STORAGE_DRIVE:
    if (kobuki->distance_complete || (object->range_mm < 150)) {
      Kobuki_Drive(STOP);
      storage_step = STORAGE_DETECT; // ready for next time
      storage_complete = true;       // tells Main_FSM to go to DEPOSIT
    } else {
      Kobuki_Drive_Distance(kobuki->distance);
    }
    break;
  }
}

#define DEPOSIT_DETECT_MM                                                      \
  ((uint16_t)500) // deposit marker counts as "seen" inside this range
#define DEPOSIT_STOP_MM ((uint16_t)125) // stop this far from the deposit marker

typedef enum {
  DEPOSIT_CENTRE, // Search for the deposit marker and face its centre
  DEPOSIT_DRIVE   // drive straight at it until close enough
} Deposit_Step_Enum;

static Deposit_Step_Enum deposit_step = DEPOSIT_CENTRE;
static Status_Enum deposit_status = STATUS_PENDING;
static bool deposit_complete = false; // tells Main_FSM to go to IDLE

void sub_FSM_Deposit(Kobuki_Typedef *kobuki, US_Typedef *object) {
  switch (deposit_step) {
  case DEPOSIT_CENTRE: {
    Status_Enum result = sub_FSM_Search(kobuki, object);
    if (result == STATUS_DONE) {
      deposit_step = DEPOSIT_DRIVE;
    } else if (result == STATUS_FAIL) {
      Kobuki_Rotate(STOP);
      deposit_status = STATUS_FAIL;
      led_1 = RED;
    }
    break;
  }

  case DEPOSIT_DRIVE:
    if (object->range_mm <= DEPOSIT_STOP_MM) {
      Kobuki_Drive(STOP);
      deposit_step = DEPOSIT_CENTRE; // ready for next time
      deposit_complete = true;       // tells Main_FSM to go to IDLE
    } else {
      Kobuki_Drive(MIN_SPEED);
    }
    break;

  default:
    break;
  }
}

// Main FSM states (Figure 5)
// B0, B1 and wheeldrop are read directly from the kobuki struct, not stored
// here.
typedef enum {
  IDLE,
  SEARCH,
  ORIENT,
  PATHFIND,
  STORAGE,
  DEPOSIT
} Main_State_Enum;
static Main_State_Enum state = IDLE;

// Stop the motors and clear every transition flag so the next run starts fresh
static void Main_FSM_Reset(void) {
  Kobuki_Drive(STOP);
  led_1 = OFF;
  led_2 = OFF;
  pathfind_complete = false;
  storage_complete = false;
  deposit_complete = false;
  orient_success = STATUS_PENDING;
  storage_step = STORAGE_DETECT;
  storage_status = STATUS_PENDING;
  deposit_step = DEPOSIT_CENTRE;
  deposit_status = STATUS_PENDING;
  pathfind_started = false;
  orient_started = false;
  pathfind_state = PATHFIND_FIRST;
  attempts = 0;
  search_state = SEARCH_INIT;
  search_retry = false;
  reverse_started = false;
  corb = 0;
  pathfind_count = 0;
  pathfind_state_prev = PATHFIND_FIRST;
}

// Overall FSM (Figure 5)
void Main_FSM(Kobuki_Typedef *kobuki, US_Typedef *object) {
  bool B0 = kobuki->button & BUTTON_0_START;
  bool B1 = kobuki->button & BUTTON_1_STOP;

  // wheeldrop|B1: every active state returns to IDLE
  if (state != IDLE && (kobuki->wheeldrop || B1)) {
    Main_FSM_Reset();
    state = IDLE;
    return;
  }

  if (state == PATHFIND && pathfind_state == PATHFIND_FIRST &&
      !pathfind_complete && (kobuki->cliff || kobuki->bumper)) {
    pathfind_state_prev = pathfind_state;
    if (kobuki->cliff == 0x01 || kobuki->cliff == 0x04) {
      corb = 1;
    } else if (kobuki->cliff == 0x02) {
      corb = 3;
    } else {
      corb = 2;
    }
    pathfind_state = PATHFIND_REVERSE;
  }

  switch (state) {
  case IDLE:
    if (B0) {
      course = UNKNOWN;
      Search_Start();
      state = SEARCH;
    } else {
      Kobuki_Drive(STOP); // !B0/stop
    }
    break;

  case SEARCH:
    if (sub_FSM_Search(kobuki, object) == STATUS_DONE) {
      state = ORIENT;
    }
    break;

  case ORIENT:
    if (orient_success == STATUS_PENDING) {
      sub_FSM_Orient(kobuki, object);
    } else {
      state = (orient_success == STATUS_DONE) ? PATHFIND : SEARCH;
      orient_success = STATUS_PENDING;
    }
    break;

  case PATHFIND:
    if (pathfind_complete) {
      pathfind_complete = false;
      Search_Start();
      state = STORAGE;
    } else {
      sub_FSM_Pathfind(kobuki);
    }
    break;

  case STORAGE:
    if (storage_status == STATUS_FAIL) {
      Main_FSM_Reset();
      state = IDLE;
      break;
    }
    if (storage_complete) {
      storage_complete = false;
      Search_Start();
      state = DEPOSIT;
    } else {
      sub_FSM_Storage(kobuki, object);
    }
    break;

  case DEPOSIT:
    if (deposit_status == STATUS_FAIL) {
      Main_FSM_Reset();
      state = IDLE;
      break;
    }
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