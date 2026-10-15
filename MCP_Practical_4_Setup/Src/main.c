#include <stdint.h>
#define STM32G474xx
#include "stm32g4xx.h"
#include "adc.h"        // New Header
#include "kobuki.h"
#include "uart.h"
#include "ultrasonic.h" // New Header

#define BUTTON_0_START  0x01U // Confirm the values from my_kobuki
#define BUTTON_1_STOP   0x02U // Confirm the values from my_kobuki
#define BUTTON_2_ROTATE 0x04U // Confirm the values from my_kobuki

#define DRIVE_SPEED_MM_S  ((int16_t)100)
#define ROTATE_SPEED_MM_S ((int16_t)100)

/* Motor speeds in mm/s */
#define SEARCH_SPEED_MM_S  ((int16_t)75)
#define FORWARD_SPEED_MM_S ((int16_t)100)
#define REVERSE_SPEED_MM_S ((int16_t)-100)
#define AVOID_SPEED_MM_S   ((int16_t)100)

/* Approximate delay for the AVOID state.
 * Adjust this value if the rotation is too short or too long. */
#define AVOID_DELAY_COUNT 300000U

typedef enum {
    IDLE = 0,
    SEARCH,
    OBJECT,
    OBSTACLE,
    AVOID
} RobotState;

static void AvoidDelay(void)
{
    for (volatile uint32_t i = 0U;
         i < AVOID_DELAY_COUNT;
         ++i) {
        __NOP();
    }
}


// Change to 1 for the Section 7 receiver test, 0 for the Section 8 transmit program
#if 0


/* ===== Section 7: Testing the receiver drivers ===== */
int main(void) {
    // Create a structure to store kobuki values:
    Kobuki_Typedef my_kobuki = {0};

    // Initialize system UART
    UART_Init();

    while (1) {

    // Read the full payload arriving at the RX pin and parse it
    Kobuki_Read(&my_kobuki);

    // Pause so you can inspect 'my_kobuki' in your debugger
    for(volatile int i = 0; i < 500000; i++);
    }
}

#else

static void Send_Kobuki_Move(int16_t speed, int16_t radius)
{
    uint8_t payload[6];

    payload[0] = 0x01U; /* Base Control sub-packet ID. */
    payload[1] = 0x04U; /* Speed and radius occupy four bytes. */
    payload[2] = (uint8_t)(speed & 0xFF);
    payload[3] = (uint8_t)((uint16_t)speed >> 8);
    payload[4] = (uint8_t)(radius & 0xFF);
    payload[5] = (uint8_t)((uint16_t)radius >> 8);

    Kobuki_Tx(payload, sizeof(payload));
}


/* ===== Section 8: Transmitting commands ===== */

int main(void)
{
    Kobuki_Typedef my_kobuki = {0};
    US_Typedef ultrasonic_object = {0};
    RobotState state = IDLE;

    /* Initialise the drivers. Keep this sequence because it is
       the same as your correctly working ultrasonic program. */
    UART_Init();
    ADC_Init();
    Ultrasonic_Init();

    /* Start in the safe stopped condition. */
    Send_Kobuki_Move(0, 0);

    while (1) {
        Kobuki_Read(&my_kobuki);
        Ultrasonic_Detect(&ultrasonic_object);

        /* Button 1 is the global stop command. It has priority over
           every other state and input. */
        if ((my_kobuki.button & BUTTON_1_STOP) != 0U) {
            state = IDLE;
        }
        else {
            // FSM transition logic
            switch (state) {
            case IDLE:
                // The robot remains stopped until Button 0 is pressed.
                if ((my_kobuki.button &
                     BUTTON_0_START) == 1U) {
                    state = SEARCH;
                }
                break;

            case SEARCH:
                /* The robot rotates while searching. When the ultrasonic sensor
                   detects an object, change to OBJECT. */
                if (ultrasonic_object.detected == 1U) {
                    state = OBJECT;
                }
                break;

            case OBJECT:
                /* The robot drives towards the object. A bumper
                   or cliff input changes the state to OBSTACLE. */
                if ((my_kobuki.bumper == 1U) ||
                    (my_kobuki.cliff == 1U)) {
                    state = OBSTACLE;
                }
                /* If the ultrasonic sensor loses the object,
                   return to SEARCH. */
                else if (ultrasonic_object.detected == 0U) {
                    state = SEARCH;
                }
                break;

            case OBSTACLE:
                /* The robot reverses while the bumper or
                 * cliff input remains active. When both inputs
                   clear, change to AVOID. */
                if ((my_kobuki.bumper == 0U) &&
                    (my_kobuki.cliff == 0U)) {
                    state = AVOID;
                }
                break;

            case AVOID:
                // The timed avoidance movement is applied in the
                // motor-output switch below.
                break;

            default:
                /* Recover safely if the state variable
                   contains an unexpected value. */
                state = IDLE;
                break;
            }
        }

        /* FSM output actions. Exactly one motor command is
           selected according to the current state. */
        switch (state) {
        case IDLE:
            // Stop the robot.
            Send_Kobuki_Move(0, 0);
            break;

        case SEARCH:
            // Rotate counter-clockwise while searching.
            Kobuki_Rotate(SEARCH_SPEED_MM_S);
            break;

        case OBJECT:
            // Drive towards the detected object.
            Kobuki_Drive(FORWARD_SPEED_MM_S);
            break;

        case OBSTACLE:
            // Reverse after bumper or cliff detection.
            Kobuki_Drive(REVERSE_SPEED_MM_S);
            break;

        case AVOID:
            // Rotate to avoid the obstacle and then return to the
            // SEARCH state.
            Kobuki_Rotate(AVOID_SPEED_MM_S);
            AvoidDelay();
            state = SEARCH;
            break;

        default:
            // Safe fallback.
            Send_Kobuki_Move(0, 0);
            state = IDLE;
            break;
        }

        // Small control-loop delay.
        for (volatile uint32_t i = 0U;
             i < 5000U;
             ++i) {
            __NOP();
        }
    }
}



#endif
