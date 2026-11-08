#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

#define ULTRASONIC_MAX_RANGE_MM    ((uint16_t)5000U)

// Object counts as detected below this range (mm). Defaults to 900, can be changed anywhere at runtime
extern uint16_t ultrasonic_detect_range_mm;

typedef struct {
    uint8_t  detected;
    uint16_t range_mm;
} US_Typedef;

void Ultrasonic_Init(void);
uint16_t Ultrasonic_Read(void);
void Ultrasonic_Detect(US_Typedef *object);

#endif
