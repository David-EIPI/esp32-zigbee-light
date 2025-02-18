
#pragma once

#include <stdint.h>

//extern volatile int32_t motionTimeout;
extern volatile int32_t motionThreshold;
extern volatile int motionDetected;
extern volatile uint32_t ld2420_Distance;

void ld2420_task(void *arg);

