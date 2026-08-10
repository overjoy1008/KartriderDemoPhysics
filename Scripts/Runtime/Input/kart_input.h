#ifndef KART_INPUT_H
#define KART_INPUT_H

#include <stdbool.h>

typedef enum KartSteeringKey {
    KART_STEERING_LEFT = -1,
    KART_STEERING_RIGHT = 1,
} KartSteeringKey;

typedef struct KartSteeringInputState {
    bool left_down;
    bool right_down;
    float value;
} KartSteeringInputState;

/* Recovered input ownership: a new direction press overwrites the steering
   value; releasing a direction clears it only while that direction owns it. */
bool kart_steering_key_event(
    KartSteeringInputState *state,
    KartSteeringKey key,
    bool pressed);

void kart_steering_input_reset(KartSteeringInputState *state);

#endif
