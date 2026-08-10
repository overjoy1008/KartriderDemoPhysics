#include "kart_input.h"

bool kart_steering_key_event(
    KartSteeringInputState *state,
    KartSteeringKey key,
    bool pressed)
{
    bool *down;
    const float value = (float)key;

    if (state == 0 || (key != KART_STEERING_LEFT && key != KART_STEERING_RIGHT)) {
        return false;
    }
    down = key == KART_STEERING_LEFT ? &state->left_down : &state->right_down;
    if (*down == pressed) {
        return false;
    }
    *down = pressed;
    if (pressed) {
        state->value = value;
    } else if (state->value == value) {
        state->value = 0.0f;
    }
    return true;
}

void kart_steering_input_reset(KartSteeringInputState *state)
{
    if (state != 0) {
        state->left_down = false;
        state->right_down = false;
        state->value = 0.0f;
    }
}
