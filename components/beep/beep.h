#ifndef BEEP_H
#define BEEP_H

#include "buzzer.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

esp_err_t beep_init(int buzzerPin);
void beep(piano_note_t note, uint32_t loudness, float loud_time, float no_loud_time, uint8_t cycle_time);

#endif  // BEEP_H
