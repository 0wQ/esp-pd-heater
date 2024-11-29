#include "beep.h"

#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// 定义队列和任务句柄
static QueueHandle_t buzzer_queue;
static TaskHandle_t buzzer_task_handle;

// 定义队列的长度
#define BUZZER_QUEUE_LENGTH 5

// 参数结构体，用于传递蜂鸣器配置
typedef struct {
    piano_note_t note;
    uint32_t loudness;
    float loud_time;
    float no_loud_time;
    uint8_t cycle_time;
} buzzer_params_t;

// 蜂鸣器任务：从队列中获取参数并控制蜂鸣器
static void buzzer_task(void *param) {
    buzzer_params_t params;

    while (1) {
        if (xQueueReceive(buzzer_queue, &params, portMAX_DELAY) == pdTRUE) {
            buzzer(params.note, params.loudness, params.loud_time, params.no_loud_time, params.cycle_time);
        }
    }
}

// 初始化蜂鸣器并创建任务的函数
esp_err_t beep_init(int buzzerPin) {
    // 初始化蜂鸣器硬件
    esp_err_t ret = buzzer_init(buzzerPin);
    if (ret != ESP_OK) {
        return ret;
    }

    // 创建队列
    buzzer_queue = xQueueCreate(BUZZER_QUEUE_LENGTH, sizeof(buzzer_params_t));
    if (buzzer_queue == NULL) {
        return ESP_FAIL;
    }

    // 创建蜂鸣器任务
    if (xTaskCreate(buzzer_task, "buzzer_task", 2048, NULL, 5, &buzzer_task_handle) != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

// 将蜂鸣器参数发送到队列
void beep(piano_note_t note, uint32_t loudness, float loud_time, float no_loud_time, uint8_t cycle_time) {
    buzzer_params_t params = {note, loudness, loud_time, no_loud_time, cycle_time};
    // xQueueSend(buzzer_queue, &params, portMAX_DELAY);
    // 队满直接失败
    if (xQueueSend(buzzer_queue, &params, 0) != pdPASS) {
        ESP_LOGW("buzzer", "Queue is full");
    }
}
