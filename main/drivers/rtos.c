#include "rtos.h"

void rtos_init() {
  xTaskCreatePinnedToCore(task_LED1, "task1", 1024 * 2, NULL, 4, NULL, app_cpu);
  xTaskCreatePinnedToCore(task_LED2, "task2", 1024 * 2, NULL, 3, NULL, app_cpu);
}

void task_LED1(void *param) {
  while (1) {
    printf("this is task 1\n");
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void task_LED2(void *param) {
  while (1) {
    printf("this is task 2\n");
  }
  vTaskDelete(NULL);
}
