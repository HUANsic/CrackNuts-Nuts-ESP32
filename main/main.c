#include <stdio.h>
#include "driver/uart.h"
#include "NutsLib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TaskHandle_t task_nut;

void app_nut(void *pvParameters)
{
    while (1)
    {
        Nut_Loop();
    }
}

void app_main(void)
{
    Nut_Init();

    xTaskCreatePinnedToCore(
        app_nut,          /* Function that implements the task. */
        "nut",            /* Text name for the task. */
        2048,             /* Stack size in words, or bytes. */
        NULL,             /* Parameter passed into the task. */
        tskIDLE_PRIORITY, /* Priority at which the task is created. */
        &task_nut,        /* Used to pass out the created task's handle. */
        0);               /* Core ID */
}
