#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include <string.h>
#include "hardware/adc.h"

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "hc06.h"

#define DEADZONE 30

QueueHandle_t xQueueAdc;
QueueHandle_t xQueueBTN;

SemaphoreHandle_t xSemaphore;

typedef struct adc {
    int axis;
    int val;
} adc_t;

const uint BTN_1 = 12;
const uint BTN_2 = 13;
const uint BTN_3 = 14;
const uint BTN_4 = 15;
const uint BTN_J = 22;

void btn_callback(uint gpio, uint32_t events) {
    if (events == 0x4) { // fall edge
        xSemaphoreGiveFromISR(xSemaphore, 0);
        printf("pls");
    }
}

void init_pins(){
    gpio_init(BTN_1);
    gpio_set_dir(BTN_1, GPIO_IN);
    gpio_pull_up(BTN_1);

    gpio_init(BTN_2);
    gpio_set_dir(BTN_2, GPIO_IN);
    gpio_pull_up(BTN_2);

    gpio_init(BTN_3);
    gpio_set_dir(BTN_3, GPIO_IN);
    gpio_pull_up(BTN_3);

    gpio_init(BTN_4);
    gpio_set_dir(BTN_4, GPIO_IN);
    gpio_pull_up(BTN_4);

    gpio_set_irq_enabled_with_callback(
        BTN_1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &btn_callback);

    gpio_set_irq_enabled(
        BTN_2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_enabled(
        BTN_3, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_enabled(
        BTN_4, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void x_task(void *p) {
    adc_init();
    adc_gpio_init(27);

    while (1) {
        adc_select_input(1);
        int result = (adc_read() - 2048)/8;
        if (abs(result) > DEADZONE){
           adc_t data = {0, result};
           xQueueSend(xQueueAdc, &data, portMAX_DELAY);
        }
        else {
            adc_t data = {0, 0};
            xQueueSend(xQueueAdc, &data, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void y_task(void *p) {
    adc_init();
    adc_gpio_init(26);

    while (1) {
        adc_select_input(0);
        int result = (adc_read() - 2048)/8;
        if (abs(result) > DEADZONE){
           adc_t data = {1, result};
           xQueueSend(xQueueAdc, &data, portMAX_DELAY);
        }
        else{
            adc_t data = {1, 0};
            xQueueSend(xQueueAdc, &data, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p) {
    adc_t data; 

    while (1) {       
        if(xQueueReceive(xQueueAdc, &data, portMAX_DELAY)){
            int val = data.val/16;
            int msb = val >> 8;
            int lsb = val & 0xFF ;
    
            uart_putc_raw(uart0, data.axis);
            uart_putc_raw(uart0, lsb);
            uart_putc_raw(uart0, msb);
            uart_putc_raw(uart0, -1);
        }
    }
}

void btn_task(void *p){
    int gpio;
    printf("BTN Task\n");

    while(1){
        if (xQueueReceive(xQueueBTN, &gpio, portMAX_DELAY)){
            printf("BTN %d\n", gpio);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void btn_1_task(void *p){
    printf("BTN 1 Task\n");
    gpio_init(BTN_1);
    gpio_set_dir(BTN_1, GPIO_IN);
    gpio_pull_up(BTN_1);

    while(1){
        if (!gpio_get(BTN_1)){
            printf("BTN 1\n");

            while (!gpio_get(BTN_1)) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            xQueueSend(xQueueBTN, &BTN_1, portMAX_DELAY);
        }
        
    }
}

void hc06_task(void *p) {
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);
    gpio_set_function(HC06_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC06_RX_PIN, GPIO_FUNC_UART);
    hc06_init("aps2_legal", "1234");

    while (true) {
        uart_puts(HC06_UART_ID, "OLAAA ");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main() {
    stdio_init_all();
    adc_init();
    //printf("Start RTOS \n");
    //init_pins();    

    xQueueAdc = xQueueCreate(32, sizeof(adc_t));
    xQueueBTN = xQueueCreate(64, sizeof(int) );
    xSemaphore = xSemaphoreCreateBinary();

    //xTaskCreate(hc06_task, "UART_Task 1", 4096, NULL, 1, NULL);
    xTaskCreate(x_task, "x_task", 4095, NULL, 1, NULL);
    xTaskCreate(y_task, "y_task", 4095, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_task", 4095, NULL, 1, NULL);
    //xTaskCreate(btn_1_task, "BTN_Task 1", 256, NULL, 1, NULL);
    //xTaskCreate(btn_task, "btn_task", 4095, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
