#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "mpu6050.h"
#include "hc06.h"

#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

#define DEADZONE 30

#define SHAKE_THRESHOLD 2.3f
#include <Fusion.h>

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 20;
const int I2C_SCL_GPIO = 21;

#define SAMPLE_PERIOD (0.1f)

typedef struct mpu {
    int axis;
    int val;
} mpu_t;

typedef struct adc {
    int axis;
    int val;
} adc_t;


const int BTN_3 = 10; // Mb
const int BTN_2 = 11; // Q
const int BTN_1 = 12; // 2
const int BTN_6 = 13; // Ma
const int BTN_5 = 14; // E
const int BTN_4 = 15; // 3

const int ENCA_PIN = 17;
const int ENCB_PIN = 16;

const int HC_STATUS = 18;
const int LED_STATUS = 19;

QueueHandle_t xQueueHC;
QueueHandle_t xQueueMPU;

SemaphoreHandle_t xSemaphore_1;
SemaphoreHandle_t xSemaphore_2;
SemaphoreHandle_t xSemaphore_3;
SemaphoreHandle_t xSemaphore_4;
SemaphoreHandle_t xSemaphore_5;
SemaphoreHandle_t xSemaphore_6;

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[14];
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 14, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        gyro[i] = (buffer[(i * 2) + 8] << 8 | buffer[(i * 2) + 9]);
    }
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t acceleration[3], gyro[3];

    static TickType_t lastShakeTime = 0;

    while(1) {
        mpu6050_read_raw(acceleration, gyro);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        float magnitude = sqrt(accelerometer.axis.x * accelerometer.axis.x +
                               accelerometer.axis.y * accelerometer.axis.y +
                               accelerometer.axis.z * accelerometer.axis.z);

        TickType_t currentTime = xTaskGetTickCount();

        if (magnitude > SHAKE_THRESHOLD && (currentTime - lastShakeTime) > pdMS_TO_TICKS(3000)) {
            int shakeDetected = 1;
            xQueueSend(xQueueMPU, &shakeDetected, portMAX_DELAY);
            lastShakeTime = currentTime;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void shake_detector_task(void *p) {
    int shakeDetected = 0;

    while (1) {
        if (xQueueReceive(xQueueMPU, &shakeDetected, 1)) {
            int result = 1;
            adc_t data = {8, result};
            xQueueSend(xQueueHC, &data, 1);

        }
    }
}

void btn_callback(uint gpio, uint32_t events) {
    if (events == 0x4 && gpio == BTN_1) { // fall edge
        xSemaphoreGiveFromISR(xSemaphore_1, 0);
    }
    if (events == 0x4 && gpio == BTN_2) {
        xSemaphoreGiveFromISR(xSemaphore_2, 0);
    }
    if (events == 0x4 && gpio == BTN_3) {
        xSemaphoreGiveFromISR(xSemaphore_3, 0);
    }
    if (events == 0x4 && gpio == BTN_4) {
        xSemaphoreGiveFromISR(xSemaphore_4, 0);
    }
    if (events == 0x4 && gpio == BTN_5) {
        xSemaphoreGiveFromISR(xSemaphore_5, 0);
    }
    if (events == 0x4 && gpio == BTN_6) {
        xSemaphoreGiveFromISR(xSemaphore_6, 0);
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

    gpio_init(BTN_5);
    gpio_set_dir(BTN_5, GPIO_IN);
    gpio_pull_up(BTN_5);

    gpio_init(BTN_6);
    gpio_set_dir(BTN_6, GPIO_IN);
    gpio_pull_up(BTN_6);

    gpio_set_irq_enabled_with_callback(
        BTN_1, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &btn_callback);

    gpio_set_irq_enabled(
        BTN_2, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_enabled(
        BTN_3, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_enabled(
        BTN_4, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_enabled(
        BTN_5, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_enabled(
        BTN_6, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void x_task(void *p) {
    adc_init();
    adc_gpio_init(27);

    while (1) {
        adc_select_input(1);
        int result = (adc_read() - 2048)/8;
        if (abs(result) > DEADZONE){
           adc_t data = {1, -result/16};
           xQueueSend(xQueueHC, &data, 1);
        }
        else {
            adc_t data = {1, 0};
            xQueueSend(xQueueHC, &data, 1);
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
           adc_t data = {0, result/16};
           xQueueSend(xQueueHC, &data, 1);
        }
        else{
            adc_t data = {0, 0};
            xQueueSend(xQueueHC, &data, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void btn_task(void *p){

// 2  Q  Mb
// 12 11 10

// 3  E  Ma
// 15 14 13

// single = [
//     uinput.REL_X,
//     uinput.REL_Y,
//     uinput.REL_WHEEL,
// ]

// double = [
//     uinput.BTN_LEFT, 3
//     uinput.KEY_E, 4
//     uinput.KEY_C, 5
//     uinput.KEY_2, 6
//     uinput.KEY_3, 7
//     uinput.KEY_Q 8
// ]

    while(1){
        if (xSemaphoreTake(xSemaphore_1, 1 / portTICK_PERIOD_MS) == pdTRUE ){
            adc_t data = {6, 1}; // APERTAR 2
            xQueueSend(xQueueHC, &data, 1);
        }
        if (xSemaphoreTake(xSemaphore_2, 1 / portTICK_PERIOD_MS) == pdTRUE ){
            adc_t data = {8, 1}; // APERTAR Q
            xQueueSend(xQueueHC, &data, 1);
        }
        if (xSemaphoreTake(xSemaphore_3, 1 / portTICK_PERIOD_MS) == pdTRUE ){
            adc_t data = {3, 1}; // APERTAR Mb
            xQueueSend(xQueueHC, &data, 1);
        }
        if (xSemaphoreTake(xSemaphore_4, 1 / portTICK_PERIOD_MS) == pdTRUE ){
            adc_t data = {7, 1}; // APERTAR 3
            xQueueSend(xQueueHC, &data, 1);
        }
        if (xSemaphoreTake(xSemaphore_5, 1 / portTICK_PERIOD_MS) == pdTRUE ){
            adc_t data = {4, 1}; // APERTAR E
            xQueueSend(xQueueHC, &data, 1);
        }
        if (xSemaphoreTake(xSemaphore_6, 1 / portTICK_PERIOD_MS) == pdTRUE ){
            adc_t data = {3, 1}; // APERTAR Mb
            xQueueSend(xQueueHC, &data, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            data.axis = 4; // APERTAR E
            xQueueSend(xQueueHC, &data, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            data.axis = 5; // APERTAR C
            xQueueSend(xQueueHC, &data, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void rotate_task(void *p) {
    static const int8_t state_table[] = {
        0, -1,  1,  0,
        1,  0,  0, -1,
        -1,  0,  0,  1,
        0,  1, -1,  0
    };
    uint8_t enc_state = 0;
    int last_sum = 0;
    int debounce_counter = 0;

    gpio_init(ENCA_PIN);
    gpio_init(ENCB_PIN);

    gpio_set_dir(ENCA_PIN, GPIO_IN);
    gpio_set_dir(ENCB_PIN, GPIO_IN);

    gpio_pull_up(ENCA_PIN);
    gpio_pull_up(ENCB_PIN);

    adc_t data;
    data.axis = 2;

    while (1) {
        int8_t encoded = (gpio_get(ENCA_PIN) << 1) | gpio_get(ENCB_PIN);
        enc_state = (enc_state << 2) | encoded;
        int sum = state_table[enc_state & 0x0f];

        if (sum != 0) {
            if (sum == last_sum) {
                if (++debounce_counter > 1) {
                    if (sum == 1) {
                        data.val = 0;
                        xQueueSend(xQueueHC, &data, 1);
                        data.val = 1;
                        xQueueSend(xQueueHC, &data, 1);
                    } else if (sum == -1) {
                        data.val = -1;
                        xQueueSend(xQueueHC, &data, 1);
                        data.val = 0;
                        xQueueSend(xQueueHC, &data, 1);
                    }
                    debounce_counter = 0; 
                }
            } else {
                debounce_counter = 0;
            }
            last_sum = sum;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void hc06_task(void *p) {
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);
    gpio_set_function(HC06_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC06_RX_PIN, GPIO_FUNC_UART);
    hc06_init("PALBALLERS", "1234");

    adc_t data;

    while (1) {
        if(xQueueReceive(xQueueHC, &data, 1)){
            int val = data.val;
            int msb = val >> 8;
            int lsb = val & 0xFF;
    
            uart_putc_raw(HC06_UART_ID, data.axis);
            uart_putc_raw(HC06_UART_ID, lsb);
            uart_putc_raw(HC06_UART_ID, msb);
            uart_putc_raw(HC06_UART_ID, -1);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void hc_status_task(void *p) {
    gpio_init(HC_STATUS);
    gpio_set_dir(HC_STATUS, GPIO_IN);

    gpio_init(LED_STATUS);
    gpio_set_dir(LED_STATUS, GPIO_OUT);

    while (1) {
        if (!gpio_get(HC_STATUS)) {
            gpio_put(LED_STATUS, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_put(LED_STATUS, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            gpio_put(LED_STATUS, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}


int main() {
    xQueueHC = xQueueCreate(32, sizeof(adc_t));
    xQueueMPU = xQueueCreate(32, sizeof(mpu_t));

    xSemaphore_1 = xSemaphoreCreateBinary();
    if (xSemaphore_1 == NULL)
      printf("falha em criar o semaforo \n");
    xSemaphore_2 = xSemaphoreCreateBinary();
    if (xSemaphore_2 == NULL)
      printf("falha em criar o semaforo \n");
    xSemaphore_3 = xSemaphoreCreateBinary();
    if (xSemaphore_3 == NULL)
      printf("falha em criar o semaforo \n");
    xSemaphore_4 = xSemaphoreCreateBinary();
    if (xSemaphore_4 == NULL)
      printf("falha em criar o semaforo \n");
    xSemaphore_5 = xSemaphoreCreateBinary();
    if (xSemaphore_5 == NULL)
      printf("falha em criar o semaforo \n");
    xSemaphore_6 = xSemaphoreCreateBinary();
    if (xSemaphore_6 == NULL)
      printf("falha em criar o semaforo \n");
    
    stdio_init_all();
    init_pins();
    adc_init();

    xTaskCreate(mpu6050_task, "mpu6050_Task", 8192, NULL, 1, NULL);
    xTaskCreate(shake_detector_task, "shake_detector_task", 4095, NULL, 1, NULL);
 
    xTaskCreate(x_task, "x_task", 4095, NULL, 1, NULL);
    xTaskCreate(y_task, "y_task", 4095, NULL, 1, NULL);

    xTaskCreate(btn_task, "btn_task", 4095, NULL, 1, NULL);

    xTaskCreate(rotate_task, "rotate_task", 4096, NULL, 1, NULL);

    xTaskCreate(hc06_task, "UART_Task 1", 4096, NULL, 1, NULL);
    xTaskCreate(hc_status_task, "hc_status_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true){
        ;
    }
}