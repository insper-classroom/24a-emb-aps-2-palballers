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

#define UART_ID uart0
#define BAUD_RATE 9600

#define SHAKE_THRESHOLD 1.3f
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

const int BTN_1 = 12;
const int BTN_2 = 13;
const int BTN_3 = 14;
const int BTN_4 = 15;
const int BTN_J = 22;

const int ENCA_PIN = 17;
const int ENCB_PIN = 16;

QueueHandle_t xQueueHC;
QueueHandle_t xQueueMPU;

SemaphoreHandle_t xSemaphore_1;
SemaphoreHandle_t xSemaphore_2;
SemaphoreHandle_t xSemaphore_3;
SemaphoreHandle_t xSemaphore_4;

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
    printf("Start MPU task");
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

        //printf("x %0.2f, y %0.2f, z %0.2f\n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);

        float magnitude = sqrt(accelerometer.axis.x * accelerometer.axis.x +
                               accelerometer.axis.y * accelerometer.axis.y +
                               accelerometer.axis.z * accelerometer.axis.z);

        TickType_t currentTime = xTaskGetTickCount();

        //printf("Magnitude: %0.2f\n", magnitude);
        if (magnitude > SHAKE_THRESHOLD && (currentTime - lastShakeTime) > pdMS_TO_TICKS(3000)) {
            int shakeDetected = 1;
            xQueueSend(xQueueMPU, &shakeDetected, portMAX_DELAY);
            lastShakeTime = currentTime;
        }
        
        //printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

        //mpu_t dataX = {0, euler.angle.pitch};
        //xQueueSend(xQueueMPU, &dataX, portMAX_DELAY);

        //mpu_t dataY = {1, -euler.angle.roll};
        //xQueueSend(xQueueMPU, &dataY, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void shake_detector_task(void *p) {
    int shakeDetected = 0;

    while (1) {
        if (xQueueReceive(xQueueMPU, &shakeDetected, 1)) {
            //printf("Shake detected\n");
            int result = 1;
            adc_t data = {2, result};
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
           adc_t data = {0, result/16};
           xQueueSend(xQueueHC, &data, 1);
        }
        else {
            adc_t data = {0, 0};
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
           adc_t data = {1, result/16};
           xQueueSend(xQueueHC, &data, 1);
        }
        else{
            adc_t data = {1, 0};
            xQueueSend(xQueueHC, &data, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void btn_task(void *p){

    while(1){
        if (xSemaphoreTake(xSemaphore_1, 1 / portTICK_PERIOD_MS) == pdTRUE ){
            //printf("BTN 1\n");
            int result = 1;
            adc_t data = {2, result};
            xQueueSend(xQueueHC, &data, 1);
        }
        if (xSemaphoreTake(xSemaphore_2, 1 / portTICK_PERIOD_MS) == pdTRUE ){
            //printf("BTN 2\n");
            int result = 1;
            adc_t data = {3, result};
            xQueueSend(xQueueHC, &data, 1);
        }
        if (xSemaphoreTake(xSemaphore_3, 1 / portTICK_PERIOD_MS) == pdTRUE ){
            //printf("BTN 3\n");
            int result = 1;
            adc_t data = {4, result};
            xQueueSend(xQueueHC, &data, 1);
        }
        if (xSemaphoreTake(xSemaphore_4, 1 / portTICK_PERIOD_MS) == pdTRUE ){
            //printf("BTN 4\n");
            int result = 1;
            adc_t data = {5, result};
            xQueueSend(xQueueHC, &data, 1);
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
    uint8_t enc_state = 0; // Current state of the encoder
    int last_sum = 0; // Last non-zero sum to filter out noise
    int debounce_counter = 0; // Debounce counter

    //adc_t data;

    // Initialize GPIO pins for the encoder
    gpio_init(ENCA_PIN);
    gpio_init(ENCB_PIN);

    gpio_set_dir(ENCA_PIN, GPIO_IN);
    gpio_set_dir(ENCB_PIN, GPIO_IN);

    gpio_pull_up(ENCA_PIN);  // Enable internal pull-up
    gpio_pull_up(ENCB_PIN);  // Enable internal pull-up

    adc_t data;
    data.axis = 6;

    while (1) {
        int8_t encoded = (gpio_get(ENCA_PIN) << 1) | gpio_get(ENCB_PIN);
        enc_state = (enc_state << 2) | encoded;
        int sum = state_table[enc_state & 0x0f];

        if (sum != 0) {
            if (sum == last_sum) {
                if (++debounce_counter > 1) {  // Check if the same movement is read consecutively
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
                    debounce_counter = 0;  // Reset the counter after confirming the direction
                }
            } else {
                debounce_counter = 0;  // Reset the counter if the direction changes
            }
            last_sum = sum;  // Update last_sum to the current sum
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Poll every 1 ms to improve responsiveness
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

    stdio_init_all();
    printf("Start RTOS \n");
    init_pins();
    adc_init();

    xTaskCreate(mpu6050_task, "mpu6050_Task", 8192, NULL, 1, NULL);
    xTaskCreate(shake_detector_task, "shake_detector_task", 4095, NULL, 1, NULL);

    xTaskCreate(x_task, "x_task", 4095, NULL, 1, NULL);
    xTaskCreate(y_task, "y_task", 4095, NULL, 1, NULL);

    xTaskCreate(btn_task, "btn_task", 4095, NULL, 1, NULL);

    xTaskCreate(rotate_task, "rotate_task", 4096, NULL, 1, NULL);

    xTaskCreate(hc06_task, "UART_Task 1", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true){
        ;
    }
}