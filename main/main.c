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
#define BAUD_RATE 115200

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

QueueHandle_t xQueueAdc;
QueueHandle_t xQueueBTN;
QueueHandle_t xQueueMPU;

SemaphoreHandle_t xSemaphore_1;

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
        
        printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

        //mpu_t dataX = {0, euler.angle.pitch};
        //xQueueSend(xQueueMPU, &dataX, portMAX_DELAY);

        //mpu_t dataY = {1, -euler.angle.roll};
        //xQueueSend(xQueueMPU, &dataY, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void btn_callback(uint gpio, uint32_t events) {
    if (events == 0x4) { // fall edge
        xSemaphoreGiveFromISR(xSemaphore_1, 0);
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
           xQueueSend(xQueueAdc, &data, 1);
        }
        else {
            adc_t data = {0, 0};
            xQueueSend(xQueueAdc, &data, 1);
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
           xQueueSend(xQueueAdc, &data, 1);
        }
        else{
            adc_t data = {1, 0};
            xQueueSend(xQueueAdc, &data, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void uart_task(void *p) {
    adc_t data; 

    while (1) {       
        if(xQueueReceive(xQueueAdc, &data, 1)){
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
        if (xSemaphoreTake(xSemaphore_1, 1 / portTICK_PERIOD_MS) == pdTRUE ){
            printf("BTN 1\n");
        }
        vTaskDelay(pdMS_TO_TICKS(50));
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
    
    xQueueAdc = xQueueCreate(32, sizeof(adc_t));
    xQueueBTN = xQueueCreate(64, sizeof(int));
    xQueueMPU = xQueueCreate(32, sizeof(mpu_t));
    xSemaphore_1 = xSemaphoreCreateBinary();
    if (xSemaphore_1 == NULL)
      printf("falha em criar o semaforo \n");

    stdio_init_all();
    printf("Start RTOS \n");
    init_pins();
    adc_init();

    //xTaskCreate(hc06_task, "UART_Task 1", 4096, NULL, 1, NULL);

    xTaskCreate(mpu6050_task, "mpu6050_Task", 8192, NULL, 1, NULL);

    //xTaskCreate(x_task, "x_task", 4095, NULL, 1, NULL);
    //xTaskCreate(y_task, "y_task", 4095, NULL, 1, NULL);
    //xTaskCreate(uart_task, "uart_task", 4095, NULL, 1, NULL);

    //xTaskCreate(btn_1_task, "BTN_Task 1", 256, NULL, 1, NULL);
    //xTaskCreate(btn_task, "btn_task", 4095, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true){
        ;
    }
}