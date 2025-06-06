// Compiler: GNU GCC for ESP32
// Target: ESP32 microcontroller
// Libraries: ESP-IDF framework libraries for ESP32
// Project structure: The project is structured with a main source file and includes necessary headers.
// Build system: The project uses CMake as the build system.
// This code is a simple example of controlling two stepper motors using RMT (Remote Control) and UART on an ESP32 microcontroller.
// It includes functionalities for motor control, UART communication, and LED control.
// The code initializes the motors, sets their speed and direction, and processes commands received via UART.
// It also includes a leak sensor task that monitors a GPIO pin for leak detection and sends notifications when a leak is confirmed.
// This code is designed to run on an ESP32 microcontroller using the ESP-IDF framework.
// SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
// SPDX-License-Identifier: Apache-2.0
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include "esp_log.h"
 #include "esp_system.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/event_groups.h"
 #include "driver/rmt_tx.h"
 #include "driver/gpio.h"
 #include "esp_log.h"
 #include "stepper_motor_encoder.h"
 #include "driver/ledc.h"
 #include "esp_err.h"
 
 
 
 #include "esp_system.h"
 #include "driver/uart.h"
 
 #include <inttypes.h>
 #include "nvs_flash.h"
 #include "nvs.h"
 
 // Motor 1
 #define MOTOR1_STEP_MOTOR_GPIO_EN       14
 #define MOTOR1_STEP_MOTOR_GPIO_DIR      12
 #define MOTOR1_STEP_MOTOR_GPIO_STEP     13
 #define MOTOR1_STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
 #define MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
 #define MOTOR1_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE
 #define MOTOR1_STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution
 
 // Motor 2
 #define MOTOR2_STEP_MOTOR_GPIO_EN       6
 #define MOTOR2_STEP_MOTOR_GPIO_DIR      4
 #define MOTOR2_STEP_MOTOR_GPIO_STEP     5
 #define MOTOR2_STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
 #define MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE 1
 #define MOTOR2_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE
 #define MOTOR2_STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution
 
 
 #define TXD1_PIN (GPIO_NUM_17)
 #define RXD1_PIN (GPIO_NUM_18)
 #define TXD2_PIN (GPIO_NUM_15)
 #define RXD2_PIN (GPIO_NUM_16)
 
 #define EN_PIN1 (GPIO_NUM_36)
 #define EN_PIN2 (GPIO_NUM_37)
 
 #define FORWORD    1
 #define BACKWORD    2
 
 #define LEFT 3
 #define RIGHT 4
 #define STOP 0
 
 #define LEDC_TIMER              LEDC_TIMER_0
 #define LEDC_MODE               LEDC_LOW_SPEED_MODE
 #define LEDC_OUTPUT_IO          (2) // Define the output GPIO
 #define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
 #define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
 #define LEDC_FREQUENCY          (50) // Frequency in Hertz. Set frequency at 4 kHz
 #define LEDC_OP_PIN0             9
 #define LEDC_OP_PIN1             10
 
 #define LEDC_TEST_CH_NUM       (8)
 // uint32_t DutyCycle[LEDC_TEST_CH_NUM] = {1024,2048,1024,2048};
 // uint32_t DutyCycle[LEDC_TEST_CH_NUM] = {512,1024,2048,3072,4096,5120,6144,7168};
 
 //Leak sensor configuration
 #define LEAK_SENSOR_GPIO 38 
 #define LEAK_NOTIFICATION_INTERVAL_MS 5000  // 5-second interval between notifications
 #define LEAK_CHECK_INTERVAL_MS 1000  // 1 second between checks
 #define LEAK_DETECTION_THRESHOLD 7   // Number of consecutive detections
 
 int32_t DutyCycle = 0;
 int32_t DutyCycle2 = 0;
 int32_t CurrentDuty= 0;
 int32_t CurrentDuty2 = 0;
 
 static const int RX_BUF_SIZE = 1024;
 // static const int TX_BUF_SIZE = 8;
 char buffer[17];
 char responseData[1024];
 int32_t SPEED = 200;
 volatile int cmdFlag = 0, state = 0,previousState = 0;;
 
 volatile bool flagMotorRUN = 0,errorFlag = 0,flagCamRUN = 0,serialFlag = 0, accelFlag = 1;
 
 static const char *TAG = "example";
 rmt_channel_handle_t motor1_chan = NULL;
 rmt_channel_handle_t motor2_chan = NULL;
 nvs_handle_t my_handle;
 
 int sendData(const char* logName, const char* data);
 int sendData1(const char* logName, const char* data);
 
 void motorInit(void);
 void motorRunFwd(void);
 void motorRunBwd(void);
 void motor1RunFwd(void);
 void motor2RunFwd(void);
 void motorSTOP(void);
 void saveSPEED(void);
 void saveBRIGHT(void);
 void check_duty_cycle(void);
 void leak_sensor_task(void *pvParameters);
 
 
 
 void leak_sensor_task(void *pvParameters) {
     // GPIO configuration
     /*
     gpio_config_t io_conf = {
         .intr_type = GPIO_INTR_DISABLE,
         .mode = GPIO_MODE_INPUT,
         .pin_bit_mask = (1ULL << LEAK_SENSOR_GPIO),
         .pull_down_en = GPIO_PULLDOWN_DISABLE,
         .pull_up_en = GPIO_PULLUP_ENABLE
     };
     gpio_config(&io_conf);
     */
    gpio_config_t io_conf = {
     .intr_type = GPIO_INTR_DISABLE,
     .mode = GPIO_MODE_INPUT,
     .pin_bit_mask = (1ULL << LEAK_SENSOR_GPIO),
     .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Enable pull-down resistor
     .pull_up_en = GPIO_PULLUP_DISABLE      // Disable pull-up resistor
 };
 gpio_config(&io_conf);
     uint8_t leak_count = 0;
 
     while (1) {
         // Check leak sensor state
         bool leak_detected = gpio_get_level(LEAK_SENSOR_GPIO) == 1;
 
         if (leak_detected) {
             leak_count++;
             
             if (leak_count >= LEAK_DETECTION_THRESHOLD) {
                 // Send leak notification
                 sprintf(responseData, "{\"cmd\":\"alert\",\"type\":\"leak\",\"status\":\"confirmed\"}\r\n");
                 sendData("LEAK_ALERT", responseData);
                 sendData1("LEAK_ALERT", responseData);
                 
                 // Reset leak count to prevent repeated notifications
                 leak_count = 0;
             }
         } else {
             // Reset leak count if no leak is detected
             leak_count = 0;
         }
 
         vTaskDelay(pdMS_TO_TICKS(LEAK_CHECK_INTERVAL_MS));
     }
 }
 
 
 
 
 void processCommand(char dataArray[]) {
 
     if (('c' == dataArray[0]) && ('f' == dataArray[1]) && ('g' == dataArray[2]) && (' ' == dataArray[3])) {
         if (('s' == dataArray[4]) && ('p' == dataArray[5]) && ('d' == dataArray[6]) && (' ' == dataArray[7])) {
             uint32_t speedinput = atoi(&dataArray[8]); 
             // printf("Setting speed to %ld\n", speedinput);
             SPEED= speedinput;
             saveSPEED();
             // lcd_clear();
             sprintf(responseData,"{\"cmd\":\"cfg\",\"value\":{\"spd\":\"success\"}}\r\n");
             sendData("TX_DATA",responseData);
             sendData1("TX_DATA",responseData);
 
             
         }
         else if (('b' == dataArray[4]) && ('r' == dataArray[5]) && ('t' == dataArray[6]) && (' ' == dataArray[7])) {
             uint32_t brightInput = atoi(&dataArray[8]); 
             // printf("Setting speed to %ld\n", speedinput);
             DutyCycle= brightInput;
            
             saveBRIGHT();
             // lcd_clear();
             sprintf(responseData,"{\"cmd\":\"cfg\",\"value\":{\"brt\":\"success\"}}\r\n");
             sendData("TX_DATA",responseData);
             sendData1("TX_DATA",responseData);
 
             
         }
         else
          {
             // printf("Invalid command\n");
         
             sprintf(responseData,"{\"cmd\":\"cfg\",\"value\":\"invalid\"}\r\n");        
             sendData("TX_DATA",responseData);
             sendData1("TX_DATA",responseData);
 
 
 
         }
     }
     else if (('a' == dataArray[0]) && ('c' == dataArray[1]) && ('t' == dataArray[2]) && (' ' == dataArray[3])) {
         if (('f' == dataArray[4]) && ('w' == dataArray[5]) && ('d' == dataArray[6])) {
             cmdFlag = 1;
             serialFlag =1;
             // printf("Activated Forward"); 
             sprintf(responseData,"{\"cmd\":\"act\",\"value\":{\"fwd\":\"success\"}}\r\n");
             sendData("TX_DATA",responseData);
             sendData1("TX_DATA",responseData);
 
 
         }
         else if (('g' == dataArray[4]) && ('r' == dataArray[5]) && ('a' == dataArray[6]) && ('b' == dataArray[7])) {
            int32_t duty = atoi(&dataArray[8]);
            DutyCycle2= duty;
            cmdFlag = 0;
            serialFlag =1;
            sprintf(responseData,"{\"cmd\":\"act\",\"value\":{\"grab\":\"success\"}}\r\n");
            sendData("TX_DATA",responseData);
            sendData1("TX_DATA",responseData);
            }
         else if (('b' == dataArray[4]) && ('w' == dataArray[5]) && ('d' == dataArray[6])) {
             cmdFlag = 2;
             serialFlag =1;
             // printf("Activated Backward");
             sprintf(responseData,"{\"cmd\":\"act\",\"value\":{\"bwd\":\"success\"}}\r\n");
             sendData("TX_DATA",responseData);
             sendData1("TX_DATA",responseData);
 
 
         }
         else if (('r' == dataArray[4]) && ('i' == dataArray[5]) && ('g' == dataArray[6]) && ('h' == dataArray[7])&&('t' == dataArray[8])) {
             cmdFlag = 3;
             serialFlag =1;
             sprintf(responseData,"{\"cmd\":\"act\",\"value\":{\"right\":\"success\"}}\r\n");
             sendData("TX_DATA",responseData);
             sendData1("TX_DATA",responseData);
 
         }
            else if (('l' == dataArray[4]) && ('e' == dataArray[5]) && ('f' == dataArray[6]) && ('t' == dataArray[7])) {
             cmdFlag = 4;
             serialFlag =1;
             sprintf(responseData,"{\"cmd\":\"act\",\"value\":{\"left\":\"success\"}}\r\n");
             sendData("TX_DATA",responseData);
             sendData1("TX_DATA",responseData);
 
         }
         else if (('s' == dataArray[4]) && ('t' == dataArray[5]) && ('o' == dataArray[6]) && ('p' == dataArray[7])) {
             cmdFlag = 0;
             serialFlag =1;
             sprintf(responseData,"{\"cmd\":\"act\",\"value\":{\"stop\":\"success\"}}\r\n");
             sendData("TX_DATA",responseData);
             sendData1("TX_DATA",responseData);
             }
         else {
             // printf("Invalid command\n");
 
             sprintf(responseData,"{\"cmd\":\"act\",\"value\":\"invalid\"}\r\n");
             sendData("TX_DATA",responseData);
             sendData1("TX_DATA",responseData);
 
     }
     }
     else if (('i' == dataArray[0]) && ('n' == dataArray[1]) && ('f' == dataArray[2]) && ('o' == dataArray[3])&& (' ' == dataArray[4])) {
         if (('s' == dataArray[5]) && ('p' == dataArray[6]) && ('d' == dataArray[7])) {
     
             sprintf(responseData,"{\"cmd\":\"info\",\"value\":{\"spd\": %ld}}\r\n", SPEED);
             sendData("TX_DATA",responseData);
             sendData1("TX_DATA",responseData);
           
         }
         // else if (('p' == dataArray[5]) && ('a' == dataArray[6]) && ('r' == dataArray[7]) && ('a' == dataArray[8])&& ('m' == dataArray[9])) {
         //     sprintf(responseData,"{\"cmd\":\"info\",\"value\":{\"spd\": %ld,\"time\":%d,\"camr\":%ld,\"pulr\":%ld}}\r\n",winchSPEED,stoptime,camRatio,pulseRate);
         //     sendData("TX_DATA",responseData);
 
 
         // }
         else {
             // printf("Invalid command\n");
 
             sprintf(responseData,"{\"cmd\":\"info\",\"value\":\"invalid\"}\r\n");
             sendData("TX_DATA",responseData);
             sendData1("TX_DATA",responseData);
 
         }
     }
     
     else {
         // printf("Invalid command\n");
         sprintf(responseData,"{\"cmd\":\"invalid\"}\r\n");
         sendData("TX_DATA",responseData);
         sendData1("TX_DATA",responseData);
 
     }
    
 }
 
 void initUART(void)
 {
     const uart_config_t uart_config = {
         .baud_rate = 115200,
         .data_bits = UART_DATA_8_BITS,
         .parity = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_1,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
         .source_clk = UART_SCLK_DEFAULT,
     };
     // We won't use a buffer for sending data.
     uart_driver_install(UART_NUM_0, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
     uart_param_config(UART_NUM_0, &uart_config);
 }
 void initUART1(void)
 {
     const uart_config_t uart_config = {
         .baud_rate = 115200,
         .data_bits = UART_DATA_8_BITS,
         .parity = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_1,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
         .source_clk = UART_SCLK_DEFAULT,
     };
     // We won't use a buffer for sending data.
     uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
     uart_param_config(UART_NUM_1, &uart_config);
     uart_set_pin(UART_NUM_1, TXD1_PIN, RXD1_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
 }
 void initUART2(void)
 {
     const uart_config_t uart_config = {
         .baud_rate = 38400,
         .data_bits = UART_DATA_8_BITS,
         .parity = UART_PARITY_DISABLE,
         .stop_bits = UART_STOP_BITS_1,
         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
         .source_clk = UART_SCLK_DEFAULT,
     };
     // We won't use a buffer for sending data.
     uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
     uart_param_config(UART_NUM_2, &uart_config);
     uart_set_pin(UART_NUM_2, TXD2_PIN, RXD2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
 }
 
 int sendData(const char* logName, const char* data)
 {
     const int len = strlen(data);
     const int txBytes = uart_write_bytes(UART_NUM_0, data, len);
     // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
     return txBytes;
 }
 int sendData1(const char* logName, const char* data)
 {
     const int len = strlen(data);
     const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
     // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
     return txBytes;
 }
 // int sendData1(const char* logName, unsigned const char* data,const int len)
 // {
 //     // const int len = strlen(data);
 //     const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
 //     // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
 //     return txBytes;
 // }
 int sendData2(const char* logName, unsigned const char* data,const int len)
 {
     // const int len = strlen(data);
     const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
     // ESP_LOGI(logName, "Wrote %d bytes", txBytes);
     return txBytes;
 }
 static void tx_task(void *arg)
 {
     static const char *TX_TASK_TAG = "TX_TASK";
     esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
     while (1) {
         sendData(TX_TASK_TAG, "Hello world");
         vTaskDelay(2000 / portTICK_PERIOD_MS);
     }
 }
 
 static void rx_task(void *arg)
 {
     static const char *RX_TASK_TAG = "CONFIGURE";
     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
     uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
     while (1) {
         const int rxBytes = uart_read_bytes(UART_NUM_0, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
         if (rxBytes > 0) {
             data[rxBytes] = 0;
             ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
             // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
 
             char dataArray [20];
             memcpy(dataArray,data, rxBytes);
 
             processCommand(dataArray);
 
 
         }
     }
     free(data);
 }
 
 static void rx_task1(void *arg)
 {
     static const char *RX_TASK_TAG = "CONFIGURE";
     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
     uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
     while (1) {
         const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
         if (rxBytes > 0) {
             data[rxBytes] = 0;
             ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
             // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
 
             char dataArray [20];
             memcpy(dataArray,data, rxBytes);
 
             processCommand(dataArray);
 
 
         }
     }
     free(data);
 }
 
 void pwmLEDC(void *arg)
 {
     int ch;
      // Configure GPIOs for both LED channels with pull-down resistors
     gpio_config_t io_conf = {
         .pin_bit_mask = (1ULL << LEDC_OP_PIN0) | (1ULL << LEDC_OP_PIN1), // Configure both LED pins
         .mode = GPIO_MODE_OUTPUT,                 // Set as output mode
         .pull_down_en = GPIO_PULLDOWN_ENABLE,     // Enable internal pull-down resistor
         .pull_up_en = GPIO_PULLUP_DISABLE,        // Disable pull-up resistor
         .intr_type = GPIO_INTR_DISABLE            // Disable interrupts
     };
     gpio_config(&io_conf);
 
     // Configure the LEDC timer
     ledc_timer_config_t ledc_timer = {
         .speed_mode       = LEDC_MODE,
         .duty_resolution  = LEDC_DUTY_RES,
         .timer_num        = LEDC_TIMER,
         .freq_hz          = LEDC_FREQUENCY,
         .clk_cfg          = LEDC_AUTO_CLK
     };
     ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
 
     // Configure two LEDC channels on GPIO 2 and GPIO 3
    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OP_PIN1 ,
        .duty           = LEDC_DUTY, // Set duty to 0%
        .hpoint         = 0
        },        
        {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OP_PIN0 ,
        .duty           = LEDC_DUTY, // Set duty to 0%
        .hpoint         = 0
        }        
    };
     // Apply the LEDC channel configurations
     for (ch = 0; ch < 2; ch++) {
         ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[ch]));
     }
 
     // Main task loop: Update duty cycles periodically
     while (true){
        if (DutyCycle!=CurrentDuty)
        {
            // for (ch = 1; ch < LEDC_TEST_CH_NUM; ch++) {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, ledc_channel[0].channel, DutyCycle));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, ledc_channel[0].channel));
            // }
            CurrentDuty= DutyCycle;
        }
        if (DutyCycle2!=CurrentDuty2)
        {
            // for (ch = 1; ch < LEDC_TEST_CH_NUM; ch++) {
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, ledc_channel[1].channel, DutyCycle2));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, ledc_channel[2].channel));
            // }
            CurrentDuty2= DutyCycle2;
        }
            vTaskDelay(10/ portTICK_PERIOD_MS);
            // ESP_LOGI(TAG, "LEDC task");

    }
 }
 
 
     
 
 void saveSPEED()    
 {                                       
     esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
     if (err != ESP_OK) {
         // printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
     } else {
         // printf("Done\n");
 
     err = nvs_set_i32(my_handle, "SPEED",SPEED);
     // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
 
 
     // printf("Committing updates in NVS ... ");
     err = nvs_commit(my_handle);
     // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
 
     // Close
     nvs_close(my_handle);
     vTaskDelay(100 / portTICK_PERIOD_MS);
 
     }
 }
 void saveBRIGHT()
 {
     esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
     if (err != ESP_OK) {
         // printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
     } else {
         // printf("Done\n");
 
     err = nvs_set_i32(my_handle, "BRIGHT",DutyCycle);
     // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
 
 
     // printf("Committing updates in NVS ... ");
     err = nvs_commit(my_handle);
     // printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
 
     // Close
     nvs_close(my_handle);
     vTaskDelay(100 / portTICK_PERIOD_MS);
 
     }
 }
 static void remote_read_task(void *pvParameter)
 {
 
     // ===========Init stepper motor ==========================================
 
         const static uint32_t accel_samples = 100;
         uint32_t uniform_speed_hz = 200;
         const static uint32_t decel_samples = 100;
 
         
         ESP_LOGI(TAG, "Initialize EN + DIR GPIO for Motor 1");
         gpio_config_t motor_en_dir_gpio_config = {
             .mode = GPIO_MODE_OUTPUT,
             .intr_type = GPIO_INTR_DISABLE,
             .pin_bit_mask = (1ULL << MOTOR1_STEP_MOTOR_GPIO_DIR) | (1ULL << MOTOR1_STEP_MOTOR_GPIO_EN|1ULL << MOTOR2_STEP_MOTOR_GPIO_DIR) | (1ULL << MOTOR2_STEP_MOTOR_GPIO_EN),
         };
         ESP_ERROR_CHECK(gpio_config(&motor_en_dir_gpio_config));
 
 
         ESP_LOGI(TAG, "Create RMT TX channel for Motor 1");
         rmt_tx_channel_config_t motor1_tx_chan_config = {
             .clk_src = RMT_CLK_SRC_DEFAULT,
             .gpio_num = MOTOR1_STEP_MOTOR_GPIO_STEP,
             .mem_block_symbols = 64,
             .resolution_hz = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
             .trans_queue_depth = 10,
         };
         ESP_ERROR_CHECK(rmt_new_tx_channel(&motor1_tx_chan_config, &motor1_chan));
 
         ESP_LOGI(TAG, "Create RMT TX channel for Motor 2");
         rmt_tx_channel_config_t motor2_tx_chan_config = {
             .clk_src = RMT_CLK_SRC_DEFAULT,
             .gpio_num = MOTOR2_STEP_MOTOR_GPIO_STEP,
             .mem_block_symbols = 64,
             .resolution_hz = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
             .trans_queue_depth = 10,
         };
         ESP_ERROR_CHECK(rmt_new_tx_channel(&motor2_tx_chan_config, &motor2_chan));
 
         ESP_LOGI(TAG, "Enable Motor 1");
         gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_EN, MOTOR1_STEP_MOTOR_ENABLE_LEVEL);
 
         ESP_LOGI(TAG, "Enable Motor 2");
         gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_EN, MOTOR2_STEP_MOTOR_ENABLE_LEVEL);
 
         ESP_LOGI(TAG, "Create motor encoders for Motor 1");
         stepper_motor_curve_encoder_config_t motor1_accel_encoder_config = {
             .resolution = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
             .sample_points = 100,
             .start_freq_hz = 0,
             .end_freq_hz = 100,
         };
         rmt_encoder_handle_t motor1_accel_encoder = NULL;
         ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor1_accel_encoder_config, &motor1_accel_encoder));
 
         stepper_motor_uniform_encoder_config_t motor1_uniform_encoder_config = {
             .resolution = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
         };
         rmt_encoder_handle_t motor1_uniform_encoder = NULL;
         ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&motor1_uniform_encoder_config, &motor1_uniform_encoder));
 
         stepper_motor_curve_encoder_config_t motor1_decel_encoder_config = {
             .resolution = MOTOR1_STEP_MOTOR_RESOLUTION_HZ,
             .sample_points = 100,
             .start_freq_hz = 100,
             .end_freq_hz = 0,
         };
         rmt_encoder_handle_t motor1_decel_encoder = NULL;
         ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor1_decel_encoder_config, &motor1_decel_encoder));
 
         ESP_LOGI(TAG, "Create motor encoders for Motor 2");
         stepper_motor_curve_encoder_config_t motor2_accel_encoder_config = {
             .resolution = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
             .sample_points = 100,
             .start_freq_hz = 0,
             .end_freq_hz = 100,
         };
         rmt_encoder_handle_t motor2_accel_encoder = NULL;
         ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor2_accel_encoder_config, &motor2_accel_encoder));
 
         stepper_motor_uniform_encoder_config_t motor2_uniform_encoder_config = {
             .resolution = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
         };
         rmt_encoder_handle_t motor2_uniform_encoder = NULL;
         ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&motor2_uniform_encoder_config, &motor2_uniform_encoder));
 
         stepper_motor_curve_encoder_config_t motor2_decel_encoder_config = {
             .resolution = MOTOR2_STEP_MOTOR_RESOLUTION_HZ,
             .sample_points = 100,
             .start_freq_hz = 100,
             .end_freq_hz = 0,
         };
         rmt_encoder_handle_t motor2_decel_encoder = NULL;
         ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&motor2_decel_encoder_config, &motor2_decel_encoder));
 
         ESP_LOGI(TAG, "Enable RMT channels");
         ESP_ERROR_CHECK(rmt_enable(motor1_chan));
         ESP_ERROR_CHECK(rmt_enable(motor2_chan));
 
         ESP_LOGI(TAG, "Spin motors for 6000 steps: 500 accel + 5000 uniform + 500 decel");
         rmt_transmit_config_t tx_config = {
             .loop_count = 0,
         };
     //=========================================================================
     //==================wait for serial events============================================
 
 
     while(true) 
     {
         previousState = state;
         state = cmdFlag;
         // ESP_LOGI(TAG, "serialFlag =  %d", serialFlag);
         if  (1  == serialFlag ) {
             ESP_LOGI(TAG, "Serial_events %d", state);
             serialFlag = 0;
         }
         // printf("status= %d",state);
 
             uniform_speed_hz = SPEED;
             switch (state)
             {
                 case RIGHT: ESP_LOGD(TAG, "CAM_RIGHT CAM PRESSED");                      
                   {
                         if(previousState != state){
                             //  if(1==flagMotorRUN)  
                             //  {
                                 tx_config.loop_count = 0;
 
                                 ESP_LOGI(TAG, "Set spin direction for Motor 2 ");
                                 gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_DIR, MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
 
                             //   }
                               if ((0 == flagMotorRUN) && (1 == accelFlag) )
                               {  
                                 tx_config.loop_count = 0;
 
                                 ESP_LOGI(TAG, "Set spin direction for Motor 2");
                                 gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_DIR, MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
 
                                 ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_accel_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
                                 flagMotorRUN=1;
                               }
                             }
                             
                             
                             tx_config.loop_count = 1;
                             ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_uniform_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
 
                             ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor2_chan, -1));
                             flagMotorRUN = 1;
                             
                         }
 
                         break;
                 case LEFT: ESP_LOGD(TAG, "CAM_LEFT CAM PRESSED");
 
                         {
                             if(previousState != state){
                             //    if(1==flagMotorRUN)
                             //    {
                                 tx_config.loop_count = 0;
                                 ESP_LOGI(TAG, "Set spin direction for Motor 1");
                                 gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_DIR, MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
                             //   }
                             if ((0 == flagMotorRUN) && (1 == accelFlag) )
                             {
                                 tx_config.loop_count = 0;
 
                                 ESP_LOGI(TAG, "Set spin direction for Motor 1");
                                 gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_DIR, MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
 
                                 ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_accel_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
                                 flagMotorRUN=1;
                               }
                                 
 
                             }
                             
                             tx_config.loop_count = 1;
                             ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_uniform_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
 
                             ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor1_chan, -1));
                             
                             
                         }
                         break;
                 case FORWORD: ESP_LOGD(TAG,"FORWARD");
                         {
                             if(previousState != state){
                         
                                 tx_config.loop_count = 0;
                                 ESP_LOGI(TAG, "Set spin direction for Motor 1");
                                 gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_DIR, MOTOR1_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
 
                                 ESP_LOGI(TAG, "Set spin direction for Motor 2");
                                 gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_DIR, MOTOR2_STEP_MOTOR_SPIN_DIR_CLOCKWISE);
                                 if (0 == flagMotorRUN)
                                 {
                                     ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_accel_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
                                     ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_accel_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
                                 }
                                 flagMotorRUN=1;
 
                             }
                             
                                 tx_config.loop_count = 1;
                                 ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_uniform_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
                                 ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_uniform_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
 
                                 ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor1_chan, -1));
                                 ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor2_chan, -1));
                             
                             
                         }
                         
                         break;                                                
                 case BACKWORD: ESP_LOGD(TAG,"Backward");
                     {
                             if(previousState != state){
                             
                                 tx_config.loop_count = 0;
                                 ESP_LOGI(TAG, "Set spin direction for Motor 1");
                                 gpio_set_level(MOTOR1_STEP_MOTOR_GPIO_DIR, MOTOR1_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
 
                                 ESP_LOGI(TAG, "Set spin direction for Motor 2");
                                 gpio_set_level(MOTOR2_STEP_MOTOR_GPIO_DIR, MOTOR2_STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
                                 if (0 == flagMotorRUN)
                                 {
                                     ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_accel_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
                                     ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_accel_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
                                 }
                                 flagMotorRUN=1;
 
                             }
                             
                             tx_config.loop_count = 1;
                             ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_uniform_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
                             ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_uniform_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
 
                             ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor1_chan, -1));
                             ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor2_chan, -1));
                             
                             
                         }
                         
                         break;          
                 case STOP:ESP_LOGD(TAG,"Stop");
                         if(previousState != state){
                             if(1==flagMotorRUN)
                             {
                             tx_config.loop_count = 0;
                             ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_decel_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
                             ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_decel_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
 
                             ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor1_chan, -1));
                             ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor2_chan, -1));
                             
                             }
                         }
                         flagMotorRUN=0;
                         break;  
                 default: ESP_LOGD(TAG,"Undefined state");
                         if(previousState != state){
                             if(1==flagMotorRUN)
                             {
                             tx_config.loop_count = 0;
                             ESP_ERROR_CHECK(rmt_transmit(motor1_chan, motor1_decel_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
                             ESP_ERROR_CHECK(rmt_transmit(motor2_chan, motor2_decel_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
 
                             ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor1_chan, -1));
                             ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor2_chan, -1));
                             
                             }
                         }
                         flagMotorRUN=0;
                         break;  
 
             }
 
     }
 
 }
 
 
 
 
 void app_main(void)
 {
     
     esp_err_t res;
 
     // Initialize NVS==============================================
     esp_err_t err = nvs_flash_init();
     if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
         // NVS partition was truncated and needs to be erased
         // Retry nvs_flash_init
         ESP_ERROR_CHECK(nvs_flash_erase());
         err = nvs_flash_init();
     }
     ESP_ERROR_CHECK( err );
     err = nvs_open("storage", NVS_READWRITE, &my_handle);
     if (err != ESP_OK) {
         ESP_LOGI(TAG,"Error (%s) opening NVS handle!\n", esp_err_to_name(err));
     } else {
         ESP_LOGI(TAG,"Done\n");
         err = nvs_get_i32(my_handle, "SPEED", &SPEED);
         switch (err) {
             case ESP_OK:
                 ESP_LOGI(TAG,"Done\n");
                 ESP_LOGI(TAG,"winch SPEED = %ld\n", SPEED);
                 break;
             case ESP_ERR_NVS_NOT_FOUND:
                 ESP_LOGE(TAG,"The value SPEED is not initialized yet!\n");
                 break;
             default :
                 ESP_LOGE(TAG,"Error (%s) reading!\n", esp_err_to_name(err));
         }
         // err = nvs_get_i32(my_handle, "Brightness", &DutyCycle);
         // switch (err) {
         //     case ESP_OK:
         //         ESP_LOGI(TAG,"Done\n");
         //         ESP_LOGI(TAG,"Dutycycle = %ld\n", DutyCycle);
         //         break;
         //     case ESP_ERR_NVS_NOT_FOUND:
         //         ESP_LOGE(TAG,"The value is not initialized yet!\n");
         //         break;
         //     default :
         //         ESP_LOGE(TAG,"Error (%s) reading!\n", esp_err_to_name(err));
         // }
         //  vTaskDelay(100 / portTICK_PERIOD_MS);
 
 
         vTaskDelay(100 / portTICK_PERIOD_MS);
             // Close
         nvs_close(my_handle);
     }
     //=============================================================
    
 
     //=================================================
 
     initUART();
     initUART1();
     initUART2();
     
     xTaskCreate(rx_task, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
     // xTaskCreate(tx_task, "uart_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
     xTaskCreate(rx_task1, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
      xTaskCreate(pwmLEDC, "pwmLEDC_task", 1024 * 4, NULL, configMAX_PRIORITIES - 3, NULL);
     // xTaskCreate(rx_task2, "uart_rx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
     // xTaskCreate(tx_task2, "uart_tx_task", 1024 * 8, NULL, configMAX_PRIORITIES - 2, NULL);
     xTaskCreate(leak_sensor_task, "leak_sensor_task", 8196, NULL, configMAX_PRIORITIES - 4, NULL);
    // xTaskCreate(remote_read_task, "remote_read_task",  1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
     // xTaskCreate(motorRunFwd, "motorRunFwd",  1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
     ESP_LOGI(TAG, "Initialization DONE");
 
     while (1) {
         ESP_LOGD(TAG, "Looping");
         vTaskDelay(100 / portTICK_PERIOD_MS);
     }
 }
 
 
 
 
 
 
 
 