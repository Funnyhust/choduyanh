
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "lora/lora.h"
#include "lcd/lcd.h"

// Định nghĩa GPIO cho nút bấm
#define BUTTON_TX_POWER 15  // Nút thay đổi TX Power
#define BUTTON_SF 16        // Nút thay đổi SF
#define BUTTON_CR 6         // Nút thay đổi CR

// Biến toàn cục
static uint8_t tx_power_index = 0;  // Chỉ số công suất: 0->5dBm, 1->10dBm, 2->15dBm, 3->20dBm
static uint8_t sf_index = 0;        // Chỉ số SF: 0->7, 1->8, 2->9, 3->10, 4->11, 5->12
static uint8_t cr_index = 0;        // Chỉ số CR: 0->4/5, 1->4/6, 2->4/7, 3->4/8
static const uint8_t tx_powers[] = {5, 10, 15, 20};  // Các mức công suất
static const uint8_t sf_values[] = {7, 8, 9, 10, 11, 12};  // Các mức SF
static const uint8_t cr_values[] = {5, 6, 7, 8};           // Các mức CR
static uint32_t count = 0;  // Biến đếm gói tin
static SemaphoreHandle_t count_mutex; // Semaphore để bảo vệ count
// static volatile bool config_changed = false; // Flag báo thay đổi cấu hình

// Hàm cấu hình GPIO cho nút bấm
void configure_buttons(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_TX_POWER) | (1ULL << BUTTON_SF) | (1ULL << BUTTON_CR),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

// Hàm kiểm tra trạng thái nút bấm (chống dội)
bool debounce_button(gpio_num_t pin, bool *last_state) {
    bool current_state = gpio_get_level(pin) == 0;  // Nút bấm nối GND, nhấn là 0
    if (current_state != *last_state) {
        vTaskDelay(pdMS_TO_TICKS(50));  // Chờ 50ms để chống dội
        current_state = gpio_get_level(pin) == 0;
        *last_state = current_state;
        return current_state;
    }
    return false;
}

// Hàm cập nhật cấu hình LoRa
void update_lora_config(void) {
    // lora_idle(); // Đặt module về STDBY trước khi thay đổi cấu hình
    lora_set_frequency(433000000);  // 433 MHz
    lora_set_spreading_factor(sf_values[sf_index]);
    lora_set_bandwidth(125E3);
    lora_set_coding_rate(cr_values[cr_index]);
    lora_enable_crc();
    lora_set_preamble_length(12);
    lora_set_sync_word(0x34);
    lora_set_tx_power(tx_powers[tx_power_index]);
    printf("LoRa config updated: TX=%d dBm, SF=%d, CR=4/%d\n",
           tx_powers[tx_power_index], sf_values[sf_index], cr_values[cr_index]);
}

// Task xử lý nút bấm
void button_task(void *pvParameters) {
    bool last_tx_power_state = false;
    bool last_sf_state = false;
    bool last_cr_state = false;

    // Cấu hình ban đầu
    update_lora_config();
    printf("LoRa Test Program Ready!\n");

    while (1) {
        // Xử lý nút thay đổi TX Power
        if (debounce_button(BUTTON_TX_POWER, &last_tx_power_state)) {
            tx_power_index = (tx_power_index + 1) % 4;  // 4 giá trị
            xSemaphoreTake(count_mutex, portMAX_DELAY);
            count = 0;  // Đặt lại count
            xSemaphoreGive(count_mutex);
                update_lora_config();
            printf("Changed TX Power to: %d dBm\n", tx_powers[tx_power_index]);
        }

        // Xử lý nút thay đổi SF
        if (debounce_button(BUTTON_SF, &last_sf_state)) {
            sf_index = (sf_index + 1) % 6;  // 6 giá trị
            xSemaphoreTake(count_mutex, portMAX_DELAY);
            count = 0;  // Đặt lại count
            xSemaphoreGive(count_mutex);
            update_lora_config();
            printf("Changed SF to: %d\n", sf_values[sf_index]);
        }

        // Xử lý nút thay đổi CR
        if (debounce_button(BUTTON_CR, &last_cr_state)) {
            cr_index = (cr_index + 1) % 4;  // 4 giá trị
            xSemaphoreTake(count_mutex, portMAX_DELAY);
            count = 0;  // Đặt lại count
            xSemaphoreGive(count_mutex);
            update_lora_config();
            printf("Changed CR to: 4/%d\n", cr_values[cr_index]);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Chờ ngắn để giảm tải CPU
    }
}

// Task hiển thị trên LCD
void lcd_task(void *pvParameters) {
    esp32_lcd_i2c_t *lcd = (esp32_lcd_i2c_t *)pvParameters;
    char line1[17]; // Buffer cho dòng 1 (16 ký tự + null)
    char line2[17]; // Buffer cho dòng 2 (16 ký tự + null)
    uint32_t last_count = UINT32_MAX; // Đảm bảo cập nhật lần đầu
    uint8_t last_tx_power_index = UINT8_MAX;
    uint8_t last_sf_index = UINT8_MAX;
    uint8_t last_cr_index = UINT8_MAX;

    // Xóa màn hình ban đầu
    esp32_lcd_i2c_clear(lcd);

    while (1) {
        // Lấy giá trị count an toàn
        xSemaphoreTake(count_mutex, portMAX_DELAY);
        uint32_t current_count = count;
        xSemaphoreGive(count_mutex);

        // Cập nhật LCD nếu có thay đổi
        if (current_count != last_count || 
            tx_power_index != last_tx_power_index ||
            sf_index != last_sf_index ||
            cr_index != last_cr_index) {
            
            // Định dạng dòng 1: TX:XX SF:YY (10 ký tự)
            snprintf(line1, sizeof(line1), "TX:%2d SF:%2d", tx_powers[tx_power_index], sf_values[sf_index]);
            
            // Định dạng dòng 2: CR:4/Z CNT:WWW (14 ký tự)
            snprintf(line2, sizeof(line2), "CR:4/%d CNT:%3lu", cr_values[cr_index], current_count % 1000);

            // Hiển thị trên LCD
            esp32_lcd_i2c_set_cursor(lcd, 0, 0);
            esp32_lcd_i2c_print(lcd, line1);
            esp32_lcd_i2c_set_cursor(lcd, 0, 1);
            esp32_lcd_i2c_print(lcd, line2);

            // Log để debug
            printf("LCD Update: %s | %s (Count: %lu)\n", line1, line2, current_count);

            last_count = current_count;
            last_tx_power_index = tx_power_index;
            last_sf_index = sf_index;
            last_cr_index = cr_index;
        }

        // Chờ 1 giây
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task gửi gói tin
void transmit_task(void *pvParameters) {
    // Chuẩn bị chuỗi 128 byte chứa 0xAA
char json_msg[128];
        float temp = 28.5;
        float moisture = 70.2;
        snprintf(json_msg, sizeof(json_msg),
                 "{\"id\":\"DEVICE_001\",\"temp\":%.1f,\"moisture\":%.1f}",
                 temp, moisture);

    while (1) {
        // Kiểm tra nếu cấu hình LoRa thay đổi
        // Gửi gói tin
        printf("Sending packet %lu...\n", count + 1);
               lora_send_packet((uint8_t *)json_msg, strlen(json_msg));
        xSemaphoreTake(count_mutex, portMAX_DELAY);
        count++;
        xSemaphoreGive(count_mutex);
        printf("Packet %lu sent successfully\n", count);

        // In thông tin phép thử lên console
        printf("TX Power: %d dBm, SF: %d, CR: 4/%d\n",
               tx_powers[tx_power_index], sf_values[sf_index], cr_values[cr_index]);
        printf("Count: %lu\n", count);

        // Chờ 7 giây
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    // Tạo semaphore cho count
    count_mutex = xSemaphoreCreateMutex();
    if (count_mutex == NULL) {
        printf("Failed to create count mutex\n");
        return;
    }

    // Khởi tạo LCD
    static esp32_lcd_i2c_t lcd;
    esp_err_t ret = esp32_lcd_i2c_init(&lcd, 0x27, 16, 2, I2C_NUM_0, 46, 2);
    if (ret != ESP_OK) {
        printf("LCD init failed: %s\n", esp_err_to_name(ret));
        return;
    }

    // Bật đèn nền
    esp32_lcd_i2c_backlight_on(&lcd);

    // Khởi tạo LoRa
    if (lora_init() == 0) {
        printf("LoRa init failed!\n");
        return;
    }

    // Cấu hình nút bấm
    configure_buttons();

    // Tạo các task
    xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
    xTaskCreate(transmit_task, "transmit_task", 4096, NULL, 5, NULL);
    xTaskCreate(lcd_task, "lcd_task", 4096, &lcd, 5, NULL);
}
