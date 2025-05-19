#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "lora/lora.h"

// Định nghĩa GPIO cho nút bấm
#define BUTTON_TX_POWER 15  // Nút thay đổi TX Power
#define BUTTON_SF 16        // Nút thay đổi SF
#define BUTTON_CR 6         // Nút thay đổi CR

// Biến toàn cục
static uint8_t tx_power_index = 0;  // Chỉ số công suất: 0->5dBm, 1->10dBm, 2->15dBm, 3->20dBm
static uint8_t sf_index = 0;        // Chỉ số SF: 0->7, 1->8, 2->9, 3->10, 4->11, 5->12 (mặc định SF=7)
static uint8_t cr_index = 0;        // Chỉ số CR: 0->4/5, 1->4/6, 2->4/7, 3->4/8 (mặc định CR=4/5)
static const uint8_t tx_powers[] = {5, 10, 15, 20};  // Các mức công suất
static const uint8_t sf_values[] = {7, 8, 9, 10, 11, 12};  // Các mức SF
static const uint8_t cr_values[] = {5, 6, 7, 8};           // Các mức CR (4/5, 4/6, 4/7, 4/8)
static uint32_t count = 0;  // Biến đếm gói tin

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
    lora_set_frequency(433000000);  // 433 MHz
    lora_set_spreading_factor(sf_values[sf_index]);
    lora_set_bandwidth(125E3);
    lora_set_coding_rate(cr_values[cr_index]);
    lora_enable_crc();
    lora_set_preamble_length(12);
    lora_set_sync_word(0x34);
    lora_set_tx_power(tx_powers[tx_power_index]);
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
            count = 0;  // Đặt lại count khi thay đổi
            update_lora_config();
            printf("Changed TX Power to: %d dBm\n", tx_powers[tx_power_index]);
        }

        // Xử lý nút thay đổi SF
        if (debounce_button(BUTTON_SF, &last_sf_state)) {
            sf_index = (sf_index + 1) % 6;  // 6 giá trị
            count = 0;  // Đặt lại count khi thay đổi
            update_lora_config();
            printf("Changed SF to: %d\n", sf_values[sf_index]);
        }

        // Xử lý nút thay đổi CR
        if (debounce_button(BUTTON_CR, &last_cr_state)) {
            cr_index = (cr_index + 1) % 4;  // 4 giá trị
            count = 0;  // Đặt lại count khi thay đổi
            update_lora_config();
            printf("Changed CR to: 4/%d\n", cr_values[cr_index]);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Chờ ngắn để giảm tải CPU
    }
}

// Task gửi gói tin
void transmit_task(void *pvParameters) {
    // Khởi tạo LoRa
    if (lora_init() == 0) {
        printf("LoRa init failed!\n");
        vTaskDelete(NULL);
    }

    // Chuẩn bị chuỗi 128 byte chứa 0xAA
    uint8_t buffer[128];
    memset(buffer, 0xAA, sizeof(buffer));

    while (1) {
        // Gửi gói tin
        lora_send_packet(buffer, sizeof(buffer));
        count++;

        // Hiển thị thông tin phép thử
        printf("Sent packet %lu: ", count);
        for (int i = 0; i < sizeof(buffer); i++) {
            printf("%02X ", buffer[i]);
        }
        printf("\n");
        printf("TX Power: %d dBm, SF: %d, CR: 4/%d\n",
               tx_powers[tx_power_index], sf_values[sf_index], cr_values[cr_index]);
        printf("Count: %lu\n", count);

        // Chờ 7 giây để phù hợp với time-on-air (~6 giây)
        vTaskDelay(pdMS_TO_TICKS(7000));
    }
}

void app_main(void) {
    // Cấu hình nút bấm
    configure_buttons();

    // Tạo các task
    xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
    xTaskCreate(transmit_task, "transmit_task", 4096, NULL, 5, NULL);
}