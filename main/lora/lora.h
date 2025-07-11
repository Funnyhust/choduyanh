
#ifndef __LORA_H__
#define __LORA_H__

#define REG_FIFO                       0x00
#define REG_OP_MODE                    0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                    0x07
#define REG_FRF_LSB                    0x08
#define REG_PA_CONFIG                  0x09
#define REG_PA_DAC                     0x4d
#define REG_LNA                        0x0c
#define REG_FIFO_ADDR_PTR              0x0d
#define REG_FIFO_TX_BASE_ADDR          0x0e
#define REG_FIFO_RX_BASE_ADDR          0x0f
#define REG_FIFO_RX_CURRENT_ADDR       0x10
#define REG_IRQ_FLAGS                  0x12
#define REG_RX_NB_BYTES                0x13
#define REG_PKT_SNR_VALUE              0x19
#define REG_PKT_RSSI_VALUE             0x1a
#define REG_MODEM_CONFIG_1             0x1d
#define REG_MODEM_CONFIG_2             0x1e
#define REG_PREAMBLE_MSB               0x20
#define REG_PREAMBLE_LSB               0x21
#define REG_PAYLOAD_LENGTH             0x22
#define REG_MODEM_CONFIG_3             0x26
#define REG_RSSI_WIDEBAND              0x2c
#define REG_DETECTION_OPTIMIZE         0x31
#define REG_DETECTION_THRESHOLD        0x37
#define REG_SYNC_WORD                  0x39
#define REG_DIO_MAPPING_1              0x40
#define REG_VERSION                    0x42

/*
 * Transceiver modes
 */
#define MODE_LONG_RANGE_MODE           0x80
#define MODE_SLEEP                     0x00
#define MODE_STDBY                     0x01
#define MODE_TX                        0x03
#define MODE_RX_CONTINUOUS             0x05
#define MODE_RX_SINGLE                 0x06

/*
 * PA configuration
 */
#define PA_BOOST                       0x80

/*
 * IRQ masks
 */
#define IRQ_TX_DONE_MASK               0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK     0x20
#define IRQ_RX_DONE_MASK               0x40

#define PA_OUTPUT_RFO_PIN              0
#define PA_OUTPUT_PA_BOOST_PIN         1

#define TIMEOUT_RESET                  100

#define CONFIG_CS_GPIO      13   // Chip Select
#define CONFIG_RST_GPIO     18// Reset
#define CONFIG_MISO_GPIO    1  // SPI MISO
#define CONFIG_MOSI_GPIO    14  // SPI MOSI
#define CONFIG_SCK_GPIO     12  // SPI SCK
#define CONFIG_DIO0_GPIO    44 // DIO0 interrupt
#define CONFIG_LORA_FREQUENCY 433000000 // 433 MHz
#define CONFIG_LORA_CRC     1   // Enable CRC

void lora_reset(void);
void lora_explicit_header_mode(void);
void lora_implicit_header_mode(int size);
void lora_idle(void);
void lora_sleep(void); 
void lora_receive(void);
void lora_set_tx_power(int level);
void lora_set_frequency(long frequency);
void lora_set_spreading_factor(int sf);
void lora_set_bandwidth(long sbw);
void lora_set_coding_rate(int denominator);
void lora_set_preamble_length(long length);
void lora_set_sync_word(int sw);
void lora_enable_crc(void);
void lora_disable_crc(void);
int lora_init(void);
void lora_send_packet(uint8_t *buf, int size);
int lora_receive_packet(uint8_t *buf, int size);
int lora_received(void);
int lora_packet_rssi(void);
float lora_packet_snr(void);
void lora_close(void);
int lora_initialized(void);
void lora_dump_registers(void);
void  lora_write_reg(int reg, int val);
int  lora_read_reg(int reg);

#endif
