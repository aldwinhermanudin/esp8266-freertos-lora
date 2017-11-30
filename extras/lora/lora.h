#ifndef DRIVER_LORA_H_
#define DRIVER_LORA_H_

#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "esp8266.h"
#include "esp/spi.h"
#include "task.h"

#define SPI_WRITE_MASK 0x80
#define SPI_READ_MASK 0x7f

#define DEFAULT_CS_PIN			15	// ESP8266 default CS pin
#define DEFAULT_RESET_PIN		16
#define DEFAULT_IRQ_PIN			5

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80
#define PA_OUTPUT_RFO_PIN      		0
#define PA_OUTPUT_PA_BOOST_PIN 		1

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH				255

/*
static  int _lora_frequency;
static int _lora_packet_index;
static int _lora_implicit_header_mode;
static void (*_lora_on_receive)(int);
static int _lora_spi_frequency = SPI_FREQ_DIV_8M;
static int _lora_ss = DEFAULT_CS_PIN;
static int _lora_reset = DEFAULT_RESET_PIN;
static int _lora_dio0 = DEFAULT_IRQ_PIN;

static uint8_t single_transfer(uint8_t address, uint8_t value);
static uint8_t read_register(uint8_t address);
static void write_register(uint8_t address, uint8_t value);
 

static void lora_explicit_header_mode();
static void lora_implicit_header_mode();

static void lora_on_dio0_rise(uint8_t gpio_num);
static void lora_handle_dio0_rise();
*/

void lora_set_pins(int ss, int reset, int dio0);
void lora_set_spi_frequency(uint32_t spi_freq);
int lora_begin(long frequency);
void lora_end();

int lora_begin_packet(int implicitHeader);
int lora_begin_packet_default();
int lora_end_packet();

int lora_parse_packet(int size);
int lora_packet_rssi();
float lora_packet_snr();

size_t lora_write(const uint8_t *buffer, size_t size);
size_t lora_write_default(uint8_t byte);
size_t lora_write_string(char * input);

int lora_available();
int lora_read();
int lora_peek();

void lora_on_receive(void(*callback)(int));

void lora_receive(int size);
void lora_receive_default();
void lora_idle();
void lora_sleep();

void lora_set_tx_power(int level, int outputPin);
void lora_set_tx_power_pa_boost(int level);
void lora_set_frequency(long frequency);
void lora_set_spreading_factor(int sf);
void lora_set_signal_bandwidth(long sbw);
void lora_set_coding_rate_4(int denominator);
void lora_set_preamble_length(long length);
void lora_set_sync_word(int sw);
void lora_enable_crc();
void lora_disable_crc();
uint8_t lora_random();

#endif
