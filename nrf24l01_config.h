#ifndef NRF24L01_NRF24L01_CONFIG_H
#define NRF24L01_NRF24L01_CONFIG_H

#include <stddef.h>
#include <stdint.h>

#define NRF24L01_TIME_STANDBY_2A_NS 130000UL

typedef enum {
    NRF24L01_ERR_OK = 0,
    NRF24L01_ERR_FAIL = 1 << 0,
    NRF24L01_ERR_NULL = 1 << 1,
} nrf24l01_err_t;

typedef enum {
    NRF24L01_REG_ADDRESS_CONFIG = 0x00,
    NRF24L01_REG_ADDRESS_EN_AA = 0x01,
    NRF24L01_REG_ADDRESS_EN_RXADDR = 0x02,
    NRF24L01_REG_ADDRESS_SETUP_AW = 0x03,
    NRF24L01_REG_ADDRESS_SETUP_RETR = 0x04,
    NRF24L01_REG_ADDRESS_RF_CH = 0x05,
    NRF24L01_REG_ADDRESS_RF_SETUP = 0x06,
    NRF24L01_REG_ADDRESS_STATUS = 0x07,
    NRF24L01_REG_ADDRESS_OBSERVE_TX = 0x08,
    NRF24L01_REG_ADDRESS_RPD = 0x09,
    NRF24L01_REG_ADDRESS_RX_ADDR_P0 = 0x0A,
    NRF24L01_REG_ADDRESS_RX_ADDR_P1 = 0x0B,
    NRF24L01_REG_ADDRESS_RX_ADDR_P2 = 0x0C,
    NRF24L01_REG_ADDRESS_RX_ADDR_P3 = 0x0D,
    NRF24L01_REG_ADDRESS_RX_ADDR_P4 = 0x0E,
    NRF24L01_REG_ADDRESS_RX_ADDR_P5 = 0x0F,
    NRF24L01_REG_ADDRESS_TX_ADDR = 0x10,
    NRF24L01_REG_ADDRESS_RX_PW_P0 = 0x11,
    NRF24L01_REG_ADDRESS_RX_PW_P1 = 0x12,
    NRF24L01_REG_ADDRESS_RX_PW_P2 = 0x13,
    NRF24L01_REG_ADDRESS_RX_PW_P3 = 0x14,
    NRF24L01_REG_ADDRESS_RX_PW_P4 = 0x15,
    NRF24L01_REG_ADDRESS_RX_PW_P5 = 0x16,
    NRF24L01_REG_ADDRESS_FIFO_STATUS = 0x17,
    NRF24L01_REG_ADDRESS_ACK_PLD,
    NRF24L01_REG_ADDRESS_TX_PLD,
    NRF24L01_REG_ADDRESS_RX_PLD,
    NRF24L01_REG_ADDRESS_DYNPD = 0x1C,
    NRF24L01_REG_ADDRESS_FEATURE = 0x1D,
} nrf24l01_reg_address_t;

typedef enum {
    NRF24L01_AIR_DATA_RATE_1MBPS = 0x00,
    NRF24L01_AIR_DATA_RATE_2MBPS = 0x01,
    NRF24L01_AIR_DATA_RATE_250KBPS,
} nrf24l01_air_data_rate_t;

typedef enum {
    NRF24L01_POWER_AMPLIFIER_P_0DBM = 0b11,
    NRF24L01_POWER_AMPLIFIERN_6DBM = 0b10,
    NRF24L01_POWER_AMPLIFIERN_12DBM = 0b01,
    NRF24L01_POWER_AMPLIFIERN_18DBM = 0b00,
} nrf24l01_power_amplifier_t;

typedef enum {
    NRF24L01_MODE_POWER_DOWN,
    NRF24L01_MODE_STANDBY_I,
    NRF24L01_MODE_STANDBY_II,
    NRF24L01_MODE_RX_MODE,
    NRF24L01_MODE_TX_MODE,
} nrf24l01_mode_t;

typedef enum {
    NRF24L01_MASK_IRQ_DISABLED = 0x01,
    NRF24L01_MASK_IRQ_ACTIVE_LOW = 0x00,
} nrf24l01_mask_irq_t;

typedef enum {
    NRF24L01_PIPE_ADDRESS_LEN_3BYTES = 0b01,
    NRF24L01_PIPE_ADDRESS_LEN_4BYTES = 0b10,
    NRF24L01_PIPE_ADDRESS_LEN_5BYTES = 0b11,
} nrf24l01_pipe_address_len_t;

typedef enum {
    NRF24L01_AUTO_RETRANSMIT_DELAY_250US = 0b0000,
    NRF24L01_AUTO_RETRANSMIT_DELAY_500US = 0b0001,
    NRF24L01_AUTO_RETRANSMIT_DELAY_750US = 0b0010,
    NRF24L01_AUTO_RETRANSMIT_DELAY_4000US = 0b1111,
} nrf24l01_auto_retransmit_delay_t;

typedef enum {
    NRF24L01_AUTO_RETRANSMIT_COUNT_DISABLED = 0b0000,
    NRF24L01_AUTO_RETRANSMIT_COUNT_X1 = 0b0001,
    NRF24L01_AUTO_RETRANSMIT_COUNT_X15 = 0b1111,
} nrf24l01_auto_retransmit_count_t;

typedef enum {
    NRF24L01_PIPE_NUM_0 = 0b000,
    NRF24L01_PIPE_NUM_1 = 0b001,
    NRF24L01_PIPE_NUM_2 = 0b010,
    NRF24L01_PIPE_NUM_3 = 0b011,
    NRF24L01_PIPE_NUM_4 = 0b100,
    NRF24L01_PIPE_NUM_5 = 0b101,
} nrf24l01_pipe_num_t;

typedef enum {
    NRF24L01_CRC_LEN_DISABLED,
    NRF24L01_CRC_LEN_1BYTE,
    NRF24L01_CRC_LEN_2BYTES,
} nrf24l01_crc_len_t;

typedef enum {
    NRF24L01_PAYLOAD_LEN_0BYTES = 0b000000,
    NRF24L01_PAYLOAD_LEN_32BYTES = 0b100000,
    NRF24L01_PAYLOAD_DONT_CARE = 0b100001,
} nrf24l01_payload_len_t;

typedef enum {
    NRF24L01_PACKET_TYPE_SHOCK_BURST,
    NRF24L01_PACKET_TYPE_ENHANCED_SHOCK_BURST,
} nrf24l01_packet_type_t;

typedef enum {
    NRF24L01_FIFO_STATE_OCCUPIED,
    NRF24L01_FIFO_STATE_RX_EMPTY,
    NRF24L01_FIFO_STATE_TX_EMPTY,
    NRF24L01_FIFO_STATE_RX_FULL,
    NRF24L01_FIFO_STATE_TX_FULL,
    NRF24L01_FIFO_STATE_TX_REUSE,
    NRF24L01_FIFO_STATE_INVALID,
} nrf24l01_fifo_state_t;

typedef struct {
    uint64_t address : 40;
} nrf24l01_pipe_address_t;

typedef struct {
    uint8_t payload_length : 6;
    uint8_t pid : 2;
} nrf24l01_control_field_t;

typedef struct {
    size_t address_len;
    uint32_t chip_enable_pin;
} nrf24l01_config_t;

typedef struct {
    void* gpio_user;
    nrf24l01_err_t (*gpio_initialize)(void*);
    nrf24l01_err_t (*gpio_deinitialize)(void*);
    nrf24l01_err_t (*gpio_write)(void*, uint32_t, bool);

    void* bus_user;
    nrf24l01_err_t (*bus_initialize)(void*);
    nrf24l01_err_t (*bus_deinitialize)(void*);
    nrf24l01_err_t (*bus_write_data)(void*, uint8_t, uint8_t const*, size_t);
    nrf24l01_err_t (*bus_read_data)(void*, uint8_t, uint8_t*, size_t);
    nrf24l01_err_t (*bus_transmit)(void*, uint8_t const*, size_t);
    nrf24l01_err_t (*bus_receive)(void*, uint8_t*, size_t);
} nrf24l01_interface_t;

#endif // NRF24L01_NRF24L01_CONFIG_H