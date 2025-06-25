#ifndef NRF24L01_NRF24L01_REGISTERS_H
#define NRF24L01_NRF24L01_REGISTERS_H

#include <stdint.h>

typedef struct {
    uint8_t mask_rx_dr : 1;
    uint8_t mask_tx_ds : 1;
    uint8_t mask_max_rt : 1;
    uint8_t en_crc : 1;
    uint8_t crco : 1;
    uint8_t pwr_up : 1;
    uint8_t prim_rx : 1;
} nrf24l01_config_reg_t;

typedef struct {
    uint8_t enaa_p5 : 1;
    uint8_t enaa_p4 : 1;
    uint8_t enaa_p3 : 1;
    uint8_t enaa_p2 : 1;
    uint8_t enaa_p1 : 1;
    uint8_t enaa_p0 : 1;
} nrf24l01_en_aa_reg_t;

typedef struct {
    uint8_t erx_p5 : 1;
    uint8_t erx_p4 : 1;
    uint8_t erx_p3 : 1;
    uint8_t erx_p2 : 1;
    uint8_t erx_p1 : 1;
    uint8_t erx_p0 : 1;
} nrf24l01_en_rxaddr_reg_t;

typedef struct {
    uint8_t aw : 2;
} nrf24l01_setup_aw_reg_t;

typedef struct {
    uint8_t ard : 4;
    uint8_t arc : 4;
} nrf24l01_setup_retr_reg_t;

typedef struct RF_CH {
    uint8_t rf_ch : 7;
} nrf24l01_rf_ch_reg_t;

typedef struct {
    uint8_t cont_wave : 1;
    uint8_t rf_dr : 2;
    uint8_t pll_lock : 1;
    uint8_t rf_pwr : 2;
} nrf24l01_rf_setup_reg_t;

typedef struct {
    uint8_t rx_dr : 1;
    uint8_t tx_ds : 1;
    uint8_t max_rt : 1;
    uint8_t rx_p_no : 3;
    uint8_t tx_full : 1;
} nrf24l01_status_reg_t;

typedef struct {
    uint8_t plos_cnt : 4;
    uint8_t arc_cnt : 4;
} nrf24l01_observe_tx_reg_t;

typedef struct {
    uint8_t rpd : 1;
} nrf24l01_rpd_reg_t;

typedef struct {
    uint64_t rx_addr_p0 : 40;
} nrf24l01_rx_addr_p0_reg_t;

typedef struct {
    uint64_t rx_addr_p1 : 40;
} nrf24l01_rx_addr_p1_reg_t;

typedef struct {
    uint8_t rx_addr_p2 : 8;
} nrf24l01_rx_addr_p2_reg_t;

typedef struct {
    uint8_t rx_addr_p3 : 8;
} nrf24l01_rx_addr_p3_reg_t;

typedef struct {
    uint8_t rx_addr_p4 : 8;
} nrf24l01_rx_addr_p4_reg_t;

typedef struct {
    uint8_t rx_addr_p5 : 8;
} nrf24l01_rx_addr_5_reg_t;

typedef struct {
    uint64_t tx_addr : 40;
} nrf24l01_tx_addr_reg_t;

typedef struct {
    uint8_t rx_pw_p0 : 6;
} nrf24l01_rx_pw_p0_reg_t;

typedef struct {
    uint8_t rx_pw_p1 : 6;
} nrf24l01_rx_pw_p1_reg_t;

typedef struct {
    uint8_t rx_pw_p2 : 6;
} nrf24l01_rx_pw_p2_reg_t;

typedef struct {
    uint8_t rx_pw_p3 : 6;
} nrf24l01_rx_pw_p3_reg_t;

typedef struct {
    uint8_t rx_pw_p4 : 6;
} nrf24l01_rx_pw_p4_reg_t;

typedef struct {
    uint8_t rx_pw_p5 : 6;
} nrf24l01_rx_pw_p5_reg_t;

typedef struct {
    uint8_t tx_reuse : 1;
    uint8_t tx_full : 1;
    uint8_t tx_empty : 1;
    uint8_t rx_full : 1;
    uint8_t rx_empty : 1;
} nrf24l01_fifo_status_reg_t;

typedef struct {
    uint8_t dpl_p5 : 1;
    uint8_t dpl_p4 : 1;
    uint8_t dpl_p3 : 1;
    uint8_t dpl_p2 : 1;
    uint8_t dpl_p1 : 1;
    uint8_t dpl_p0 : 1;
} nrf24l01_dynpd_reg_t;

typedef struct {
    uint8_t en_dpl : 1;
    uint8_t en_ack_pay : 1;
    uint8_t en_dyn_ack : 1;
} nrf24l01_feature_reg_t;

#endif // NRF24L01_NRF24L01_REGISTERS_H