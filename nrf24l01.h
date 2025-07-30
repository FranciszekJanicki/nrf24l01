#ifndef NRF24L01_NRF24L01_H
#define NRF24L01_NRF24L01_H

#include "nrf24l01_commands.h"
#include "nrf24l01_config.h"
#include "nrf24l01_registers.h"

typedef struct {
  nrf24l01_config_t config;
  nrf24l01_interface_t interface;
} nrf24l01_t;

nrf24l01_err_t nrf24l01_initialize(nrf24l01_t *nrf24l01,
                                   nrf24l01_config_t const *config,
                                   nrf24l01_interface_t const *interface);
nrf24l01_err_t nrf24l01_deinitialize(nrf24l01_t *nrf24l01);

nrf24l01_err_t nrf24l01_receive_pipe_data(nrf24l01_t const *nrf24l01,
                                          nrf24l01_pipe_num_t pipe_num,
                                          uint8_t *receive_data,
                                          size_t receive_size);
nrf24l01_err_t nrf24l01_transmit_pipe_data(nrf24l01_t const *nrf24l01,
                                           nrf24l01_pipe_num_t pipe_num,
                                           uint8_t const *transmit_data,
                                           size_t transmit_size);

uint8_t nrf24l01_get_allowed_payload_size(nrf24l01_t const *nrf24l01,
                                          nrf24l01_pipe_num_t pipe_num,
                                          size_t proposed_size,
                                          size_t *allowed_size);

nrf24l01_err_t nrf24l01_start_listening(nrf24l01_t const *nrf24l01);
nrf24l01_err_t nrf24l01_stop_listening(nrf24l01_t const *nrf24l01);

nrf24l01_err_t nrf24l01_is_payload_available(nrf24l01_t const *nrf24l01,
                                             nrf24l01_pipe_num_t pipe_num,
                                             bool *is_payload_available);

nrf24l01_err_t
nrf24l01_open_writing_pipe(nrf24l01_t const *nrf24l01,
                           nrf24l01_pipe_address_t *pipe_address);
nrf24l01_err_t
nrf24l01_open_reading_pipe(nrf24l01_t const *nrf24l01,
                           nrf24l01_pipe_num_t pipe_num,
                           nrf24l01_pipe_address_t *pipe_address);

nrf24l01_err_t nrf24l01_close_reading_pipe(nrf24l01_t const *nrf24l01,
                                           nrf24l01_pipe_num_t pipe_num);
nrf24l01_err_t nrf24l01_close_writing_pipe(nrf24l01_t const *nrf24l01);

nrf24l01_err_t nrf24l01_power_up(nrf24l01_t const *nrf24l01);
nrf24l01_err_t nrf24l01_power_down(nrf24l01_t const *nrf24l01);

nrf24l01_err_t nrf24l01_reuse_tx(nrf24l01_t const *nrf24l01);

nrf24l01_err_t nrf24l01_set_chip_enable(nrf24l01_t const *nrf24l01);
nrf24l01_err_t nrf24l01_reset_chip_enable(nrf24l01_t const *nrf24l01);

nrf24l01_err_t nrf24l01_get_config_reg(nrf24l01_t const *nrf24l01,
                                       nrf24l01_config_reg_t *reg);
nrf24l01_err_t nrf24l01_set_config_reg(nrf24l01_t const *nrf24l01,
                                       nrf24l01_config_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_en_aa_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_en_aa_reg_t *reg);
nrf24l01_err_t nrf24l01_set_en_aa_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_en_aa_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_en_rxaddr_reg(nrf24l01_t const *nrf24l01,
                                          nrf24l01_en_rxaddr_reg_t *reg);
nrf24l01_err_t nrf24l01_set_en_rxaddr_reg(nrf24l01_t const *nrf24l01,
                                          nrf24l01_en_rxaddr_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_setup_aw_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_setup_aw_reg_t *reg);
nrf24l01_err_t nrf24l01_set_setup_aw_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_setup_aw_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_setup_retr_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_setup_retr_reg_t *reg);
nrf24l01_err_t
nrf24l01_set_setup_retr_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_setup_retr_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rf_ch_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_rf_ch_reg_t *reg);
nrf24l01_err_t nrf24l01_set_rf_ch_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_rf_ch_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rf_setup_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rf_setup_reg_t *reg);
nrf24l01_err_t nrf24l01_set_rf_setup_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rf_setup_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_status_reg(nrf24l01_t const *nrf24l01,
                                       nrf24l01_status_reg_t *reg);
nrf24l01_err_t nrf24l01_set_status_reg(nrf24l01_t const *nrf24l01,
                                       nrf24l01_status_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_observe_tx_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_observe_tx_reg_t *reg);

nrf24l01_err_t nrf24l01_get_rpd_reg(nrf24l01_t const *nrf24l01,
                                    nrf24l01_rpd_reg_t *reg);

nrf24l01_err_t nrf24l01_get_rx_addr_p0_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_p0_reg_t *reg);
nrf24l01_err_t
nrf24l01_set_rx_addr_p0_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_rx_addr_p0_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rx_addr_p1_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_p1_reg_t *reg);
nrf24l01_err_t
nrf24l01_set_rx_addr_p1_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_rx_addr_p1_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rx_addr_p2_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_p2_reg_t *reg);
nrf24l01_err_t
nrf24l01_set_rx_addr_p2_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_rx_addr_p2_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rx_addr_p3_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_p3_reg_t *reg);
nrf24l01_err_t
nrf24l01_set_rx_addr_p3_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_rx_addr_p3_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rx_addr_p4_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_p4_reg_t *reg);
nrf24l01_err_t
nrf24l01_set_rx_addr_p4_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_rx_addr_p4_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rx_addr_p5_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_5_reg_t *reg);
nrf24l01_err_t nrf24l01_set_rx_addr_p5_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_5_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_tx_addr_reg(nrf24l01_t const *nrf24l01,
                                        nrf24l01_tx_addr_reg_t *reg);
nrf24l01_err_t nrf24l01_set_tx_addr_reg(nrf24l01_t const *nrf24l01,
                                        nrf24l01_tx_addr_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rx_pw_p0_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p0_reg_t *reg);
nrf24l01_err_t nrf24l01_set_rx_pw_p0_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p0_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rx_pw_p1_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p1_reg_t *reg);
nrf24l01_err_t nrf24l01_set_rx_pw_p1_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p1_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rx_pw_p2_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p2_reg_t *reg);
nrf24l01_err_t nrf24l01_set_rx_pw_p2_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p2_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rx_pw_p3_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p3_reg_t *reg);
nrf24l01_err_t nrf24l01_set_rx_pw_p3_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p3_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rx_pw_p4_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p4_reg_t *reg);
nrf24l01_err_t nrf24l01_set_rx_pw_p4_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p4_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_rx_pw_p5_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p5_reg_t *reg);
nrf24l01_err_t nrf24l01_set_rx_pw_p5_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p5_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_fifo_status_reg(nrf24l01_t const *nrf24l01,
                                            nrf24l01_fifo_status_reg_t *reg);

nrf24l01_err_t nrf24l01_get_dynpd_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_dynpd_reg_t *reg);
nrf24l01_err_t nrf24l01_set_dynpd_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_dynpd_reg_t const *reg);

nrf24l01_err_t nrf24l01_get_feature_reg(nrf24l01_t const *nrf24l01,
                                        nrf24l01_feature_reg_t *reg);
nrf24l01_err_t nrf24l01_set_feature_reg(nrf24l01_t const *nrf24l01,
                                        nrf24l01_feature_reg_t const *reg);

nrf24l01_err_t nrf24l01_send_rx_payload_cmd(nrf24l01_t const *nrf24l01,
                                            nrf24l01_rx_payload_cmd_t *cmd);
nrf24l01_err_t nrf24l01_send_tx_payload_cmd(nrf24l01_t const *nrf24l01,
                                            nrf24l01_tx_payload_cmd_t const *);

nrf24l01_err_t nrf24l01_send_flush_tx_cmd(nrf24l01_t const *nrf24l01);
nrf24l01_err_t nrf24l01_send_flush_rx_cmd(nrf24l01_t const *nrf24l01);

nrf24l01_err_t nrf24l01_send_reuse_tx_pl_cmd(nrf24l01_t const *nrf24l01);

nrf24l01_err_t nrf24l01_send_activate_cmd(nrf24l01_t const *nrf24l01);

nrf24l01_err_t nrf24l01_send_r_rx_pl_wid_cmd(nrf24l01_t const *nrf24l01);

nrf24l01_err_t nrf24l01_send_w_ack_payload_p0_cmd(nrf24l01_t const *nrf24l01);
nrf24l01_err_t nrf24l01_send_w_ack_payload_p1_cmd(nrf24l01_t const *nrf24l01);
nrf24l01_err_t nrf24l01_send_w_ack_payload_p2_cmd(nrf24l01_t const *nrf24l01);
nrf24l01_err_t nrf24l01_send_w_ack_payload_p3_cmd(nrf24l01_t const *nrf24l01);
nrf24l01_err_t nrf24l01_send_w_ack_payload_p4_cmd(nrf24l01_t const *nrf24l01);
nrf24l01_err_t nrf24l01_send_w_ack_payload_p5_cmd(nrf24l01_t const *nrf24l01);

nrf24l01_err_t nrf24l01_send_w_tx_payload_noack_cmd(nrf24l01_t const *nrf24l01);

nrf24l01_err_t nrf24l01_send_nop_cmd(nrf24l01_t const *nrf24l01);

#endif // NRF24L01_NRF24L01_H