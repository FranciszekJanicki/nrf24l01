#include "nrf24l01.h"
#include "nrf24l01_commands.h"
#include "nrf24l01_registers.h"
#include <assert.h>
#include <string.h>

static nrf24l01_err_t nrf24l01_gpio_initialize(nrf24l01_t const *nrf24l01) {
  return nrf24l01->interface.gpio_initialize
             ? nrf24l01->interface.gpio_initialize(
                   nrf24l01->interface.gpio_user)
             : NRF24L01_ERR_NULL;
}

static nrf24l01_err_t nrf24l01_gpio_deinitialize(nrf24l01_t const *nrf24l01) {
  return nrf24l01->interface.gpio_deinitialize
             ? nrf24l01->interface.gpio_deinitialize(
                   nrf24l01->interface.gpio_user)
             : NRF24L01_ERR_NULL;
}

static nrf24l01_err_t nrf24l01_gpio_write(nrf24l01_t const *nrf24l01,
                                          uint32_t pin, bool state) {
  return nrf24l01->interface.gpio_write
             ? nrf24l01->interface.gpio_write(nrf24l01->interface.gpio_user,
                                              pin, state)
             : NRF24L01_ERR_NULL;
}

static nrf24l01_err_t nrf24l01_bus_initialize(nrf24l01_t const *nrf24l01) {
  return nrf24l01->interface.bus_initialize
             ? nrf24l01->interface.bus_initialize(nrf24l01->interface.bus_user)
             : NRF24L01_ERR_NULL;
}

static nrf24l01_err_t nrf24l01_bus_deinitialize(nrf24l01_t const *nrf24l01) {
  return nrf24l01->interface.bus_deinitialize
             ? nrf24l01->interface.bus_deinitialize(
                   nrf24l01->interface.bus_user)
             : NRF24L01_ERR_NULL;
}

static nrf24l01_err_t nrf24l01_bus_write_data(nrf24l01_t const *nrf24l01,
                                              uint8_t write_address,
                                              uint8_t const *write_data,
                                              size_t write_size) {
  return nrf24l01->interface.bus_write_data
             ? nrf24l01->interface.bus_write_data(nrf24l01->interface.bus_user,
                                                  write_address, write_data,
                                                  write_size)
             : NRF24L01_ERR_NULL;
}

static nrf24l01_err_t nrf24l01_bus_read_data(nrf24l01_t const *nrf24l01,
                                             uint8_t read_address,
                                             uint8_t *read_data,
                                             size_t read_size) {
  return nrf24l01->interface.bus_read
             ? nrf24l01->interface.bus_read_data(nrf24l01->interface.bus_user,
                                                 read_address, read_data,
                                                 read_size)
             : NRF24L01_ERR_NULL;
}

static nrf24l01_err_t nrf24l01_bus_transmit(nrf24l01_t const *nrf24l01,
                                            uint8_t const *transmit_data,
                                            size_t transmit_size) {
  return nrf24l01->interface.bus_transmit
             ? nrf24l01->interface.bus_transmit(nrf24l01->interface.bus_user,
                                                transmit_data, transmit_size)
             : NRF24L01_ERR_NULL;
}

static nrf24l01_err_t nrf24l01_bus_receive(nrf24l01_t const *nrf24l01,
                                           uint8_t *receive_data,
                                           size_t receive_size) {
  return nrf24l01->interface.bus_receive
             ? nrf24l01->interface.bus_receive(nrf24l01->interface.bus_user,
                                               receive_data, receive_size)
             : NRF24L01_ERR_NULL;
}

nrf24l01_err_t nrf24l01_initialize(nrf24l01_t *nrf24l01,
                                   nrf24l01_config_t const *config,
                                   nrf24l01_interface_t const *interface) {
  assert(nrf24l01 && config && interface);

  memset(nrf24l01, 0, sizeof(*nrf24l01));
  memcpy(&nrf24l01->config, config, sizeof(*config));
  memcpy(&nrf24l01->interface, interface, sizeof(*interface));

  nrf24l01_err_t err = nrf24l01_bus_initialize(nrf24l01);
  err |= nrf24l01_gpio_initialize(nrf24l01);

  return err;
}

nrf24l01_err_t nrf24l01_deinitialize(nrf24l01_t *nrf24l01) {
  assert(nrf24l01);

  nrf24l01_err_t err = nrf24l01_bus_deinitialize(nrf24l01);
  err |= nrf24l01_gpio_deinitialize(nrf24l01);

  memset(nrf24l01, 0, sizeof(*nrf24l01));

  return err;
}

nrf24l01_err_t nrf24l01_receive_pipe_data(nrf24l01_t const *nrf24l01,
                                          nrf24l01_pipe_num_t pipe_num,
                                          uint8_t *receive_data,
                                          size_t receive_size) {
  assert(nrf24l01 && receive_data);

  size_t allowed_size = {};
  nrf24l01_err_t err = nrf24l01_get_allowed_payload_size(
      nrf24l01, pipe_num, receive_size, &allowed_size);

  if (receive_size <= allowed_size) {
    nrf24l01_rx_payload_cmd_t cmd = {};
    err |= nrf24l01_send_rx_payload_cmd(nrf24l01, &cmd);
    memcpy(receive_data, cmd.payload, receive_size);

    nrf24l01_status_reg_t reg = {};
    err |= nrf24l01_get_status_reg(nrf24l01, &reg);
    reg.rx_dr = true;
    err |= nrf24l01_set_status_reg(nrf24l01, &reg);
  }

  return err;
}

nrf24l01_err_t nrf24l01_transmit_pipe_data(nrf24l01_t const *nrf24l01,
                                           nrf24l01_pipe_num_t pipe_num,
                                           uint8_t const *transmit_data,
                                           size_t transmit_size) {
  assert(nrf24l01 && transmit_data);

  size_t allowed_size = {};
  nrf24l01_err_t err = nrf24l01_get_allowed_payload_size(
      nrf24l01, pipe_num, transmit_size, &allowed_size);

  if (transmit_size <= allowed_size) {
    nrf24l01_tx_payload_cmd_t cmd = {};
    memcpy(cmd.payload, transmit_data, transmit_size);
    err |= nrf24l01_send_tx_payload_cmd(nrf24l01, &cmd);

    nrf24l01_status_reg_t reg = {};
    err |= nrf24l01_get_status_reg(nrf24l01, &reg);
    reg.tx_ds = true;
    err |= nrf24l01_set_status_reg(nrf24l01, &reg);
  }

  return err;
}

uint8_t nrf24l01_get_allowed_payload_size(nrf24l01_t const *nrf24l01,
                                          nrf24l01_pipe_num_t pipe_num,
                                          size_t proposed_size,
                                          size_t *allowed_size) {
  assert(nrf24l01 && allowed_size);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_DYNPD, &data, sizeof(data));

  if (data & (1U << pipe_num)) {
    if (proposed_size < 1UL) {
      proposed_size = 1UL;
    } else if (proposed_size > 32UL) {
      proposed_size = 32;
    }

    *allowed_size = proposed_size;
  } else {
    err |= nrf24l01_bus_read_data(nrf24l01,
                                  NRF24L01_REG_ADDRESS_RX_PW_P0 + pipe_num,
                                  &data, sizeof(data));

    *allowed_size = data & 0x3FU;
  }

  return err;
}

nrf24l01_err_t nrf24l01_start_listening(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  nrf24l01_err_t err = nrf24l01_power_up(nrf24l01);

  nrf24l01_config_reg_t config_reg = {};
  err |= nrf24l01_get_config_reg(nrf24l01, &config_reg);
  config_reg.prim_rx = true;
  err |= nrf24l01_set_config_reg(nrf24l01, &config_reg);

  nrf24l01_status_reg_t status_reg = {};
  err |= nrf24l01_get_status_reg(nrf24l01, &status_reg);
  status_reg.rx_dr = true;
  status_reg.tx_ds = true;
  status_reg.max_rt = true;
  err |= nrf24l01_set_status_reg(nrf24l01, &status_reg);

  err |= nrf24l01_set_chip_enable(nrf24l01);

  nrf24l01_rx_addr_p0_reg_t p0_reg = {};
  err |= nrf24l01_get_rx_addr_p0_reg(nrf24l01, &p0_reg);
  if (p0_reg.rx_addr_p0) {
    nrf24l01_pipe_address_t pipe_address = {.address = p0_reg.rx_addr_p0};
    err |= nrf24l01_open_reading_pipe(nrf24l01, NRF24L01_PIPE_NUM_0,
                                      &pipe_address);
  } else {
    err |= nrf24l01_close_reading_pipe(nrf24l01, NRF24L01_PIPE_NUM_0);
  }

  return err;
}

nrf24l01_err_t nrf24l01_stop_listening(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  nrf24l01_err_t err = nrf24l01_reset_chip_enable(nrf24l01);

  nrf24l01_feature_reg_t feature_reg = {};
  err |= nrf24l01_get_feature_reg(nrf24l01, &feature_reg);

  if (feature_reg.en_ack_pay) {
    err |= nrf24l01_send_flush_tx_cmd(nrf24l01);
  }

  nrf24l01_config_reg_t config_reg = {};
  err |= nrf24l01_get_config_reg(nrf24l01, &config_reg);
  config_reg.prim_rx = false;
  err |= nrf24l01_set_config_reg(nrf24l01, &config_reg);

  err |= nrf24l01_close_reading_pipe(nrf24l01, NRF24L01_PIPE_NUM_0);

  return err;
}

nrf24l01_err_t nrf24l01_is_payload_available(nrf24l01_t const *nrf24l01,
                                             nrf24l01_pipe_num_t pipe_num,
                                             bool *is_payload_available) {
  assert(nrf24l01 && is_payload_available);

  nrf24l01_fifo_status_reg_t fifo_status_reg = {};
  nrf24l01_err_t err = nrf24l01_get_fifo_status_reg(nrf24l01, &fifo_status_reg);

  nrf24l01_status_reg_t status_reg = {};
  err |= nrf24l01_get_status_reg(nrf24l01, &status_reg);

  *is_payload_available =
      (fifo_status_reg.rx_empty ? status_reg.rx_p_no == pipe_num : false);

  return err;
}

nrf24l01_err_t
nrf24l01_open_writing_pipe(nrf24l01_t const *nrf24l01,
                           nrf24l01_pipe_address_t *pipe_address) {
  assert(nrf24l01 && pipe_address);

  nrf24l01_tx_addr_reg_t reg = {.tx_addr = pipe_address->address};

  return nrf24l01_set_tx_addr_reg(nrf24l01, &reg);
}

nrf24l01_err_t
nrf24l01_open_reading_pipe(nrf24l01_t const *nrf24l01,
                           nrf24l01_pipe_num_t pipe_num,
                           nrf24l01_pipe_address_t *pipe_address) {
  assert(nrf24l01 && pipe_address);

  uint8_t data[5] = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_EN_RXADDR, data, 1UL);
  data[0] |= (1U << pipe_num);
  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_EN_RXADDR, data,
                                 1UL);

  // address in big endian
  data[0] = (pipe_address->address >> 32U) & 0xFFU;
  data[1] = (pipe_address->address >> 24U) & 0xFFU;
  data[2] = (pipe_address->address >> 16U) & 0xFFU;
  data[3] = (pipe_address->address >> 8U) & 0xFFU;
  data[4] = pipe_address->address & 0xFFU;

  if (pipe_num < NRF24L01_PIPE_NUM_2) {
    err |= nrf24l01_bus_write_data(nrf24l01,
                                   NRF24L01_REG_ADDRESS_RX_ADDR_P0 + pipe_num,
                                   data, sizeof(data));
  } else {
    err |= nrf24l01_bus_write_data(nrf24l01,
                                   NRF24L01_REG_ADDRESS_RX_ADDR_P0 + pipe_num,
                                   data, sizeof(data) - 1UL);
    err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_SETUP_AW,
                                   data + 4, 1UL);
  }

  return err;
}

nrf24l01_err_t nrf24l01_close_reading_pipe(nrf24l01_t const *nrf24l01,
                                           nrf24l01_pipe_num_t pipe_num) {
  assert(nrf24l01);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_EN_RXADDR, &data, sizeof(data));
  data &= ~(1U << pipe_num);
  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_EN_RXADDR,
                                 &data, sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_close_writing_pipe(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  nrf24l01_tx_addr_reg_t reg = {.tx_addr = 0};

  return nrf24l01_set_tx_addr_reg(nrf24l01, &reg);
}

nrf24l01_err_t nrf24l01_power_up(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  nrf24l01_config_reg_t reg = {};

  nrf24l01_err_t err = nrf24l01_get_config_reg(nrf24l01, &reg);
  reg.pwr_up = true;
  err |= nrf24l01_set_config_reg(nrf24l01, &reg);

  return err;
}

nrf24l01_err_t nrf24l01_power_down(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  nrf24l01_config_reg_t reg = {};

  nrf24l01_err_t err = nrf24l01_get_config_reg(nrf24l01, &reg);
  reg.pwr_up = true;
  err |= nrf24l01_set_config_reg(nrf24l01, &reg);
  err |= nrf24l01_reset_chip_enable(nrf24l01);

  return err;
}

nrf24l01_err_t nrf24l01_reuse_tx(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  nrf24l01_config_reg_t reg = {};

  nrf24l01_err_t err = nrf24l01_reset_chip_enable(nrf24l01);
  err |= nrf24l01_get_config_reg(nrf24l01, &reg);
  reg.mask_max_rt = true;
  err |= nrf24l01_set_config_reg(nrf24l01, &reg);
  err |= nrf24l01_send_reuse_tx_pl_cmd(nrf24l01);
  err |= nrf24l01_set_chip_enable(nrf24l01);

  return err;
}

nrf24l01_err_t nrf24l01_set_chip_enable(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  return nrf24l01_gpio_write(nrf24l01, nrf24l01->config.chip_enable_pin, true);
}

nrf24l01_err_t nrf24l01_reset_chip_enable(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  return nrf24l01_gpio_write(nrf24l01, nrf24l01->config.chip_enable_pin, false);
}

nrf24l01_err_t nrf24l01_get_config_reg(nrf24l01_t const *nrf24l01,
                                       nrf24l01_config_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_CONFIG, &data, sizeof(data));

  reg->mask_rx_dr = (data >> 6U) & 0x01U;
  reg->mask_tx_ds = (data >> 5U) & 0x01U;
  reg->mask_max_rt = (data >> 4U) & 0x01U;
  reg->en_crc = (data >> 3U) & 0x01U;
  reg->crco = (data >> 2U) & 0x01U;
  reg->pwr_up = (data >> 1U) & 0x01U;
  reg->prim_rx = data & 0x01U;

  return err;
}

nrf24l01_err_t nrf24l01_set_config_reg(nrf24l01_t const *nrf24l01,
                                       nrf24l01_config_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_CONFIG, &data, sizeof(data));

  data &= ~((0x01U << 6U) | (0x01U << 5U) | (0x01U << 4U) | (0x01U << 3U) |
            (0x01U << 2U) | (0x01U << 1U) | 0x01U);

  data |= (reg->mask_rx_dr & 0x01U) << 6U;
  data |= (reg->mask_tx_ds & 0x01U) << 5U;
  data |= (reg->mask_max_rt & 0x01U) << 4U;
  data |= (reg->en_crc & 0x01U) << 3U;
  data |= (reg->crco & 0x01U) << 2U;
  data |= (reg->pwr_up & 0x01U) << 1U;
  data |= reg->prim_rx & 0x01U;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_CONFIG, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_en_aa_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_en_aa_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_EN_AA, &data, sizeof(data));

  reg->enaa_p5 = (data >> 5U) & 0x01U;
  reg->enaa_p4 = (data >> 4U) & 0x01U;
  reg->enaa_p3 = (data >> 3U) & 0x01U;
  reg->enaa_p2 = (data >> 2U) & 0x01U;
  reg->enaa_p1 = (data >> 1U) & 0x01U;
  reg->enaa_p0 = data & 0x01U;

  return err;
}

nrf24l01_err_t nrf24l01_set_en_aa_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_en_aa_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_EN_AA, &data, sizeof(data));

  data &= ~((0x01U << 5U) | (0x01U << 4U) | (0x01U << 3U) | (0x01U << 2U) |
            (0x01U << 1U) | 0x01U);

  data |= (reg->enaa_p5 & 0x01U) << 5U;
  data |= (reg->enaa_p4 & 0x01U) << 4U;
  data |= (reg->enaa_p3 & 0x01U) << 3U;
  data |= (reg->enaa_p2 & 0x01U) << 2U;
  data |= (reg->enaa_p1 & 0x01U) << 1U;
  data |= reg->enaa_p0 & 0x01U;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_EN_AA, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_en_rxaddr_reg(nrf24l01_t const *nrf24l01,
                                          nrf24l01_en_rxaddr_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_EN_RXADDR, &data, sizeof(data));

  reg->erx_p5 = (data >> 5U) & 0x01U;
  reg->erx_p4 = (data >> 4U) & 0x01U;
  reg->erx_p3 = (data >> 3U) & 0x01U;
  reg->erx_p2 = (data >> 2U) & 0x01U;
  reg->erx_p1 = (data >> 1U) & 0x01U;
  reg->erx_p0 = data & 0x01U;

  return err;
}

nrf24l01_err_t nrf24l01_set_en_rxaddr_reg(nrf24l01_t const *nrf24l01,
                                          nrf24l01_en_rxaddr_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_EN_RXADDR, &data, sizeof(data));

  data &= ~((0x01U << 5U) | (0x01U << 4U) | (0x01U << 3U) | (0x01U << 2U) |
            (0x01U << 1U) | 0x01U);

  data |= (reg->erx_p5 & 0x01U) << 5U;
  data |= (reg->erx_p4 & 0x01U) << 4U;
  data |= (reg->erx_p3 & 0x01U) << 3U;
  data |= (reg->erx_p2 & 0x01U) << 2U;
  data |= (reg->erx_p1 & 0x01U) << 1U;
  data |= reg->erx_p0 & 0x01U;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_EN_RXADDR,
                                 &data, sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_setup_aw_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_setup_aw_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_SETUP_AW, &data, sizeof(data));

  reg->aw = data & 0x03U;

  return err;
}

nrf24l01_err_t nrf24l01_set_setup_aw_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_setup_aw_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_SETUP_AW, &data, sizeof(data));

  data &= ~0x03U;

  data |= reg->aw & 0x03U;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_SETUP_AW, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_setup_retr_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_setup_retr_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_SETUP_RETR, &data, sizeof(data));

  reg->ard |= (data >> 4U) & 0x0FU;
  reg->arc |= data & 0x0FU;

  return err;
}

nrf24l01_err_t
nrf24l01_set_setup_retr_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_setup_retr_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  data |= (reg->ard & 0x0FU) << 4U;
  data |= reg->arc & 0x0FU;

  return nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_SETUP_RETR,
                                 &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_get_rf_ch_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_rf_ch_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RF_CH, &data, sizeof(data));

  reg->rf_ch = data & 0x7FU;

  return err;
}

nrf24l01_err_t nrf24l01_set_rf_ch_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_rf_ch_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RF_CH, &data, sizeof(data));

  data &= ~0xEFU;

  data |= reg->rf_ch & 0xEFU;

  return err;
}

nrf24l01_err_t nrf24l01_get_rf_setup_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rf_setup_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RF_SETUP, &data, sizeof(data));

  reg->cont_wave = (data >> 7U) & 0x01U;
  reg->rf_dr = (((data >> 3U) & 0x01U) << 1U) | ((data >> 5U) & 0x01U);
  reg->pll_lock = (data >> 4U) & 0x01U;
  reg->rf_pwr = (data >> 1U) & 0x03U;

  return err;
}

nrf24l01_err_t nrf24l01_set_rf_setup_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rf_setup_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RF_SETUP, &data, sizeof(data));

  data &= ~((0x01U << 7U) | (0x05U << 3U) | (0x01U << 4U) | (0x03U << 1U));

  data |= (reg->cont_wave & 0x01U) << 7U;
  data |= (reg->rf_dr & 0x01U) << 5U;
  data |= ((reg->rf_dr >> 1U) & 0x01U) << 3U;
  data |= (reg->pll_lock & 0x01U) << 4U;
  data |= (reg->rf_pwr & 0x03U) << 1U;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RF_SETUP, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_status_reg(nrf24l01_t const *nrf24l01,
                                       nrf24l01_status_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_STATUS, &data, sizeof(data));

  reg->rx_dr = (data >> 6U) & 0x01U;
  reg->tx_ds = (data >> 5U) & 0x01U;
  reg->max_rt = (data >> 4U) & 0x01U;
  reg->rx_p_no = (data >> 1U) & 0x07U;
  reg->tx_full = data & 0x01U;

  return err;
}

nrf24l01_err_t nrf24l01_set_status_reg(nrf24l01_t const *nrf24l01,
                                       nrf24l01_status_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_STATUS, &data, sizeof(data));

  data &=
      ~((0x01U << 6U) | (0x01U << 5U) | (0x01U << 4U) | (0x07U << 1U) | 0x01U);

  data |= (reg->rx_dr & 0x01U) << 6U;
  data |= (reg->tx_ds & 0x01U) << 5U;
  data |= (reg->max_rt & 0x01U) << 4U;
  data |= (reg->rx_p_no & 0x07U) << 1U;
  data |= reg->tx_ds & 0x01U;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_STATUS, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_observe_tx_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_observe_tx_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_OBSERVE_TX, &data, sizeof(data));

  reg->plos_cnt = (data >> 4U) & 0x0FU;
  reg->arc_cnt = data & 0x0FU;

  return err;
}

nrf24l01_err_t nrf24l01_get_rpd_reg(nrf24l01_t const *nrf24l01,
                                    nrf24l01_rpd_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RPD, &data, sizeof(data));

  reg->rpd = data & 0x01U;

  return err;
}

nrf24l01_err_t nrf24l01_get_rx_addr_p0_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_p0_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data[5] = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P0, data, sizeof(data));

  reg->rx_addr_p0 =
      ((((uint64_t)data[0] & 0xFFU) << 32U) |
       (((uint64_t)data[1] & 0xFFU) << 24U) |
       (((uint64_t)data[2] & 0xFFU) << 16U) |
       (((uint64_t)data[3] & 0xFFU) << 8U) | ((uint64_t)data[4] & 0xFFU));

  return err;
}

nrf24l01_err_t
nrf24l01_set_rx_addr_p0_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_rx_addr_p0_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data[5] = {};

  data[0] = (reg->rx_addr_p0 >> 32U) & 0xFFU;
  data[1] = (reg->rx_addr_p0 >> 24U) & 0xFFU;
  data[2] = (reg->rx_addr_p0 >> 16U) & 0xFFU;
  data[3] = (reg->rx_addr_p0 >> 8U) & 0xFFU;
  data[4] = reg->rx_addr_p0 & 0xFFU;

  return nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P0,
                                 data, sizeof(data));
}

nrf24l01_err_t nrf24l01_get_rx_addr_p1_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_p1_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data[5] = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P1, data, sizeof(data));

  reg->rx_addr_p1 =
      ((((uint64_t)data[0] & 0xFFU) << 32U) |
       (((uint64_t)data[1] & 0xFFU) << 24U) |
       (((uint64_t)data[2] & 0xFFU) << 16U) |
       (((uint64_t)data[3] & 0xFFU) << 8U) | ((uint64_t)data[4] & 0xFFU));

  return err;
}

nrf24l01_err_t
nrf24l01_set_rx_addr_p1_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_rx_addr_p1_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data[5] = {};

  data[0] = (reg->rx_addr_p1 >> 32U) & 0xFFU;
  data[1] = (reg->rx_addr_p1 >> 24U) & 0xFFU;
  data[2] = (reg->rx_addr_p1 >> 16U) & 0xFFU;
  data[3] = (reg->rx_addr_p1 >> 8U) & 0xFFU;
  data[4] = reg->rx_addr_p1 & 0xFFU;

  return nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P1,
                                 data, sizeof(data));
}

nrf24l01_err_t nrf24l01_get_rx_addr_p2_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_p2_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P2, &data, sizeof(data));

  reg->rx_addr_p2 = data & 0xFFU;

  return err;
}

nrf24l01_err_t
nrf24l01_set_rx_addr_p2_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_rx_addr_p2_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  data = reg->rx_addr_p2 & 0xFFU;

  return nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P2,
                                 &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_get_rx_addr_p3_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_p3_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P3, &data, sizeof(data));

  reg->rx_addr_p3 = data & 0xFFU;

  return err;
}

nrf24l01_err_t
nrf24l01_set_rx_addr_p3_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_rx_addr_p3_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  data = reg->rx_addr_p3 & 0xFFU;

  return nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P3,
                                 &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_get_rx_addr_p4_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_p4_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P4, &data, sizeof(data));

  reg->rx_addr_p4 = data & 0xFFU;

  return err;
}

nrf24l01_err_t
nrf24l01_set_rx_addr_p4_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_rx_addr_p4_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  data = reg->rx_addr_p4 & 0xFFU;

  return nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P4,
                                 &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_get_rx_addr_p5_reg(nrf24l01_t const *nrf24l01,
                                           nrf24l01_rx_addr_5_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P5, &data, sizeof(data));

  reg->rx_addr_p5 = data & 0xFFU;

  return err;
}

nrf24l01_err_t
nrf24l01_set_rx_addr_p5_reg(nrf24l01_t const *nrf24l01,
                            nrf24l01_rx_addr_5_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  data = reg->rx_addr_p5 & 0xFFU;

  return nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_ADDR_P5,
                                 &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_get_tx_addr_reg(nrf24l01_t const *nrf24l01,
                                        nrf24l01_tx_addr_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data[5] = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_TX_ADDR, data, sizeof(data));

  reg->tx_addr =
      ((((uint64_t)data[0] & 0xFFU) << 32U) |
       (((uint64_t)data[1] & 0xFFU) << 24U) |
       (((uint64_t)data[2] & 0xFFU) << 16U) |
       (((uint64_t)data[3] & 0xFFU) << 8U) | ((uint64_t)data[4] & 0xFFU));

  return err;
}

nrf24l01_err_t nrf24l01_set_tx_addr_reg(nrf24l01_t const *nrf24l01,
                                        nrf24l01_tx_addr_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data[5] = {};

  data[0] = (reg->tx_addr >> 32U) & 0xFFU;
  data[1] = (reg->tx_addr >> 24U) & 0xFFU;
  data[2] = (reg->tx_addr >> 16U) & 0xFFU;
  data[3] = (reg->tx_addr >> 8U) & 0xFFU;
  data[4] = reg->tx_addr & 0xFFU;

  return nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_TX_ADDR, data,
                                 sizeof(data));
}

nrf24l01_err_t nrf24l01_get_rx_pw_p0_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p0_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P0, &data, sizeof(data));

  reg->rx_pw_p0 = data & 0x3FU;

  return err;
}

nrf24l01_err_t nrf24l01_set_rx_pw_p0_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p0_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P0, &data, sizeof(data));

  data &= ~(0x3FU);

  data |= reg->rx_pw_p0 & 0x3FU;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P0, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_rx_pw_p1_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p1_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P1, &data, sizeof(data));

  reg->rx_pw_p1 = data & 0x3FU;

  return err;
}

nrf24l01_err_t nrf24l01_set_rx_pw_p1_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p1_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P1, &data, sizeof(data));

  data &= ~(0x3FU);

  data |= reg->rx_pw_p1 & 0x3FU;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P1, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_rx_pw_p2_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p2_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P2, &data, sizeof(data));

  reg->rx_pw_p2 = data & 0x3FU;

  return err;
}

nrf24l01_err_t nrf24l01_set_rx_pw_p2_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p2_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P2, &data, sizeof(data));

  data &= ~(0x3FU);

  data |= reg->rx_pw_p2 & 0x3FU;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P2, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_rx_pw_p3_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p3_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P3, &data, sizeof(data));

  reg->rx_pw_p3 = data & 0x3FU;

  return err;
}

nrf24l01_err_t nrf24l01_set_rx_pw_p3_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p3_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P3, &data, sizeof(data));

  data &= ~(0x3FU);

  data |= reg->rx_pw_p3 & 0x3FU;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P3, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_rx_pw_p4_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p4_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P4, &data, sizeof(data));

  reg->rx_pw_p4 = data & 0x3FU;

  return err;
}

nrf24l01_err_t nrf24l01_set_rx_pw_p4_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p4_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P4, &data, sizeof(data));

  data &= ~(0x3FU);

  data |= reg->rx_pw_p4 & 0x3FU;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P4, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_rx_pw_p5_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p5_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P5, &data, sizeof(data));

  reg->rx_pw_p5 = data & 0x3FU;

  return err;
}

nrf24l01_err_t nrf24l01_set_rx_pw_p5_reg(nrf24l01_t const *nrf24l01,
                                         nrf24l01_rx_pw_p5_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P5, &data, sizeof(data));

  data &= ~(0x3FU);

  data |= reg->rx_pw_p5 & 0x3FU;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_RX_PW_P5, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_fifo_status_reg(nrf24l01_t const *nrf24l01,
                                            nrf24l01_fifo_status_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_FIFO_STATUS, &data, sizeof(data));

  reg->tx_reuse = (data >> 6U) & 0x01U;
  reg->tx_full = (data >> 5U) & 0x01U;
  reg->tx_empty = (data >> 4U) & 0x01U;
  reg->rx_full = (data >> 1U) & 0x01U;
  reg->rx_empty = data & 0x01U;

  return err;
}

nrf24l01_err_t nrf24l01_get_dynpd_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_dynpd_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_DYNPD, &data, sizeof(data));

  reg->dpl_p5 = (data >> 5U) & 0x01U;
  reg->dpl_p4 = (data >> 4U) & 0x01U;
  reg->dpl_p3 = (data >> 3U) & 0x01U;
  reg->dpl_p2 = (data >> 2U) & 0x01U;
  reg->dpl_p1 = (data >> 1U) & 0x01U;
  reg->dpl_p0 = data & 0x01U;

  return err;
}

nrf24l01_err_t nrf24l01_set_dynpd_reg(nrf24l01_t const *nrf24l01,
                                      nrf24l01_dynpd_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_DYNPD, &data, sizeof(data));

  data &= ~((0x01U << 5U) | (0x01U << 4U) | (0x01U << 3U) | (0x01U << 2U) |
            (0x01U << 1U) | 0x01U);

  data |= (reg->dpl_p5 & 0x01U) << 5U;
  data |= (reg->dpl_p4 & 0x01U) << 4U;
  data |= (reg->dpl_p3 & 0x01U) << 3U;
  data |= (reg->dpl_p2 & 0x01U) << 2U;
  data |= (reg->dpl_p1 & 0x01U) << 1U;
  data |= reg->dpl_p0 & 0x01U;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_DYNPD, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_get_feature_reg(nrf24l01_t const *nrf24l01,
                                        nrf24l01_feature_reg_t *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_FEATURE, &data, sizeof(data));

  reg->en_dpl = (data >> 2U) & 0x01U;
  reg->en_ack_pay = (data >> 1U) & 0x01U;
  reg->en_dyn_ack = data & 0x01U;

  return err;
}

nrf24l01_err_t nrf24l01_set_feature_reg(nrf24l01_t const *nrf24l01,
                                        nrf24l01_feature_reg_t const *reg) {
  assert(nrf24l01 && reg);

  uint8_t data = {};

  nrf24l01_err_t err = nrf24l01_bus_read_data(
      nrf24l01, NRF24L01_REG_ADDRESS_FEATURE, &data, sizeof(data));

  data &= ~((0x01 << 2U) | (0x01U << 1U) | 0x01U);

  data |= (reg->en_dpl & 0x01U) << 2U;
  data |= (reg->en_ack_pay & 0x01U) << 1U;
  data |= reg->en_dyn_ack & 0x01U;

  err |= nrf24l01_bus_write_data(nrf24l01, NRF24L01_REG_ADDRESS_FEATURE, &data,
                                 sizeof(data));

  return err;
}

nrf24l01_err_t nrf24l01_send_rx_payload_cmd(nrf24l01_t const *nrf24l01,
                                            nrf24l01_rx_payload_cmd_t *cmd) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_RX_PAYLOAD;

  nrf24l01_err_t err = nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
  err |= nrf24l01_bus_receive(nrf24l01, cmd->payload, sizeof(cmd->payload));

  return err;
}

nrf24l01_err_t
nrf24l01_send_tx_payload_cmd(nrf24l01_t const *nrf24l01,
                             nrf24l01_tx_payload_cmd_t const *cmd) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_TX_PAYLOAD;

  nrf24l01_err_t err = nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
  err |= nrf24l01_bus_transmit(nrf24l01, cmd->payload, sizeof(cmd->payload));

  return err;
}

nrf24l01_err_t nrf24l01_send_flush_tx_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_FLUSH_TX;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_send_flush_rx_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_FLUSH_RX;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_send_reuse_tx_pl_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_REUSE_TX_PL;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_send_activate_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_ACTIVATE;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_send_r_rx_pl_wid_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_R_RX_PL_WID;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_send_w_ack_payload_p0_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_W_ACK_PAYLOAD_P0;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_send_w_ack_payload_p1_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_W_ACK_PAYLOAD_P1;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_send_w_ack_payload_p2_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_W_ACK_PAYLOAD_P2;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_send_w_ack_payload_p3_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_W_ACK_PAYLOAD_P3;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_send_w_ack_payload_p4_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_W_ACK_PAYLOAD_P4;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_send_w_ack_payload_p5_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_W_ACK_PAYLOAD_P5;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t
nrf24l01_send_w_tx_payload_noack_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_W_TX_PAYLOAD_NOACK;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}

nrf24l01_err_t nrf24l01_send_nop_cmd(nrf24l01_t const *nrf24l01) {
  assert(nrf24l01);

  uint8_t data = NRF24L01_CMD_NOP;

  return nrf24l01_bus_transmit(nrf24l01, &data, sizeof(data));
}
