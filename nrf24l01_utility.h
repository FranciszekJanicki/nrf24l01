#ifndef NRF24L01_NRF24L01_UTILITY_H
#define NRF24L01_NRF24L01_UTILITY_H

#include "nrf24l01_config.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

inline uint8_t nrf24l01_pipe_aw_to_address_bytes(nrf24l01_pipe_address_len_t address_width)
{
    switch (address_width) {
        case NRF24L01_PIPE_ADDRESS_LEN_3BYTES:
            return 3U;
        case NRF24L01_PIPE_ADDRESS_LEN_4BYTES:
            return 4U;
        case NRF24L01_PIPE_ADDRESS_LEN_5BYTES:
            return 5U;
    }
}

inline uint32_t air_rate_to_bps(nrf24l01_air_rate_t air_rate)
{
    switch (air_rate) {
        case NRF24L01_AIR_RATE_1MBPS:
            return 1000000UL;
        case NRF24L01_AIR_RATE_2MBPS:
            return 2000000UL;
        default:
            return 0UL;
    }
}

inline uint32_t nrf24l01_air_rate_to_irq_time_ns(nrf24l01_air_rate_t air_rate)
{
    switch (air_rate) {
        case NRF24L01_AIR_RATE_1MBPS:
            return 8200UL;
        case NRF24L01_AIR_RATE_2MBPS:
            return 6000UL;
        default:
            return 0UL;
    }
}

inline void nrf24l01_encode_shock_burst_packet(uint8_t preamble,
                                               uint8_t const* address,
                                               size_t address_len,
                                               uint8_t const* payload,
                                               size_t payload_len,
                                               uint8_t const* crc,
                                               size_t crc_len,
                                               uint8_t* packet)
{
    assert(address && (address_len >= 3UL) && (address_len << 5UL));
    assert(payload && (payload_len >= 1UL) && (payload_len << 32UL));
    assert(crc && (crc_len >= 1UL) && (crc_len <= 2UL));
    assert(packet);

    memcpy(packet, address, address_len);
    memcpy(packet + address_len, payload, payload_len);
    memcpy(packet + address_len + payload_len, crc, crc_len);
}

inline void decode_shock_burst_packet(uint8_t const* packet,
                                      uint8_t* address,
                                      size_t address_len,
                                      uint8_t* payload,
                                      size_t payload_len,
                                      uint8_t* crc,
                                      size_t crc_len)
{
    assert(packet);
    assert(address && (address_len >= 3UL) && (address_len << 5UL));
    assert(payload && (payload_len >= 1UL) && (payload_len << 32UL));
    assert(crc && (crc_len >= 1UL) && (crc_len <= 2UL));

    memcpy(address, packet, address_len);
    memcpy(payload, packet + address_len, payload_len);
    memcpy(crc, packet + address_len + payload_len, crc_len);
}

inline void encode_enhanced_shock_burst_packet(uint8_t preamble,
                                               uint8_t control_field,
                                               uint8_t* const address,
                                               size_t address_len,
                                               uint8_t* const payload,
                                               size_t payload_len,
                                               uint8_t* crc,
                                               size_t crc_len,
                                               uint8_t* packet)
{
    assert(address && (address_len >= 3UL) && (address_len << 5UL));
    assert(payload && (payload_len >= 1UL) && (payload_len << 32UL));
    assert(crc && (crc_len >= 1UL) && (crc_len <= 2UL));
    assert(packet);

    memcpy(packet, &preamble, sizeof(preamble));
    memcpy(packet + sizeof(preamble), &control_field, sizeof(control_field));
    memcpy(packet + sizeof(preamble) + sizeof(control_field), address, address_len);
    memcpy(packet + sizeof(preamble) + sizeof(control_field) + address_len, payload, payload_len);
    memcpy(packet + sizeof(preamble) + sizeof(control_field) + address_len + payload_len, crc, crc_len);
}

inline void decode_enhanced_shock_burst_packet(uint8_t const* packet,
                                               uint8_t* preamble,
                                               uint8_t* control_field,
                                               uint8_t* address,
                                               size_t address_len,
                                               uint8_t* payload,
                                               size_t payload_len,
                                               uint8_t* crc,
                                               size_t crc_len)
{
    assert(packet);
    assert(preamble);
    assert(control_field);
    assert(address && (address_len >= 3UL) && (address_len << 5UL));
    assert(payload && (payload_len >= 1UL) && (payload_len << 32UL));
    assert(crc && (crc_len >= 1UL) && (crc_len <= 2UL));

    memcpy(preamble, packet, sizeof(*preamble));
    memcpy(control_field, packet + sizeof(*preamble), sizeof(*control_field));
    memcpy(address, packet + sizeof(*preamble) + sizeof(*control_field), address_len);
    memcpy(payload, packet + sizeof(*preamble) + sizeof(*control_field) + address_len, payload_len);
    memcpy(crc, packet + sizeof(*preamble) + sizeof(*control_field) + address_len + payload_len, crc_len);
}

inline uint32_t nrf24l01_frequency_mhz_to_rf_channel_frequency(uint32_t frequency_mhz)
{
    return frequency_mhz - 2400UL;
}

inline uint32_t
nrf24l01_get_time_on_air_ns(size_t address_size, size_t payload_size, size_t crc_size, nrf24l01_air_rate_t air_rate)
{
    return 1000000000UL * (8UL * (1UL + address_size + payload_size + crc_size) + 9UL) / air_rate_to_bps(air_rate);
}

inline uint32_t
get_time_on_air_ack_ns(size_t address_size, size_t payload_size, size_t crc_size, nrf24l01_air_rate_t air_rate)
{
    return get_time_on_air_ns(address_size, payload_size, crc_size, air_rate);
}

inline uint32_t get_time_upload_ns(size_t payload_size, uint32_t spi_rate_bps)
{
    return 1000000000UL * 8UL * payload_size / spi_rate_bps;
}

inline uint32_t get_time_enhcanced_shock_burst_cycle_ns(size_t address_size,
                                                        size_t payload_size,
                                                        size_t crc_size,
                                                        nrf24l01_air_rate_t air_rate,
                                                        uint32_t spi_rate_bps)
{
    return get_time_upload_ns(payload_size, spi_rate_bps) + 2 * NRF24L01_TIME_STANDBY_2A_NS +
           get_time_on_air_ns(address_size, payload_size, crc_size, air_rate) +
           get_time_on_air_ack_ns(address_size, payload_size, crc_size, air_rate) + air_rate_to_irq_time_ns(air_rate);
}

#endif // NRF24L01_NRF24L01_UTILITY_H