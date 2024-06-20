#include <esb.h>
#include "../clocks.h"
#include "radio.h"



bool timer_state = false;

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0,
														  0, 0, 0, 0, 0, 0, 0, 0);

// this was randomly generated
uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

uint8_t paired_addr[8] = {0,0,0,0,0,0,0,0};

uint8_t base_addr_0[4] = {0,0,0,0};
uint8_t base_addr_1[4] = {0,0,0,0};
uint8_t addr_prefix[8] = {0,0,0,0,0,0,0,0};

uint8_t last_reset = 0;

uint16_t led_clock = 0;
uint32_t led_clock_offset = 0;


void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_INF("TX FAILED");
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {
			if (paired_addr[0] == 0x00) {
				if (rx_payload.length == 8) {
					for (int i = 0; i < 8; i++) {
						paired_addr[i] = rx_payload.data[i];
					}
				}
			} else {
				if (rx_payload.length == 4) {
					if (timer_state == false) {
						resume_timer();
						timer_state = true;
					}
					clear_timer();
					last_reset = 0;
					led_clock = (rx_payload.data[0] << 8) + rx_payload.data[1]; // sync led flashes :)
					led_clock_offset = 0;
					//LOG_INF("RX, timer reset");
				}
			}
		}
		break;
	}
}



int esb_initialize(void)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	// config.protocol = ESB_PROTOCOL_ESB_DPL;
	// config.mode = ESB_MODE_PTX;
	config.event_handler = event_handler;
	// config.bitrate = ESB_BITRATE_2MBPS;
	// config.crc = ESB_CRC_16BIT;
	config.tx_output_power = 4;
	// config.retransmit_delay = 600;
	config.retransmit_count = 0;
	config.tx_mode = ESB_TXMODE_MANUAL;
	// config.payload_length = 32;
	config.selective_auto_ack = true;

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	err = esb_init(&config);

	if (err)
	{
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err)
	{
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err)
	{
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err)
	{
		return err;
	}

	return 0;
}

int esb_initialize_rx(void)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	// config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	// config.bitrate = ESB_BITRATE_2MBPS;
	// config.crc = ESB_CRC_16BIT;
	config.tx_output_power = 4;
	// config.retransmit_delay = 600;
	// config.retransmit_count = 3;
	// config.tx_mode = ESB_TXMODE_AUTO;
	// config.payload_length = 32;
	config.selective_auto_ack = true;

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	err = esb_init(&config);

	if (err)
	{
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err)
	{
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err)
	{
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err)
	{
		return err;
	}

	return 0;
}