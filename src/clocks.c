#include "clocks.h"
#include <nrfx_timer.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

static volatile uint32_t m_counter = 1;
const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);

#define LAST_RESET_LIMIT 10

extern uint8_t paired_addr[8];

extern uint8_t last_reset;
extern uint16_t led_clock;
extern uint32_t led_clock_offset;
extern bool timer_state;

bool esb_state = false;
bool send_data = false;

extern struct esb_payload tx_payload;

void clear_timer() {
    nrfx_timer_clear(&m_timer);
}
void resume_timer() {
    nrfx_timer_resume(&m_timer);
}


int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		// LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		// LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			// LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);

	// LOG_DBG("HF clock started");
	return 0;

    
}

static void timer_handler(nrf_timer_event_t event_type, void *p_context) {
	if (event_type == NRF_TIMER_EVENT_COMPARE1 && esb_state == true) {
		if (last_reset < LAST_RESET_LIMIT) {
			last_reset++;
			if (send_data) { // scuffed check
				esb_write_payload(&tx_payload); // Add transmission to queue
				esb_start_tx();
				send_data = false;
			}
//			esb_flush_tx();
		} else {
			esb_disable();
			esb_initialize_rx();
			esb_start_rx();
			esb_state = false;
			nrfx_timer_pause(&m_timer);
			timer_state = false;
			LOG_INF("timer reset elapsed");
		}
	} else if (event_type == NRF_TIMER_EVENT_COMPARE2 && esb_state == true) {
		esb_disable();
		esb_initialize_rx();
		esb_start_rx();
		esb_state = false;
	} else if (event_type == NRF_TIMER_EVENT_COMPARE3 && esb_state == false) {
		esb_stop_rx();
		esb_disable();
		esb_initialize();
		esb_state = true;
	}
}

void timer_init(void) {
//	nrfx_err_t err;
	nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG(1000000);  
	//timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
	//timer_cfg.mode = NRF_TIMER_MODE_TIMER;
	//timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_16;
	//timer_cfg.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY;
	//timer_cfg.p_context = NULL;
	nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
	uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, 3);
	nrfx_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
	LOG_INF("timer at %d", ticks * (paired_addr[1]*2 + 3) / 21); // TODO: temp set max 8
	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL1, ticks * (paired_addr[1]*2 + 3) / 21, true); // timeslot to send data  TODO: temp set max 8
	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL2, ticks * 19 / 21, true); // switch to rx
	nrfx_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL3, ticks * 2 / 21, true); // switch to tx
	nrfx_timer_enable(&m_timer);
	IRQ_DIRECT_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, 0);
	irq_enable(TIMER1_IRQn);
	timer_state = true;
}
