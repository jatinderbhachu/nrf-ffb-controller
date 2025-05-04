/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */
#include <dk_buttons_and_leds.h>
#include <esb.h>
#include <math.h>
#include <nrf.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>

#if defined(CONFIG_CLOCK_CONTROL_NRF2)
#include <hal/nrf_lrcconf.h>
#endif

/* change this to any other UART peripheral if desired */
// #define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define UART_DEVICE_NODE DT_CHOSEN(nordic_nrf_uarte)

#define UART_MSG_SIZE 16

struct ControllerState {
  float encoder_angle;
  uint16_t axis1;
  uint16_t axis2;
  uint8_t buttons;
};

K_MSGQ_DEFINE(uart_recv_msgq, UART_MSG_SIZE, 10, 4);
K_MSGQ_DEFINE(uart_send_msgq, UART_MSG_SIZE, 10, 4);
K_MSGQ_DEFINE(esb_msgq, sizeof(struct esb_payload), 10, 1);

void esb_recv(void);
#define ESB_RECV_THREAD_STACKSIZE 2048
#define ESB_RECV_THREAD_PRIO 14
K_THREAD_DEFINE(esb_recv_tid, ESB_RECV_THREAD_STACKSIZE, esb_recv, NULL, NULL,
                NULL, ESB_RECV_THREAD_PRIO, 0, 0);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

// arbitrary magic header to identify start of message in uart
static uint8_t UART_MAGIC_HEADER[4] = {0x42, 0x03, 0x02, 0x01};
static char uart_rx_buf[UART_MSG_SIZE];
static int uart_rx_buf_pos = 0;

static struct esb_payload rx_payload;
static struct esb_payload tx_payload =
    ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08);

#define _RADIO_SHORTS_COMMON                                                   \
  (RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk |               \
   RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_RSSISTOP_Msk)

void event_handler(struct esb_evt const *event) {
  switch (event->evt_id) {
  case ESB_EVENT_TX_SUCCESS:
    break;
  case ESB_EVENT_TX_FAILED:
    esb_flush_tx();
    // esb_start_tx();
    break;
  case ESB_EVENT_RX_RECEIVED:
    if (esb_read_rx_payload(&rx_payload) == 0) {
      // printf("rx torque\n");
      while (k_msgq_put(&esb_msgq, &rx_payload, K_NO_WAIT) != 0) {
        /* message queue is full: purge old data & try again */
        k_msgq_purge(&esb_msgq);
      }
    } else {
      // err
    }
    break;
  }
}

int clocks_start(void) {
  int err;
  int res;
  struct onoff_manager *clk_mgr;
  struct onoff_client clk_cli;

  clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
  if (!clk_mgr) {
    return -ENXIO;
  }

  sys_notify_init_spinwait(&clk_cli.notify);

  err = onoff_request(clk_mgr, &clk_cli);
  if (err < 0) {
    return err;
  }

  do {
    err = sys_notify_fetch_result(&clk_cli.notify, &res);
    if (!err && res) {
      return res;
    }
  } while (err);

#if defined(NRF54L15_XXAA)
  /* MLTPAN-20 */
  nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

  return 0;
}

bool esb_send(uint8_t *packet, uint32_t packet_length) {
  memcpy(tx_payload.data, packet, packet_length);
  tx_payload.length = packet_length;
  tx_payload.pipe = 0;
  tx_payload.noack = false;
  esb_write_payload(&tx_payload);
}

int esb_initialize(void) {
  int err;
  /* These are arbitrary default addresses. In end user products
   * different addresses should be used for each set of devices.
   */
  uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
  uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
  uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

  struct esb_config esb_config = ESB_DEFAULT_CONFIG;
  esb_config.mode = ESB_MODE_PTX;
  esb_config.event_handler = event_handler;
  esb_config.selective_auto_ack = false; // require all packets to acknowledge
  esb_config.retransmit_count = 0;
  err = esb_init(&esb_config);

  if (err) {
    return err;
  }

  err = esb_set_base_address_0(base_addr_0);
  if (err) {
    return err;
  }

  err = esb_set_base_address_1(base_addr_1);
  if (err) {
    return err;
  }

  err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
  if (err) {
    return err;
  }

  // err = esb_start_rx();
  // printf("start rx %d\n", err);

  err = esb_start_tx();
  printf("start tx %d\n", err);

  return 0;
}

void serial_cb(const struct device *dev, void *user_data) {
  uint8_t c;

  uart_irq_update(uart_dev);

  // read magic header + struct Controller state
  if (uart_irq_rx_ready(uart_dev)) {
    /* read until FIFO empty */
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
      if (uart_rx_buf_pos < 4 && c == UART_MAGIC_HEADER[uart_rx_buf_pos]) {
        uart_rx_buf_pos++;
      } else if (uart_rx_buf_pos >= 4) {
        uart_rx_buf[uart_rx_buf_pos++] = c;
        if (uart_rx_buf_pos == 16) {
          k_msgq_put(&uart_recv_msgq, &uart_rx_buf, K_NO_WAIT);
          uart_rx_buf_pos = 0;
        }
      } else {
        uart_rx_buf_pos = 0;
      }
    }
  }
}

void uart_initialize() {
  printf("Begin uart_initialize\n");
  if (!device_is_ready(uart_dev)) {
    printk("UART device not found!\n");
    return;
  }

  /* configure interrupt and callback to receive data */
  int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

  if (ret < 0) {
    return;
  }

  uart_irq_rx_enable(uart_dev);
}

// receives torque updates and sends it to motor controller over uart
void esb_recv(void) {
  printf("Begin esb_recv\n");
  struct esb_payload payload;
  while (true) {
    if (k_msgq_get(&esb_msgq, &payload, K_NO_WAIT) != 0) {
      k_sleep(K_MSEC(1));
      continue;
    }
    int16_t torque = *(uint16_t *)(payload.data);
    uint8_t data[UART_MSG_SIZE];
    data[0] = UART_MAGIC_HEADER[0];
    data[1] = UART_MAGIC_HEADER[1];
    data[2] = UART_MAGIC_HEADER[2];
    data[3] = UART_MAGIC_HEADER[3];
    memcpy(&data[4], &torque, sizeof(torque));
    // printf("send %d\n", torque);

    for (int i = 0; i < UART_MSG_SIZE; i++) {
      uart_poll_out(uart_dev, data[i]);
    }
  }
}

int main(void) {
  int err;

  err = clocks_start();
  if (err) {
    return 0;
  }

  uart_initialize();

  err = esb_initialize();
  if (err) {
    return 0;
  }

  while (1) {

    char data[UART_MSG_SIZE];

#if 1
    // send controller state to other nrf receiver
    while (k_msgq_get(&uart_recv_msgq, &data, K_FOREVER) == 0) {
      struct ControllerState *state = (struct ControllerState *)&data[4];
      // printf("%X,%X,%X,%X %X,%X,%X,%X %X,%X,%X,%X %X,%X,%X,%X\n", data[0],
      //        data[1], data[2], data[3], data[4], data[5], data[6], data[7],
      //        data[8], data[9], data[10], data[11], data[12], data[13],
      //        data[14], data[15]);

      // printf("%d,%d,%1.2f,%u\n", state->axis1, state->axis2,
      //        state->encoder_angle, state->buttons);
      esb_send(state, sizeof(struct ControllerState));
      k_sleep(K_USEC(5));
    }
#else // for testing
    int64_t time_ms = k_uptime_get();

    struct ControllerState state = {
        .encoder_angle = sin(time_ms / 100.0f),
        .axis1 = 0,
        .axis2 = 0,
        .buttons = 0,
    };
    esb_send(&state, sizeof(struct ControllerState));
    k_sleep(K_MSEC(1));
#endif
  }
}
