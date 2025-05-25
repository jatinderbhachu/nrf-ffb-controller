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
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>

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

const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

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

#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY 2
#define STATE_POLL_THREAD_STACK_SIZE 512
#define STATE_POLL_THREAD_PRIORITY 2
#define LED_MSG_ID 0x10
#define COUNTER_MSG_ID 0x12345
#define SET_LED 1
#define RESET_LED 0
#define SLEEP_TIME K_MSEC(250)

K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(poll_state_stack, STATE_POLL_THREAD_STACK_SIZE);

struct k_thread rx_thread_data;
struct k_thread poll_state_thread_data;
struct k_work_poll change_led_work;
struct k_work state_change_work;
enum can_state current_state;
struct can_bus_err_cnt current_err_cnt;

CAN_MSGQ_DEFINE(change_led_msgq, 2);
CAN_MSGQ_DEFINE(counter_msgq, 2);

static struct k_poll_event change_led_events[1] = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                    K_POLL_MODE_NOTIFY_ONLY, &change_led_msgq,
                                    0)};

void tx_irq_callback(const struct device *dev, int error, void *arg) {
  char *sender = (char *)arg;

  ARG_UNUSED(dev);

  if (error != 0) {
    printf("Callback! error-code: %d\nSender: %s\n", error, sender);
  }
}

void rx_thread(void *arg1, void *arg2, void *arg3) {
  ARG_UNUSED(arg1);
  ARG_UNUSED(arg2);
  ARG_UNUSED(arg3);
  const struct can_filter filter = {
      .flags = CAN_FILTER_IDE, .id = COUNTER_MSG_ID, .mask = CAN_EXT_ID_MASK};
  struct can_frame frame;
  int filter_id;

  filter_id = can_add_rx_filter_msgq(can_dev, &counter_msgq, &filter);
  printf("Counter filter id: %d\n", filter_id);

  while (1) {
    k_msgq_get(&counter_msgq, &frame, K_FOREVER);

    if (IS_ENABLED(CONFIG_CAN_ACCEPT_RTR) &&
        (frame.flags & CAN_FRAME_RTR) != 0U) {
      continue;
    }

    if (frame.dlc != 2U) {
      printf("Wrong data length: %u\n", frame.dlc);
      continue;
    }

    printf("Counter received: %u\n",
           sys_be16_to_cpu(UNALIGNED_GET((uint16_t *)&frame.data)));
  }
}

char *state_to_str(enum can_state state) {
  switch (state) {
  case CAN_STATE_ERROR_ACTIVE:
    return "error-active";
  case CAN_STATE_ERROR_WARNING:
    return "error-warning";
  case CAN_STATE_ERROR_PASSIVE:
    return "error-passive";
  case CAN_STATE_BUS_OFF:
    return "bus-off";
  case CAN_STATE_STOPPED:
    return "stopped";
  default:
    return "unknown";
  }
}

void poll_state_thread(void *unused1, void *unused2, void *unused3) {
  struct can_bus_err_cnt err_cnt = {0, 0};
  struct can_bus_err_cnt err_cnt_prev = {0, 0};
  enum can_state state_prev = CAN_STATE_ERROR_ACTIVE;
  enum can_state state;
  int err;

  while (1) {
    err = can_get_state(can_dev, &state, &err_cnt);
    if (err != 0) {
      printf("Failed to get CAN controller state: %d", err);
      k_sleep(K_MSEC(100));
      continue;
    }

    if (err_cnt.tx_err_cnt != err_cnt_prev.tx_err_cnt ||
        err_cnt.rx_err_cnt != err_cnt_prev.rx_err_cnt || state_prev != state) {

      err_cnt_prev.tx_err_cnt = err_cnt.tx_err_cnt;
      err_cnt_prev.rx_err_cnt = err_cnt.rx_err_cnt;
      state_prev = state;
      printf("state: %s\n"
             "rx error count: %d\n"
             "tx error count: %d\n",
             state_to_str(state), err_cnt.rx_err_cnt, err_cnt.tx_err_cnt);
    } else {
      k_sleep(K_MSEC(100));
    }
  }
}

void state_change_work_handler(struct k_work *work) {
  printf("State Change ISR\nstate: %s\n"
         "rx error count: %d\n"
         "tx error count: %d\n",
         state_to_str(current_state), current_err_cnt.rx_err_cnt,
         current_err_cnt.tx_err_cnt);
}

void state_change_callback(const struct device *dev, enum can_state state,
                           struct can_bus_err_cnt err_cnt, void *user_data) {
  struct k_work *work = (struct k_work *)user_data;

  ARG_UNUSED(dev);

  current_state = state;
  current_err_cnt = err_cnt;
  k_work_submit(work);
}

void can_main() {
  const struct can_filter change_led_filter = {
      .flags = 0U, .id = LED_MSG_ID, .mask = CAN_STD_ID_MASK};
  struct can_frame change_led_frame = {.flags = 0, .id = LED_MSG_ID, .dlc = 1};
  struct can_frame counter_frame = {
      .flags = CAN_FRAME_IDE, .id = COUNTER_MSG_ID, .dlc = 2};
  uint8_t toggle = 1;
  uint16_t counter = 0;
  k_tid_t rx_tid, get_state_tid;
  int ret;

  if (!device_is_ready(can_dev)) {
    printf("CAN: Device %s not ready.\n", can_dev->name);
    return 0;
  }

#ifdef CONFIG_LOOPBACK_MODE
  ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
  if (ret != 0) {
    printf("Error setting CAN mode [%d]", ret);
    return 0;
  }
#endif
  ret = can_start(can_dev);
  if (ret != 0) {
    printf("Error starting CAN controller [%d]", ret);
    return 0;
  }

  k_work_init(&state_change_work, state_change_work_handler);

  ret = can_add_rx_filter_msgq(can_dev, &change_led_msgq, &change_led_filter);
  if (ret == -ENOSPC) {
    printf("Error, no filter available!\n");
    return 0;
  }

  printf("Change LED filter ID: %d\n", ret);

  ret = k_work_poll_submit(&change_led_work, change_led_events,
                           ARRAY_SIZE(change_led_events), K_FOREVER);
  if (ret != 0) {
    printf("Failed to submit msgq polling: %d", ret);
    return 0;
  }

  rx_tid = k_thread_create(&rx_thread_data, rx_thread_stack,
                           K_THREAD_STACK_SIZEOF(rx_thread_stack), rx_thread,
                           NULL, NULL, NULL, RX_THREAD_PRIORITY, 0, K_NO_WAIT);
  if (!rx_tid) {
    printf("ERROR spawning rx thread\n");
  }

  get_state_tid = k_thread_create(&poll_state_thread_data, poll_state_stack,
                                  K_THREAD_STACK_SIZEOF(poll_state_stack),
                                  poll_state_thread, NULL, NULL, NULL,
                                  STATE_POLL_THREAD_PRIORITY, 0, K_NO_WAIT);
  if (!get_state_tid) {
    printf("ERROR spawning poll_state_thread\n");
  }

  can_set_state_change_callback(can_dev, state_change_callback,
                                &state_change_work);

  printf("Finished init.\n");

  while (1) {
    change_led_frame.data[0] = toggle++ & 0x01 ? SET_LED : RESET_LED;
    /* This sending call is none blocking. */
    // can_send(can_dev, &change_led_frame, K_FOREVER, tx_irq_callback,
    //          "LED change");
    k_sleep(SLEEP_TIME);

    UNALIGNED_PUT(sys_cpu_to_be16(counter), (uint16_t *)&counter_frame.data[0]);
    counter++;
    /* This sending call is blocking until the message is sent. */
    // can_send(can_dev, &counter_frame, K_MSEC(100), NULL, NULL);
    k_sleep(SLEEP_TIME);
  }
}

int main(void) {
  int err;

  err = clocks_start();
  if (err) {
    return 0;
  }

  can_main();

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
