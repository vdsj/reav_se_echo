/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/att.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <bluetooth/services/hrs.h>

#include <device.h>
#include <drivers/uart.h>
#include <string.h>

// -------- BLE ---------

#define BLE_UART_SERVICE_TX_CHAR_OFFSET    3

typedef void (* ble_uart_service_rx_callback) (const uint8_t *data, size_t length);


static ble_uart_service_rx_callback rx_callback = NULL;

static struct bt_uuid_128 ble_uart_svc_uuid = BT_UUID_INIT_128(
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0x00, 0x89, 0x67, 0x43, 0x21);

static struct bt_uuid_128 ble_uart_rx_uuid = BT_UUID_INIT_128(
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0x01, 0x89, 0x67, 0x43, 0x21);

static struct bt_uuid_128 ble_uart_tx_uuid = BT_UUID_INIT_128(
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0x02, 0x89, 0x67, 0x43, 0x21);

static ssize_t ble_uart_rx_from_host (struct bt_conn *conn,
					 const struct bt_gatt_attr *attr,
					 const void *buf, uint16_t len,
					 uint16_t offset, uint8_t flags) {
	(void)conn;
	(void)attr;
	(void)offset;
	(void)flags;

    if(rx_callback) {
        rx_callback((const uint8_t *)buf,len);
    }

    return len;
}

static void ble_uart_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	(void)attr;
	(void)value;
}

static struct bt_gatt_attr ble_uart_attr_table[] = {
	BT_GATT_PRIMARY_SERVICE(&ble_uart_svc_uuid),
	BT_GATT_CHARACTERISTIC(&ble_uart_rx_uuid.uuid, BT_GATT_CHRC_WRITE_WITHOUT_RESP ,
			BT_GATT_PERM_WRITE, NULL, ble_uart_rx_from_host, NULL),
	BT_GATT_CHARACTERISTIC(&ble_uart_tx_uuid.uuid, BT_GATT_CHRC_NOTIFY,
			BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(ble_uart_ccc_changed, BT_GATT_PERM_WRITE),
};



//static struct bt_gatt_service ble_uart_service = BT_GATT_SERVICE(ble_uart_attr_table);


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL))
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};


static void echo_notify(char *buf)
{
	uint8_t heartrate = 90U;
	// printk("\n heartrate %d", heartrate);

	// int rc;
	// static uint8_t hrm[2];

	// char buffer = "0";
	// int len = sizeof(buffer);

    // //struct bt_conn *conn = ble_get_connection_ref();
	// return ( bt_gatt_notify(NULL,
	// 					&ble_uart_attr_table[BLE_UART_SERVICE_TX_CHAR_OFFSET],
	// 					buf,
	// 					len));

	// return rc == -ENOTCONN ? 0 : rc;

	//printk('%c', &buf);
	//printk('%c', *buf);

	bt_hrs_notify(*buf);
}


// -------- Uart ---------


/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) {

		uart_fifo_read(uart_dev, &c, 1);

		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

// struct bt_conn *ble_get_connection_ref() {
// 	return ble_connection;
// }

//

void main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	bt_conn_auth_cb_register(&auth_cb_display);



	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return;
	}

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(uart_dev);

	print_uart("Hello! I'm your simple echo bot.\r\n");
	print_uart("Tell me something and press enter and I send a alert to central:\r\n");


	/* indefinitely wait for input from the user */
	while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
		print_uart("Echo: ");
		print_uart(tx_buf);
		echo_notify(tx_buf);
		print_uart("\r\n");
	}

}
