/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <sys/byteorder.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_vs.h>

#include <nrfx_power.h>

#define TAG_NAME "FYP tag"
#define TAG_NAME_LEN (sizeof(TAG_NAME) - 1)
#define TAG_NUMBER  0x05

#define ADV_PERIOD_MS   1000
#define ADV_DUTY_CYCLE  0.1
#define ADV_RUN_TIME_MS    (ADV_PERIOD_MS*ADV_DUTY_CYCLE)
#define ADV_SLEEP_TIME_MS  (ADV_PERIOD_MS-ADV_RUN_TIME_MS)

#define TX_POWER 0x04 // Calibrated Tx power at 0m


/*
 * Set advertisement data (24 bits)
 */
static const struct bt_data ad[] = {
  BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL, BT_LE_AD_NO_BREDR),
  BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA, 0xFF, 0xFF, 'F', 'H'), // Company (just used my initals prefixed by 255 255)
  BT_DATA_BYTES(0x16, 'F', 'Y', 'P', TAG_NUMBER),
  BT_DATA_BYTES(BT_DATA_TX_POWER, TX_POWER),
  BT_DATA(BT_DATA_NAME_COMPLETE, TAG_NAME, TAG_NAME_LEN)
};

/**
 * @brief Set the tx power
 * 
 * @param handle_type 
 * @param handle 
 * @param tx_pwr_lvl 
 */
static void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
{
	struct bt_hci_cp_vs_write_tx_power_level *cp;
	struct bt_hci_rp_vs_write_tx_power_level *rp;
	struct net_buf *buf, *rsp = NULL;
	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				sizeof(*cp));
	if (!buf) {
		printk("Unable to allocate command buffer\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);
	cp->handle_type = handle_type;
	cp->tx_power_level = tx_pwr_lvl;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				   buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_vs_write_tx_power_level *)
			  rsp->data)->status : 0;
		printk("Set Tx power err: %d reason 0x%02x\n", err, reason);
		return;
	}

	rp = (void *)rsp->data;
	printk("Actual Tx Power: %d\n", rp->selected_tx_power);

	net_buf_unref(rsp);
}

void main(void)
{
  int err;
  nrf_power_dcdcen_set(NRF_POWER, true);

  // Make sure the DCDC converter is working
  printk("DCDC converter: %s\n", nrf_power_dcdcen_get(NRF_POWER) ? "true" : "false");

  err = bt_enable(NULL);
  if (err)
  {
    __ASSERT(err, "BT init failed, err: %d", err);
  }

  set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0, -10);

  while(true)
  {
    err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err)
    {
      __ASSERT(err, "ADV failed to start, err: %d", err);
    }

    k_sleep(K_MSEC(ADV_RUN_TIME_MS));

    err = bt_le_adv_stop();
    if (err)
    {
      __ASSERT(err, "ADV failed to stop, err: %d", err);
    }

    k_sleep(K_MSEC(ADV_SLEEP_TIME_MS));
  }

}
