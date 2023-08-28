
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#define BT_UUID_IMU_SERVICE_VAL BT_UUID_128_ENCODE(0xCAD26101, 0x0197F, 0x44A8, 0x88A5, 0x7DB4D1EFFC0F)
#define BT_UUID_IMU_PREDICTIONS_VAL BT_UUID_128_ENCODE(0xCAD26102, 0x0197F, 0x44A8, 0x88A5, 0x7DB4D1EFFC0F)
#define BT_UUID_MESSAGE_VAL BT_UUID_128_ENCODE(0xCAD26103, 0x0197F, 0x44A8, 0x88A5, 0x7DB4D1EFFC0F)

#define BT_UUID_IMU_SERVICE BT_UUID_DECLARE_128(BT_UUID_IMU_SERVICE_VAL)
#define BT_UUID_IMU_PREDICTIONS BT_UUID_DECLARE_128(BT_UUID_IMU_PREDICTIONS_VAL)
#define BT_UUID_IMU_MESSAGE BT_UUID_DECLARE_128(BT_UUID_MESSAGE_VAL)

struct bt_remote_service_cb
{
    void (*notif_changed)(bool status);
    void (*data_received)(struct bt_conn *conn, const uint8_t *data, uint16_t len);
};

int bluetooth_init(struct bt_conn_cb *bt_cb, struct bt_remote_service_cb *remote_cb);
int send_gatt_notification_float(struct bt_conn *conn, float *value, uint8_t len);
int send_gatt_notification_int(struct bt_conn *conn, int *value, uint8_t len);