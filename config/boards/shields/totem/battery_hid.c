
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>

LOG_MODULE_REGISTER(zmk_split_battery, CONFIG_ZMK_LOG_LEVEL);

#define REPORT_ID_BATTERY 0x01
#define BATTERY_UNKNOWN   0xFF
#define PERIODIC_RESEND_SEC 30

static const uint8_t hid_report_desc[] = {
    0x06, 0x00, 0xFF,
    0x09, 0x01,
    0xA1, 0x01,
    0x85, REPORT_ID_BATTERY,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x01,
    0x09, 0x01,
    0x81, 0x02,
    0x09, 0x02,
    0x81, 0x02,
    0xC0,
};

static const struct device *hid_dev;
static K_SEM_DEFINE(report_sem, 1, 1);
static uint8_t levels[2] = { BATTERY_UNKNOWN, BATTERY_UNKNOWN };

static void int_in_ready_cb(const struct device *dev)
{
    ARG_UNUSED(dev);
    k_sem_give(&report_sem);
}

static const struct hid_ops ops = {
    .int_in_ready = int_in_ready_cb,
};

static int send_report(void)
{
    if (!hid_dev) {
        return -ENODEV;
    }
    uint8_t buf[3] = { REPORT_ID_BATTERY, levels[0], levels[1] };

    if (k_sem_take(&report_sem, K_MSEC(100)) != 0) {
        LOG_WRN("battery report semaphore busy");
        return -EBUSY;
    }
    int err = hid_int_ep_write(hid_dev, buf, sizeof(buf), NULL);
    if (err) {
        k_sem_give(&report_sem);
        LOG_ERR("hid_int_ep_write failed: %d", err);
    }
    return err;
}

static void periodic_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(periodic_work, periodic_handler);

static void periodic_handler(struct k_work *work)
{
    send_report();
    k_work_reschedule(&periodic_work, K_SECONDS(PERIODIC_RESEND_SEC));
}

static int zmk_split_battery_hid_init(void)
{
    hid_dev = device_get_binding("HID_1");
    if (!hid_dev) {
        LOG_ERR("device_get_binding(HID_1) returned NULL — "
                "is CONFIG_USB_HID_DEVICE_COUNT >= 2?");
        return -ENODEV;
    }
    usb_hid_register_device(hid_dev, hid_report_desc, sizeof(hid_report_desc), &ops);
    int err = usb_hid_init(hid_dev);
    if (err) {
        LOG_ERR("usb_hid_init(HID_1) failed: %d", err);
        return err;
    }
    k_work_schedule(&periodic_work, K_SECONDS(PERIODIC_RESEND_SEC));
    LOG_INF("split battery HID interface initialized on HID_1");
    return 0;
}

SYS_INIT(zmk_split_battery_hid_init, APPLICATION, 91);

static int battery_listener(const zmk_event_t *eh)
{
    const struct zmk_peripheral_battery_state_changed *ev =
        as_zmk_peripheral_battery_state_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (ev->source < ARRAY_SIZE(levels)) {
        levels[ev->source] = ev->state_of_charge;
        send_report();
        LOG_DBG("battery update src=%u soc=%u", ev->source, ev->state_of_charge);
    } else {
        LOG_WRN("battery event from unexpected source=%u", ev->source);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(zmk_split_battery_listener, battery_listener);
ZMK_SUBSCRIPTION(zmk_split_battery_listener, zmk_peripheral_battery_state_changed);
