#include <assert.h>
#include <errno.h>
#include <soc.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/hids.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/dis.h>
 
LOG_MODULE_REGISTER(gamepad, LOG_LEVEL_DBG);

#define REPORT_ID 3
#define REPORT_ID_IDX 0
#define REPORT_LEN 10
#define HIDS_QUEUE_SIZE 10
#define DISCONNECTED_SLEEP_TIMEOUT K_SECONDS(60)
#define CONNECTED_SLEEP_TIMEOUT K_SECONDS(600)
#define SYS_BUTTON_LONG_PRESS_MS 3000
#define SYS_BUTTON_VERY_LONG_PRESS_MS 10000
#define CHK(x) do { int ret = (x); if (ret) { printk("Error %d at %s:%d\n", ret, __FILE__, __LINE__); return; } } while(0)

#define DPAD_ADC_RESOLUTION 12
#define DPAD_ADC_GAIN ADC_GAIN_1
#define DPAD_ADC_REF ADC_REF_INTERNAL
#define DPAD_ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT
#define DPAD_CENTER 2048
#define DPAD_DEADZONE 300
 
struct __attribute__((packed)) report_t {
    uint8_t dpad;
    uint8_t capture : 1;
    uint8_t assistant : 1;
    uint8_t l2 : 1;
    uint8_t r2 : 1;
    uint8_t stadia : 1;
    uint8_t menu : 1;
    uint8_t options : 1;
    uint8_t r3 : 1;
    uint8_t l3 : 1;
    uint8_t r1 : 1;
    uint8_t l1 : 1;
    uint8_t y : 1;
    uint8_t x : 1;
    uint8_t b : 1;
    uint8_t a : 1;
    uint8_t padding : 1;
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
    uint8_t l2_axis;
    uint8_t r2_axis;
    uint8_t extra_buttons;
};

static struct report_t report;
static struct report_t prev_report;

// Full HID report descriptor, as in your original file
static uint8_t const report_map[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x25, 0x07,        //   Logical Maximum (7)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x65, 0x14,        //   Unit (System: English Rotation, Length: Centimeter)
    0x09, 0x39,        //   Usage (Hat switch)
    0x81, 0x42,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
    0x45, 0x00,        //   Physical Maximum (0)
    0x65, 0x00,        //   Unit (None)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x09,        //   Usage Page (Button)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x0F,        //   Report Count (15)
    0x09, 0x12,        //   Usage (0x12)
    0x09, 0x11,        //   Usage (0x11)
    0x09, 0x14,        //   Usage (0x14)
    0x09, 0x13,        //   Usage (0x13)
    0x09, 0x0D,        //   Usage (0x0D)
    0x09, 0x0C,        //   Usage (0x0C)
    0x09, 0x0B,        //   Usage (0x0B)
    0x09, 0x0F,        //   Usage (0x0F)
    0x09, 0x0E,        //   Usage (0x0E)
    0x09, 0x08,        //   Usage (0x08)
    0x09, 0x07,        //   Usage (0x07)
    0x09, 0x05,        //   Usage (0x05)
    0x09, 0x04,        //   Usage (0x04)
    0x09, 0x02,        //   Usage (0x02)
    0x09, 0x01,        //   Usage (0x01)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x15, 0x01,        //   Logical Minimum (1)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x09, 0x32,        //     Usage (Z)
    0x09, 0x35,        //     Usage (Rz)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0x05, 0x02,        //   Usage Page (Sim Ctrls)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x02,        //   Report Count (2)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x09, 0xC5,        //   Usage (Brake)
    0x09, 0xC4,        //   Usage (Accelerator)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x0C,        //   Usage Page (Consumer)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x09, 0xE9,        //   Usage (Volume Increment)
    0x09, 0xEA,        //   Usage (Volume Decrement)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0xCD,        //   Usage (Play/Pause)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x05,        //   Report Count (5)
    0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,  // End Collection
};

static const struct gpio_dt_spec buttons[] = {
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_start), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_select), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_y), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_x), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_b), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_a), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_zr), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_zl), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_rb), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_lb), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_l3), gpios),
    GPIO_DT_SPEC_GET(DT_ALIAS(btn_r3), gpios),
};

BT_HIDS_DEF(hids_obj, REPORT_LEN);
K_MSGQ_DEFINE(hids_queue, REPORT_LEN, HIDS_QUEUE_SIZE, 4);

static struct bt_conn* active_conn = NULL;
static bool try_directed;
static int prev_sys_button_state = 0;
static int64_t sys_button_pressed_at;

static void sleep_work_fn(struct k_work* work) {
    LOG_INF("Going to sleep...");
    sys_poweroff();
}
static K_WORK_DELAYABLE_DEFINE(sleep_work, sleep_work_fn);

static void advertising_work_fn(struct k_work* work) {
    LOG_INF("");
    struct bt_le_adv_param adv_param;
    adv_param = *BT_LE_ADV_CONN;
    adv_param.options |= BT_LE_ADV_OPT_ONE_TIME;
    if (!bt_le_adv_start(&adv_param, NULL, 0, NULL, 0)) {
        LOG_INF("Regular advertising started.");
    }
}
static K_WORK_DEFINE(advertising_work, advertising_work_fn);

static void advertising_start(void) {
    try_directed = false;
    k_work_submit(&advertising_work);
    k_work_reschedule(&sleep_work, DISCONNECTED_SLEEP_TIMEOUT);
}

static void advertising_restart(void) {
    try_directed = false;
    k_work_submit(&advertising_work);
}

static void connected(struct bt_conn* conn, uint8_t err) {
    if (err) return;
    active_conn = conn;
    CHK(bt_hids_connected(&hids_obj, conn));
    k_work_reschedule(&sleep_work, CONNECTED_SLEEP_TIMEOUT);
}

static void disconnected(struct bt_conn* conn, uint8_t reason) {
    if (conn == active_conn) {
        bt_hids_disconnected(&hids_obj, conn);
        active_conn = NULL;
    }
    advertising_start();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void hid_init(void) {
    struct bt_hids_init_param hids_init_param = { 0 };
    struct bt_hids_inp_rep* hids_inp_rep;
    hids_init_param.rep_map.data = report_map;
    hids_init_param.rep_map.size = sizeof(report_map);

    hids_init_param.info.bcd_hid = 0x0101;
    hids_init_param.info.b_country_code = 0x00;
    hids_init_param.info.flags = (BT_HIDS_REMOTE_WAKE |
                                  BT_HIDS_NORMALLY_CONNECTABLE);

    hids_inp_rep = &hids_init_param.inp_rep_group_init.reports[0];
    hids_inp_rep->size = sizeof(struct report_t);
    hids_inp_rep->id = REPORT_ID;
    hids_init_param.inp_rep_group_init.cnt++;

    bt_hids_init(&hids_obj, &hids_init_param);
}

static void report_sent_cb(struct bt_conn* conn, void* user_data) {
    LOG_DBG("");
}

static void hids_work_fn(struct k_work* work) {
    uint8_t rep[REPORT_LEN];
    while (!k_msgq_get(&hids_queue, rep, K_NO_WAIT)) {
        if (active_conn != NULL) {
            k_work_reschedule(&sleep_work, CONNECTED_SLEEP_TIMEOUT);
            bt_hids_inp_rep_send(&hids_obj, active_conn, REPORT_ID_IDX, rep, REPORT_LEN, report_sent_cb);
        }
    }
}
static K_WORK_DEFINE(hids_work, hids_work_fn);

static void reset_to_bootloader() {
    sys_reboot(0x57);
}

static void report_init() {
    memset(&report, 0, sizeof(report));
    report.dpad = 8;
    report.lx = 0x80;
    report.ly = 0x80;
    report.rx = 0x80;
    report.ry = 0x80;
    memcpy(&prev_report, &report, sizeof(report));
}

static void configure_buttons(void) {
    for (size_t i = 0; i < ARRAY_SIZE(buttons); i++) {
        gpio_pin_configure_dt(&buttons[i], GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
    }
}

static int read_adc(const struct device *adc_dev, uint8_t channel) {
    int16_t buf = 0;
    struct adc_channel_cfg cfg = {
        .gain = DPAD_ADC_GAIN,
        .reference = DPAD_ADC_REF,
        .acquisition_time = DPAD_ADC_ACQUISITION_TIME,
        .channel_id = channel,
        .differential = 0,
    };
    adc_channel_setup(adc_dev, &cfg);
    struct adc_sequence sequence = {
        .channels    = BIT(channel),
        .buffer      = &buf,
        .buffer_size = sizeof(buf),
        .resolution  = DPAD_ADC_RESOLUTION,
    };
    if (adc_read(adc_dev, &sequence) != 0) return DPAD_CENTER;
    return buf;
}

static void handle_buttons() {
    int sys_button_state = gpio_pin_get_dt(&buttons[0]);
    int64_t now = k_uptime_get();
    if (!prev_sys_button_state && sys_button_state) {
        sys_button_pressed_at = now;
    }
    if (prev_sys_button_state && !sys_button_state) {
        int64_t duration = now - sys_button_pressed_at;
        if (duration >= SYS_BUTTON_VERY_LONG_PRESS_MS) {
            reset_to_bootloader();
        }
    }
    prev_sys_button_state = sys_button_state;

    report.menu = gpio_pin_get_dt(&buttons[0]);
    report.options = gpio_pin_get_dt(&buttons[1]);
    report.y = gpio_pin_get_dt(&buttons[2]);
    report.x = gpio_pin_get_dt(&buttons[3]);
    report.b = gpio_pin_get_dt(&buttons[4]);
    report.a = gpio_pin_get_dt(&buttons[5]);
    report.r2 = gpio_pin_get_dt(&buttons[6]);
    report.l2 = gpio_pin_get_dt(&buttons[7]);
    report.r1 = gpio_pin_get_dt(&buttons[8]);
    report.l1 = gpio_pin_get_dt(&buttons[9]);
    report.l3 = gpio_pin_get_dt(&buttons[10]);
    report.r3 = gpio_pin_get_dt(&buttons[11]);

    // Analog joystick handling
    const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc));
    int lx = read_adc(adc_dev, 0); // LEFT_X (AIN0: P0.02)
    int ly = read_adc(adc_dev, 7); // LEFT_Y (AIN7: P1.15)
    int rx = read_adc(adc_dev, 3); // RIGHT_X (AIN3: P0.31)
    int ry = read_adc(adc_dev, 5); // RIGHT_Y (AIN5: P0.29)
    int dpad_x = read_adc(adc_dev, 2); // DPAD_X (AIN2: P1.04)
    int dpad_y = read_adc(adc_dev, 6); // DPAD_Y (AIN6: P1.06)

    // Map analog values to 8-bit
    report.lx = (lx >> 4) & 0xFF;
    report.ly = (ly >> 4) & 0xFF;
    report.rx = (rx >> 4) & 0xFF;
    report.ry = (ry >> 4) & 0xFF;

    // DPAD analog to hat value
    uint8_t dpad = 8; // default neutral

    if (dpad_y > DPAD_CENTER + DPAD_DEADZONE && abs(dpad_x - DPAD_CENTER) < DPAD_DEADZONE)
        dpad = 0; // Up
    else if (dpad_y < DPAD_CENTER - DPAD_DEADZONE && abs(dpad_x - DPAD_CENTER) < DPAD_DEADZONE)
        dpad = 4; // Down
    else if (dpad_x < DPAD_CENTER - DPAD_DEADZONE && abs(dpad_y - DPAD_CENTER) < DPAD_DEADZONE)
        dpad = 6; // Left
    else if (dpad_x > DPAD_CENTER + DPAD_DEADZONE && abs(dpad_y - DPAD_CENTER) < DPAD_DEADZONE)
        dpad = 2; // Right;
    else if (dpad_x > DPAD_CENTER + DPAD_DEADZONE && dpad_y > DPAD_CENTER + DPAD_DEADZONE)
        dpad = 1; // Up-Right
    else if (dpad_x < DPAD_CENTER - DPAD_DEADZONE && dpad_y > DPAD_CENTER + DPAD_DEADZONE)
        dpad = 7; // Up-Left
    else if (dpad_x > DPAD_CENTER + DPAD_DEADZONE && dpad_y < DPAD_CENTER - DPAD_DEADZONE)
        dpad = 3; // Down-Right
    else if (dpad_x < DPAD_CENTER - DPAD_DEADZONE && dpad_y < DPAD_CENTER - DPAD_DEADZONE)
        dpad = 5; // Down-Left

    report.dpad = dpad;

    if (memcmp(&prev_report, &report, sizeof(report))) {
        k_msgq_put(&hids_queue, &report, K_NO_WAIT);
        k_work_submit(&hids_work);
        memcpy(&prev_report, &report, sizeof(report));
    }
}

int main() {
    LOG_INF("Gamepad nRF52");

    report_init();
    hid_init();

    bt_enable(NULL);
    settings_load();
    advertising_start();
    configure_buttons();

    while (1) {
        k_sleep(K_MSEC(1));
        handle_buttons();
    }
}
