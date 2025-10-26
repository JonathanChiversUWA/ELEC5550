//USB Functionality as device
#include "tinyusb.h" //general functionality
#include "class/hid/hid_device.h" //for functionality as HID device
#include "usb_device.h"
#include "freertos/task.h" //For Task Delay and pdMS TO TICKS

/* ----------------------------- USB Descriptors ---------------------------- */

//Descriptors for Mouse/Keyboard reports
const uint8_t hid_report_descriptor[] = 
{
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))
};

//string descriptor for device (from template)
const char* hid_string_descriptor[5] = 
{
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "UART HID to HOST",  // 4: HID
};

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN) //required for config descriptor
//configuration descriptor for device
static const uint8_t hid_configuration_descriptor[] = 
{
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/* -------------------------------------------------------------------------- */
/*                             Required USB Config                            */
/* -------------------------------------------------------------------------- */

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize){}

/* ---------------------------------- Main ---------------------------------- */

static bool g_device_installed = false;

static void usb_device_check_status(void *params){
    while(g_device_installed){
        vTaskDelay(10);
        if (usb_device_is_connected()){
            continue;
        }
    }
}

void usb_device_start(void)
{
    if (g_device_installed) return;

    //configuration to act as a device
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL, //not defined yet
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false, //using esp32s3 internal phy
        .configuration_descriptor = hid_configuration_descriptor
    };
    //install hid device with pointer to configuration
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    g_device_installed = true;
    // xTaskCreate(usb_device_check_status, "usb_device_check_status", 2048, NULL, 5, NULL);
}

void usb_device_stop(void)
{
    if (!g_device_installed) return;
    g_device_installed = false;
    vTaskDelay(20);
    tinyusb_driver_uninstall();
}

bool usb_device_is_connected(void)
{
    // tinyusb provides tud_mounted() to indicate device is enumerated by host
    if (g_device_installed){
        return tud_mounted();
    }
    return false;
}