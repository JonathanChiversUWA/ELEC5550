//USB Functionality as host including HID devices
#include "usb/usb_host.h" //general host functionality
#include "usb/hid_host.h" //hid devices
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"
#include "usb_host.h"

static bool g_host_installed = false;
static bool g_host_device_connected = false;

// Internal callback from hid host driver to set device connected flag
static void hid_host_driver_cb(hid_host_device_handle_t hid_device_handle,
                               const hid_host_driver_event_t event,
                               void *arg)
{
    (void)hid_device_handle; (void)arg;
    if (event == HID_HOST_DRIVER_EVENT_CONNECTED) {
        g_host_device_connected = true;
    }
}

void usb_host_start(void)
{
    if (g_host_installed) return;

    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_driver_cb,
        .callback_arg = NULL
    };

    ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));
    g_host_installed = true;
}

void usb_host_stop(void)
{
    if (!g_host_installed) return;
    ESP_ERROR_CHECK(hid_host_uninstall());
    g_host_installed = false;
    g_host_device_connected = false;
}

bool usb_host_is_installed(void)
{
    return g_host_installed;
}

bool usb_host_device_connected(void)
{
    return g_host_device_connected;
}