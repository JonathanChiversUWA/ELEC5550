// usb_device.h - small wrapper API for TinyUSB device control and status
#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include <stdbool.h>

// Start the USB device stack (TinyUSB). Returns 0 on success, esp_err_t otherwise.
void usb_device_start(void);

// Stop/uninstall the USB device stack.
void usb_device_stop(void);

// Returns true if USB device is currently mounted/connected to a host.
bool usb_device_is_connected(void);

#endif // USB_DEVICE_H
