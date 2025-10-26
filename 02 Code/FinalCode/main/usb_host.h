// usb_host.h - include api for usb host functionality
#ifndef USB_HOST_H
#define USB_HOST_H

#include <stdbool.h>

// Start HID host stack. Installs HID host driver and starts background task.
void usb_host_start(void);

// Stop HID host stack and uninstall drivers.
void usb_host_stop(void);

// Returns true if host stack is currently installed/running.
bool usb_host_is_installed(void);

// Returns true if the HID host has detected a connected HID device.
bool usb_host_device_connected(void);

#endif // USB_HOST_H
