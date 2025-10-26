idf_component_register(SRCS "main.c" "usb_device.c" "usb_host.c" 
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES usb esp_driver_gpio esp_driver_uart esp_tinyusb
                    )