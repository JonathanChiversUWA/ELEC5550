//USB Functionality as host including HID devices
#include "usb/usb_host.h" //general host functionality
#include "usb/hid_host.h" //hid devices
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h> //For string manipulation memcpy
//Custom
#include "usb_host.h"
#include "globals.h"

static bool g_host_installed = false;
static bool g_host_device_connected = false;
TaskHandle_t usb_lib_task_handle = NULL;
volatile bool uart_host_task_running = false;
static TaskHandle_t uart_task_handle = NULL;

QueueHandle_t app_event_queue = NULL; //HID Host Event Queue

//Types of Events
typedef enum {
    APP_EVENT = 0, //General event like APP quit (IO0)
    APP_EVENT_HID_HOST //HID Host Driver event like connections / reports
} app_event_group_t;

//struct for HID Events
typedef struct {
    app_event_group_t event_group; //type of event as above
    /* HID Host - Device related info */
    struct {
        hid_host_device_handle_t handle;
        hid_host_driver_event_t event;
        void *arg;
    } hid_host_device; //device handler and event id to include in HID event definitions
} app_event_queue_t;
app_event_queue_t evt_queue;

// USB Host Setup
static void usb_lib_task(){
    //configuration of host
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    //install host with configuration above
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    while (1){
        uint32_t event_flags;
        //handle events repeatedly with maximum timeout
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        //free all devices if shutdown event, then break loop
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS){
            ESP_ERROR_CHECK(usb_host_device_free_all());
            break;
        }
    }

    vTaskDelay(10);
    ESP_ERROR_CHECK(usb_host_uninstall());
    xTaskNotifyGive(xTaskGetCurrentTaskHandle());
    vTaskDelete(NULL);
}

//Callback function to put a HID Device event in the queue
void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                              const hid_host_driver_event_t event,
                              void *arg){
    const app_event_queue_t evt_queue = {
        .event_group = APP_EVENT_HID_HOST,
        //Information for the HID Host
        .hid_host_device.handle = hid_device_handle,
        .hid_host_device.event = event,
        .hid_host_device.arg = arg //unused
    };

    if (app_event_queue){
        xQueueSend(app_event_queue, &evt_queue, 0);
    }
};

//Callback for reports to the host from device
static void hid_host_report_callback(const uint8_t *const data, const int length, const uint8_t proto){

    if (module_status == STATUS_CALIBRATION) return;

    uint8_t buf[length+1];
    buf[0] = proto; //mouse is 2, keyboard is 1
    memcpy(&buf[1], data, length); //Copy data to the end of buf
    uart_write_bytes(uart_num, (const char *)&buf, length+1);
}

//Callback function to run when device receives an event
void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                 const hid_host_interface_event_t event,
                                 void *arg){
    uint8_t data[64] = {0}; // initialise 64 bytes of data as 0s
    size_t data_length = 0; //initially 0 bytes of information

    //get the device parameters again
    hid_host_dev_params_t dev_params;
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    switch (event){
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT: //data sent
        //Extract data
        ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(hid_device_handle,data,64,&data_length));
        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class){ //If a boot device (mouse/keyboard)
            //If it is a keyboard or mouse
            if ( (HID_PROTOCOL_KEYBOARD == dev_params.proto) | (HID_PROTOCOL_MOUSE == dev_params.proto) ){ 
                hid_host_report_callback(data,data_length,dev_params.proto); //Run callback data
            }
        } //do nothing if not keyboard or mouse
        break;
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED: //close device when it's disconnected
        ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
        break;
    default:
        break;
    }

}

//For when a device is connected to the host(This)
static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void *arg){
    hid_host_dev_params_t dev_params; //parameters for device
    //get parameters for device with given handle, and put into dev_params
    ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

    switch(event){
    case HID_HOST_DRIVER_EVENT_CONNECTED: //if the event is a device connected
        const hid_host_device_config_t dev_config = {
            .callback = hid_host_interface_callback, //define the callback function for when the device receives an event
            .callback_arg = NULL
        };
        ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config)); //Open the device with the config described.
        if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class){ //BOOT is a subclass that simplifies communication
            if (HID_PROTOCOL_KEYBOARD == dev_params.proto){ //keyboard should not spam empty events, so set idle to 0
                ESP_ERROR_CHECK(hid_class_request_set_idle(hid_device_handle,0,0));
                ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_BOOT)); //set to BOOT for keyboards so doesn't spam empty keys
            }
        }
        ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));
        break;

    default: //Ignore if not
        break;
    }
}

void uart_event_task(){
    uart_host_task_running = true;

    while(uart_host_task_running){
        if(xQueueReceive(app_event_queue, (void *)&evt_queue, pdMS_TO_TICKS(100))){
            if(evt_queue.event_group == APP_EVENT_HID_HOST){
                gpio_set_level(LED_GPIO_COM,1); //Turn on COM LED
                hid_host_device_event(evt_queue.hid_host_device.handle,
                                      evt_queue.hid_host_device.event,
                                      evt_queue.hid_host_device.arg);
            }
        }else{
            gpio_set_level(LED_GPIO_COM,0); //Turn off COM LED
            usb_host_lib_info_t lib_info; //variable to store information of host
            ESP_ERROR_CHECK(usb_host_lib_info(&lib_info)); //retrieve info
            if (lib_info.num_devices == 0){
                break;
            }else{
                //Cannot shutdown safely if devices still connected.
            }
        }
    }
    gpio_set_level(LED_GPIO_COM,0); //Turn off COM LED
    uart_task_handle = NULL;
    vTaskDelete(NULL);
}

void usb_host_start(void)
{
    if (g_host_installed) return;

    //create usb host task:
    BaseType_t task_created = xTaskCreatePinnedToCore(usb_lib_task, "usb_events", 4096, xTaskGetCurrentTaskHandle(), 2, &usb_lib_task_handle, 0);
    assert(task_created == pdTRUE);

    //wait 100ms max for usb_lib_task to respond - important
    ulTaskNotifyTake(false, 100);

    //Configure HID Driver
    const hid_host_driver_config_t hid_host_driver_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_device_callback, //function that puts HID Device event in queue
        .callback_arg = NULL
    };

    //Install HID Driver
    ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

    //Create Queue for holding HID events
    app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));
    g_host_installed = true;
}

void usb_host_uart_start(void)
{
    if (uart_task_handle != NULL || module_status == STATUS_CALIBRATION) return;
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 5, &uart_task_handle);
}

// void usb_host_uart_pause(void)
// {
//     if (uart_task_handle == NULL) return;
    
//     uart_host_task_running = false;
//     while (uart_task_handle != NULL) {vTaskDelay(pdMS_TO_TICKS(10));}
// }

void usb_host_stop(void)
{
    if (!g_host_installed) return;
    ESP_ERROR_CHECK(hid_host_uninstall());
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)); //wait for usb task to end
    xQueueReset(app_event_queue);
    vQueueDelete(app_event_queue);
    g_host_installed = false;
    g_host_device_connected = false;
}

bool usb_host_is_installed(void)
{
    return g_host_installed;
}

bool usb_host_device_connected(void)
{
    if (g_host_installed){
        usb_host_lib_info_t lib_info;
        ESP_ERROR_CHECK(usb_host_lib_info(&lib_info));
        return (lib_info.num_devices > 0);
    }
    return false;
}