#include "freertos/FreeRTOS.h"
#include "freertos/task.h" //For Task Delay and pdMS TO TICKS
#include <stdint.h> //uint8_t etc
#include "driver/uart.h" //for uart stuff, REQUIRES esp_driver_uart
#include <string.h> //For string manipulation memcpy
#include <esp_log.h> //For Serial debugging ESP_LOGI
#include "usb/usb_host.h" //for host functionality
#include "driver/gpio.h" //for configuring GPIOs

//HID Functionality for Mouse and Keyboard
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"

//Reset pin
#define APP_QUIT_PIN GPIO_NUM_0

const int uart_buffer_size = (1024*2); 
const uart_port_t uart_num = UART_NUM_2; //From datasheet (0-2)
QueueHandle_t uart_queue; //UART Queue
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

//Boot button callback function
static void gpio_isr_cb(){
    BaseType_t xTaskWoken = pdFALSE; //if there has been a higher priority task woken
    const app_event_queue_t evt_queue = {
        .event_group = APP_EVENT,
    };

    if (app_event_queue){ //if event queue exists (not already turned off)
        // queue to add event to, item to add to queue, ...
        xQueueSendFromISR(app_event_queue, &evt_queue, &xTaskWoken);
    }

    //Code to yield if a higher priority task is woken
    if (xTaskWoken == pdTRUE){
        portYIELD_FROM_ISR();
    }
}

//initialise UART
static void uart_init(void){ //static - internal function for this file only. Void - no return
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_ODD,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, //RTS and CTS can't be sent through laser, so don't use
    };
    //load configuration above
    ESP_ERROR_CHECK(uart_param_config(uart_num,&uart_config));
    //Install driver: Driver Num, TX / RX Buffer, 10 is length of queue, &uart_queue is pointer to queue (rather than new variable), 0 no flags
    ESP_ERROR_CHECK(uart_driver_install(uart_num,uart_buffer_size,uart_buffer_size,128, &uart_queue, 0)); 
    //invert the UART so laser is not on for as long
    ESP_ERROR_CHECK(uart_set_line_inverse(uart_num, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV));
    //Rx, Tx, RTS, CTS (IO number and NO CHANGE for default / unused)
    ESP_ERROR_CHECK(uart_set_pin(uart_num,37,38,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
}

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
    vTaskDelete(NULL); //delete this task (NULL = itself)
}


//Callback for reports to the host from device
static void hid_host_report_callback(const uint8_t *const data, const int length, const uint8_t proto){
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

void app_main(void){

    BaseType_t task_created;
    app_event_queue_t evt_queue;

    //Enable Boot pin to exit the app (Reset will restart)
    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT, //read this pin
        .pull_up_en = GPIO_PULLUP_ENABLE, //have this pulled up
        .intr_type = GPIO_INTR_NEGEDGE, //trigger interrupts on falling edge
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin)); //Set up the pin as above
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1)); //low priority interrupt on gpio
    //run gpio_isr_cb when interrupt triggers on APP_QUIT_PIN
    ESP_ERROR_CHECK(gpio_isr_handler_add(APP_QUIT_PIN, gpio_isr_cb, NULL));

    uart_init(); //initialise uart line for transmission


    //create usb host task:
    task_created = xTaskCreatePinnedToCore(usb_lib_task,//task defined in function
        "usb_events", //name
        4096, //depth of stack
        xTaskGetCurrentTaskHandle(), //run on current task
        2, NULL, 0); //Priority, created task?, core
    assert(task_created == pdTRUE);

    ulTaskNotifyTake(false,100); //wait 100ms max for usb_lib_task to respond.

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

    //From here on the Device is ready to be connected

    while(1){
        //xQueueReceive will check if the app_event_queue has events in it (populated by the installed HID Driver)
        //&evt_queue is a pointer to the buffer to store events for processing
        //portMax_DELAY wait the maximum time for a new event if the Queue is empty
        if(xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY)){
            if(APP_EVENT == evt_queue.event_group){ //if this is a reset request (user press button)
                usb_host_lib_info_t lib_info; //variable to store information of host
                ESP_ERROR_CHECK(usb_host_lib_info(&lib_info)); //retrieve info
                if (lib_info.num_devices == 0){
                    break;
                }else{
                    //Cannot shutdown safely if devices still connected.
                }
            }

            if(APP_EVENT_HID_HOST == evt_queue.event_group){
                hid_host_device_event(evt_queue.hid_host_device.handle,
                                      evt_queue.hid_host_device.event,
                                      evt_queue.hid_host_device.arg);
            }
        }
    }

    //This part is reached if an APP_EVENT (shutdown) is called, and there are no devices connected
    ESP_ERROR_CHECK(hid_host_uninstall());
    gpio_isr_handler_remove(APP_QUIT_PIN);

    //Housekeeping: Reset and delete queue 
    xQueueReset(app_event_queue);
    vQueueDelete(app_event_queue);
}