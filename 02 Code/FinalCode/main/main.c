#include <stdio.h>
#include <stdint.h> //uint8_t etc
#include <string.h> //For string manipulation e.g. strlen

//General ESP Functinonality
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" //For Task Delay and pdMS TO TICKS
#include "driver/uart.h" //for uart stuff, REQUIRES esp_driver_uart
#include <esp_log.h> //For Serial debugging ESP_LOGI
#include "driver/gpio.h" //for configuring GPIOs

//Separated functionality for host/device for organisation and due to include/define clashses
#include "usb_device.h"
#include "usb_host.h"

/* -------------------------------------------------------------------------- */
/*                      Global Variables and Definitions                      */
/* -------------------------------------------------------------------------- */

#define LED_GPIO_PWR GPIO_NUM_41 //LED to show Power
#define LED_GPIO_CON GPIO_NUM_40 //LED to show Device/Host Connected
#define LED_GPIO_CAL GPIO_NUM_39 //LED to show Calibration Mode
#define LED_GPIO_COM GPIO_NUM_38 //LED to show receiver/transmitter is high
#define CAL_PIN GPIO_NUM_0 //Pin for Calibration button

#define LINE_PIN GPIO_NUM_33 //Pin to switch data line (low = microUSB, high = USB A)

#define UART_TASK_STACK_SZ 8192
#define BUF_SIZE 128
#define TX_PIN GPIO_NUM_43
#define RX_PIN GPIO_NUM_44
const int uart_buffer_size = (1024*2);
const uart_port_t uart_num = UART_NUM_0; //Built in UART on GPIO 43 and 44
QueueHandle_t uart_queue; //UART Queue
QueueHandle_t app_event_queue = NULL; //HID Host Event Queue

volatile bool boot_pressed = false; //if switch active (prevent spam press)
volatile bool searching_for_device = true;

/* ---------------------------- Enums and Structs --------------------------- */

//Stores current status of the module
typedef enum {
    STATUS_CALIBRATION,
    STATUS_CONNECTION
} module_status_t;
volatile module_status_t module_status = STATUS_CONNECTION;

//Stores current status of connection
typedef enum {
    STATUS_NONE,
    STATUS_HOST,
    STATUS_DEVICE
} conn_status_t;
volatile conn_status_t conn_status = STATUS_NONE;

typedef enum {
    TO_HOST,
    TO_DEVICE
} dataline_t;

/* -------------------------------------------------------------------------- */
/*                                 Functions??                                */
/* -------------------------------------------------------------------------- */

// //initialise acting as host
// static void host_init(void){

//     app_event_queue_t evt_queue;

//     //Create the Host task
//     BaseType_t task_created = xTaskCreatePinnedToCore(usb_lib_task,//task defined in function
//         "usb_events", //name
//         4096, //depth of stack
//         xTaskGetCurrentTaskHandle(), //run on current task
//         2, NULL, 0); //Priority, created task?, core
//     assert(task_created == pdTRUE);

//     ulTaskNotifyTake(false,100); //wait 100ms max for usb_lib_task to respond.


//     //Install the HID driver to enable the Host Library to work with HID communication
//     //Background task is set to true, which monitors usb_host_install for new events
//     //These events are passed to hid_host_device_callback
//     const hid_host_driver_config_t hid_host_driver_config = {
//         .create_background_task = true,
//         .task_priority = 5,
//         .stack_size = 4096,
//         .core_id = 0,
//         .callback = hid_host_device_callback, //function that puts HID Device event in queue
//         .callback_arg = NULL
//     };
//     //Install HID Driver
//     ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

//     //Create Queue for holding HID events
//     app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));

//     while(1){
//         //xQueueReceive will check if the app_event_queue has events in it (populated by the installed HID Driver)
//         //&evt_queue is a pointer to the buffer to store events for processing
//         //portMax_DELAY wait the maximum time for a new event if the Queue is empty
//         if(xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY)){
//             if(APP_EVENT_HID_HOST == evt_queue.event_group){
//                 hid_host_device_event(evt_queue.hid_host_device.handle,
//                                         evt_queue.hid_host_device.event,
//                                         evt_queue.hid_host_device.arg);
//             }
//         }
//     }
//     ESP_ERROR_CHECK(hid_host_uninstall());

//     xQueueReset(app_event_queue);
//     vQueueDelete(app_event_queue);
// }


// // Set up Host task
// static void usb_lib_task(){

//     //Install the Host Library to enable acting as a host
//     const usb_host_config_t host_config = {
//         .skip_phy_setup = false,
//         .intr_flags = ESP_INTR_FLAG_LEVEL1,
//     };
//     ESP_ERROR_CHECK(usb_host_install(&host_config));

//     while (1){
//         uint32_t event_flags;
//         //handle events repeatedly with maximum timeout
//         usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

//         //free all devices if status changed, or shutdown event, then break loop
//         if (status != STATUS_HOST || (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)){
//             ESP_ERROR_CHECK(usb_host_device_free_all());
//             break;
//         }
//     }

//     //return to searching if closed while still in Host status.
//     if (status == STATUS_HOST){ status = STATUS_SEARCHING; } 

//     //Teardown self
//     vTaskDelay(10);
//     ESP_ERROR_CHECK(usb_host_uninstall());
//     ESP_ERROR_CHECK(hid_host_uninstall());
//     vTaskDelete(NULL); //NULL = itself
// }


//Callback function to put a HID Device event in the queue
// void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
//                               const hid_host_driver_event_t event,
//                               void *arg){
//     const app_event_queue_t evt_queue = {
//         .event_group = APP_EVENT_HID_HOST,
//         //Information for the HID Host
//         .hid_host_device.handle = hid_device_handle,
//         .hid_host_device.event = event,
//         .hid_host_device.arg = arg //unused
//     };

//     if (app_event_queue){
//         xQueueSend(app_event_queue, &evt_queue, 0);
//     }
// };

/* -------------------------------------------------------------------------- */
/*                               Status Control                               */
/* -------------------------------------------------------------------------- */

/* ------------------------------- Calibration ------------------------------ */

//Task to send calibration bytes
static void calibrating_tx()
{
    while(module_status == STATUS_CALIBRATION){
        //write uart test every 50ms
        uint8_t cal_byte = 0xAA;
        uart_write_bytes(uart_num,(const char *)&cal_byte, 1); //send calibration byte)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}
//Task to monitor calibration bytes
static void calibrating_rx()
{
    uart_event_t event; //holds UART event
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE); //allocate memory for temp buffer
    
    gpio_set_level(LED_GPIO_COM,0);
    while(module_status == STATUS_CALIBRATION){
        //Wait 60ms for test data to come through otherwise turn receive off.
        if(xQueueReceive(uart_queue, (void *)&event, pdMS_TO_TICKS(60))){
            bzero(dtmp,BUF_SIZE); //Clear Buffer
            if(event.type == UART_DATA){
                uint8_t dtmp[1];
                int len = uart_read_bytes(uart_num,dtmp,event.size,portMAX_DELAY);
                //turn receive LED on if received calibration byte
                if(len>0 && dtmp[0] == 0xAA){ gpio_set_level(LED_GPIO_COM,1); }
            }else{
                uart_flush_input(uart_num);
            }
        }else{
            gpio_set_level(LED_GPIO_COM,0);
        }
        
    }
    gpio_set_level(LED_GPIO_COM,0);
    vTaskDelete(NULL);
}
void setup_calibrating()
{
    module_status = STATUS_CALIBRATION;
    xTaskCreate(calibrating_tx, "calibrating_tx_task", 2048, NULL, 5, NULL);
    xTaskCreate(calibrating_rx, "calibrating_rx_task", 2048, NULL, 5, NULL);
}

/* ------------------------------- Connection ------------------------------- */

/**
 * @brief Set up connection based on current status
 * 
 * If no connection, start searching task
 * If connected as device or host, restart the task (already setup)
 */
void setup_connection()
{
    switch (conn_status) {
        case STATUS_NONE:
            // xTaskCreate(searching, "searching_task", 2048, NULL, 5, NULL); //start task to search for connection
            break; //end case
        case STATUS_DEVICE:
            return;
        case STATUS_HOST:
            return;
    }
}

/* ---------------------------- Calibrate Button ---------------------------- */

/**
 * @brief Task to monitor for calibration button press and toggle calibration mode
 * 
 * Wait's 100 microseconds after toggling status to allow any relevant tasks to close
 * Calls the relevant setup function based on new mode.
 * Wait's 300 microseconds before allowing it to be called again (prevent button spam)
 */
static void set_calibrate()
{
    while(1){
        if(boot_pressed){
            module_status ^= 1; //Toggle between calibration and connection mode
            vTaskDelay(pdMS_TO_TICKS(100));
            if(module_status == STATUS_CALIBRATION){
                setup_calibrating();
            }else{
                setup_connection(); //UPDATE TO CHECK IF ALR CONNECTED
            }
            vTaskDelay(pdMS_TO_TICKS(300));
            boot_pressed=false;
        }else{
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

/**
 * @brief Callback function to run when calibration button is pressed
 * 
 * Sets flag to indicate button pressed
 */
static void gpio_toggle_calibrate_callback()
{
    boot_pressed = true;
}

static void led_status_task(){
    while(1){
        if(module_status == STATUS_CALIBRATION){ //Switch LED at 5Hz if in calibration mode
            gpio_set_level(LED_GPIO_CAL,1); //Turn on Calibration LED
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(LED_GPIO_CAL,0); //Turn off Calibration LED
            vTaskDelay(pdMS_TO_TICKS(50));
        }else if(conn_status == STATUS_NONE){ //No connection, slow blink
            gpio_set_level(LED_GPIO_CON,1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED_GPIO_CON,0);
            vTaskDelay(pdMS_TO_TICKS(200));
        }else{ //Connected, solid on
            gpio_set_level(LED_GPIO_CON,1);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                                    UART                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialise UART
 * 
 * Called during startup to enable UART for entire operation
 */
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 38400,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_ODD,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE //No RTS and CTS
    };
    //Configure UART parameters
    uart_param_config(uart_num, &uart_config);
    //Install UART driver, and get the queue.
    uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 128, &uart_queue, 0);
    //invert the UART so laser is not on for as long
    ESP_ERROR_CHECK(uart_set_line_inverse(uart_num, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV));
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(uart_num, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
};

/* -------------------------------------------------------------------------- */
/*                                  Functions                                 */
/* -------------------------------------------------------------------------- */

static void search(){
    if(searching_for_device){ 
        //act as host and try connect to a device

        // gpio_set_level(LINE_PIN,TO_DEVICE); //Set data line switch to look for Host (microUSB)
        vTaskDelay(pdMS_TO_TICKS(100)); //Allow line to stabilise

        // usb_host_start();
        vTaskDelay(pdMS_TO_TICKS(80)); //give time for device to connect

        // if(usb_host_device_connected()){
        //     //setup host
        //     conn_status = STATUS_HOST;
        // }else{
        //     //teardown host and keep looking
        //     usb_host_stop();
        // }

    } else { 
        //act as device and try enumerate with other side (host)

        gpio_set_level(LED_GPIO_COM,1);
        gpio_set_level(LINE_PIN,TO_HOST); //Set data line switch to look for Host (microUSB)
        vTaskDelay(pdMS_TO_TICKS(100)); //Allow line to stabilise

        usb_device_start();
        vTaskDelay(pdMS_TO_TICKS(10000)); //give time for device to connect
        
        if(usb_device_is_connected()){
            //setup device
            conn_status = STATUS_DEVICE;
            gpio_set_level(LED_GPIO_CAL,1);
        }else{
            //teardown device and keep looking
            gpio_set_level(LED_GPIO_COM,0);
            usb_device_stop();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

    }
    searching_for_device = !searching_for_device;
}

/* -------------------------------------------------------------------------- */
/*                                    Main                                    */
/* -------------------------------------------------------------------------- */

void app_main(void)
{

    //Set up LEDs
    gpio_num_t leds[] = {LED_GPIO_PWR,LED_GPIO_CON,LED_GPIO_CAL,LED_GPIO_COM};
    for(int i=0;i<4;i++){
        gpio_reset_pin(leds[i]);
        gpio_set_direction(leds[i],GPIO_MODE_OUTPUT);
    }
    gpio_set_level(LED_GPIO_PWR,0); //Power LED off
    gpio_set_level(LED_GPIO_CON,0); //Connection LED on
    gpio_set_level(LED_GPIO_CAL,0); //Calibratino LED on
    gpio_set_level(LED_GPIO_COM,0); //Communication LED on
    vTaskDelay(pdMS_TO_TICKS(300));
    gpio_set_level(LED_GPIO_PWR,1); //Power LED on

    //Set up Calibration button
    xTaskCreate(set_calibrate, "boot_button_task", 1024, NULL, 5, NULL);
    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(CAL_PIN),
        .mode = GPIO_MODE_INPUT, //read this pin
        .pull_up_en = GPIO_PULLUP_ENABLE, //enable pull up resistor
        .pull_down_en = GPIO_PULLDOWN_DISABLE, //disable pull down resistor
        .intr_type = GPIO_INTR_NEGEDGE, //trigger interrupts on falling edge
    };
    //Set up the pin as above
    ESP_ERROR_CHECK(gpio_config(&input_pin));
    //low priority interrupt on gpio
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    //run gpio_toggle_calibrate when interrupt triggers on CAL_PIN
    ESP_ERROR_CHECK(gpio_isr_handler_add(CAL_PIN, gpio_toggle_calibrate_callback, NULL)); 

    //Initialise UART lines
    uart_init();

    xTaskCreate(led_status_task, "led_status_task", 1024, NULL, 6, NULL);

    while(1){
        if(module_status == STATUS_CALIBRATION){
            continue;
        }else if(conn_status == STATUS_NONE){ //No connection, slow blink
            search(); //cannot be called in a FreeRTOStask, hence is a function
        }else{ //Connected, solid on
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}