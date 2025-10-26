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
#include "globals.h"
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

#define TX_PIN GPIO_NUM_43
#define RX_PIN GPIO_NUM_44
const int uart_buffer_size = (1024*2);
const uart_port_t uart_num = UART_NUM_0; //Built in UART on GPIO 43 and 44
QueueHandle_t uart_queue;

volatile bool boot_pressed = false; //if switch active (prevent spam press)
volatile bool searching_for_device = true;

/* ---------------------------- Enums and Structs --------------------------- */
volatile module_status_t module_status = STATUS_CONNECTION;
volatile conn_status_t conn_status = STATUS_NONE;

typedef enum {
    TO_HOST,
    TO_DEVICE
} dataline_t;

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
                if(len>0 && dtmp[0] == 0xAA){
                    gpio_set_level(LED_GPIO_COM,1);
                }
            }else{
                uart_flush_input(uart_num);
            }
        }else{
            gpio_set_level(LED_GPIO_COM,0);
            vTaskDelay(pdMS_TO_TICKS(10));
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
                // if (conn_status == STATUS_HOST){
                //     //pause host uart task while calibrating (don't double use UART);
                //     usb_host_uart_pause();
                // }else if (conn_status == STATUS_DEVICE){
                //     //pause device uart task while calibrating (don't double use UART);
                //     usb_device_uart_pause();
                // }
                setup_calibrating();
            }else{
                // if (conn_status == STATUS_HOST){
                //     //resume host uart task after calibration if connected
                //     usb_host_uart_start();
                // }else if (conn_status == STATUS_DEVICE){
                //     //resume device uart task after calibration if connected
                //     usb_device_uart_start();
                // }
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

static void led_con_status_task(){
    while(1){
        if(conn_status == STATUS_NONE){ //No connection, slow blink
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

static void led_cal_status_task(){
    while(1){
        if(module_status == STATUS_CALIBRATION){ //Switch LED at 5Hz if in calibration mode
            gpio_set_level(LED_GPIO_CAL,1); //Turn on Calibration LED
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(LED_GPIO_CAL,0); //Turn off Calibration LED
            vTaskDelay(pdMS_TO_TICKS(50));
        }else{
            gpio_set_level(LED_GPIO_CAL,0); //Turn on Calibration LED
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

        gpio_set_level(LINE_PIN,TO_DEVICE); //Set data line switch to look for Device (USB A)
        vTaskDelay(pdMS_TO_TICKS(5)); //Allow line to stabilise

        usb_host_start();
        TickType_t start = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(2000)){
            if(usb_host_device_connected()){
                usb_host_uart_start();
                conn_status = STATUS_HOST;
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        usb_host_stop();
        vTaskDelay(pdMS_TO_TICKS(10));

    } else {
        //act as device and try enumerate with other side (host)

        gpio_set_level(LINE_PIN,TO_HOST); //Set data line switch to look for Host (microUSB)
        vTaskDelay(pdMS_TO_TICKS(5)); //Allow line to stabilise

        usb_device_start();
        TickType_t start = xTaskGetTickCount();
        while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(2000)){
            if(usb_device_is_connected()){
                usb_device_uart_start();
                conn_status = STATUS_DEVICE;
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        usb_device_stop();
        vTaskDelay(pdMS_TO_TICKS(10));

    }
    searching_for_device = !searching_for_device;
}

/* -------------------------------------------------------------------------- */
/*                                    Main                                    */
/* -------------------------------------------------------------------------- */

void app_main(void)
{

    //Set up LEDs
    gpio_num_t leds[] = {LED_GPIO_PWR,LED_GPIO_CON,LED_GPIO_CAL,LED_GPIO_COM,LINE_PIN};
    for(int i=0;i<5;i++){
        gpio_reset_pin(leds[i]);
        gpio_set_direction(leds[i],GPIO_MODE_OUTPUT);
        gpio_set_level(leds[i],0); //LEDs begin off
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(LED_GPIO_PWR,1); //Power LED on after 200ms

    //Set up LED status tasks
    xTaskCreate(led_con_status_task, "led_con_status_task", 1024, NULL, 10, NULL);
    xTaskCreate(led_cal_status_task, "led_cal_status_task", 1024, NULL, 11, NULL);

    //Set up Calibrate task (monitors for button pressed variable constantly)
    xTaskCreate(set_calibrate, "boot_button_task", 1024, NULL, 9, NULL);
    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(CAL_PIN),
        .mode = GPIO_MODE_INPUT, //read this pin
        .pull_up_en = GPIO_PULLUP_ENABLE, //enable pull up resistor
        .pull_down_en = GPIO_PULLDOWN_DISABLE, //disable pull down resistor
        .intr_type = GPIO_INTR_NEGEDGE, //trigger interrupts on falling edge
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CAL_PIN, gpio_toggle_calibrate_callback, NULL)); 

    //Initialise UART lines
    uart_init();

    //Main loop
    while(1){
        switch (conn_status) {
            case STATUS_NONE:
                search(); //cannot be called in a FreeRTOStask, hence is a function
                break; //end case
            case STATUS_DEVICE:
                if(!usb_device_is_connected()){
                    conn_status = STATUS_NONE;
                    usb_device_stop();
                }
                break;
            case STATUS_HOST:
                if(!usb_host_device_connected()){
                    conn_status = STATUS_NONE;
                    usb_host_stop();
                }
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}