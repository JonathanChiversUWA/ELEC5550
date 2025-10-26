#ifndef GLOBALS_H
#define GLOBALS_H

#include "driver/uart.h"
#include "driver/gpio.h"

/* ---------------------------- LED Definitions ---------------------------- */
#define LED_GPIO_PWR GPIO_NUM_41 //LED to show Power
#define LED_GPIO_CON GPIO_NUM_40 //LED to show Device/Host Connected
#define LED_GPIO_CAL GPIO_NUM_39 //LED to show Calibration Mode
#define LED_GPIO_COM GPIO_NUM_38 //LED to show receiver/transmitter is high

/* ---------------------------- UART Globals ----------------------------- */
#define BUF_SIZE 128
extern const int uart_buffer_size;
extern const uart_port_t uart_num;
extern QueueHandle_t uart_queue; //UART Queue

/* --------------------------------- Status --------------------------------- */
//Stores current status of the module
typedef enum {
    STATUS_CALIBRATION,
    STATUS_CONNECTION
} module_status_t;
extern volatile module_status_t module_status;

//Stores current status of connection
typedef enum {
    STATUS_NONE,
    STATUS_HOST,
    STATUS_DEVICE
} conn_status_t;
extern volatile conn_status_t conn_status;

#endif // GLOBALS_H