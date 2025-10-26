#include <stdint.h> //uint8_t etc
#include "driver/uart.h" //for UART, REQUIRES esp_driver_uart
#include "driver/gpio.h" //for configuring GPIOs
#include "class/hid/hid_device.h" //for functionality as HID device
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h" //for functionality as a device
// #include "esp_log.h"

/* -------------------------------------------------------------------------- */
/*                          CONSTANTS AND DEFINITIONS                         */
/* -------------------------------------------------------------------------- */

#define led_gpio 36

/* ---------------------------------- UART ---------------------------------- */
const int uart_buffer_size = (1024*2); 
const uart_port_t uart_num = UART_NUM_2;
QueueHandle_t uart_queue; //UART Queue
QueueHandle_t app_event_queue = NULL; //Event Queue
#define UART_TASK_STACK_SZ 8192
#define BUF_SIZE 128

/* ----------------------------- USB DESCRIPTORS ---------------------------- */
//https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/usb_device.html

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

//Descriptors for Mouse/Keyboard reports
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))
};

//string descriptor for device (from template)
const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "UART HID to HOST",  // 4: HID
};

//configuration descriptor for device
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/* ------------------------- HID DEVICE DEFINITIONS ------------------------- */
typedef struct {
    enum key_state {
        KEY_STATE_PRESSED = 0x00,
        KEY_STATE_RELEASED = 0x01
    } state;
    uint8_t modifier;
    uint8_t key_code;
} key_event_t;

const uint8_t keycode2ascii [57][2] = {
    {0, 0}, /* HID_KEY_NO_PRESS        */
    {0, 0}, /* HID_KEY_ROLLOVER        */
    {0, 0}, /* HID_KEY_POST_FAIL       */
    {0, 0}, /* HID_KEY_ERROR_UNDEFINED */
    {'a', 'A'}, /* HID_KEY_A               */
    {'b', 'B'}, /* HID_KEY_B               */
    {'c', 'C'}, /* HID_KEY_C               */
    {'d', 'D'}, /* HID_KEY_D               */
    {'e', 'E'}, /* HID_KEY_E               */
    {'f', 'F'}, /* HID_KEY_F               */
    {'g', 'G'}, /* HID_KEY_G               */
    {'h', 'H'}, /* HID_KEY_H               */
    {'i', 'I'}, /* HID_KEY_I               */
    {'j', 'J'}, /* HID_KEY_J               */
    {'k', 'K'}, /* HID_KEY_K               */
    {'l', 'L'}, /* HID_KEY_L               */
    {'m', 'M'}, /* HID_KEY_M               */
    {'n', 'N'}, /* HID_KEY_N               */
    {'o', 'O'}, /* HID_KEY_O               */
    {'p', 'P'}, /* HID_KEY_P               */
    {'q', 'Q'}, /* HID_KEY_Q               */
    {'r', 'R'}, /* HID_KEY_R               */
    {'s', 'S'}, /* HID_KEY_S               */
    {'t', 'T'}, /* HID_KEY_T               */
    {'u', 'U'}, /* HID_KEY_U               */
    {'v', 'V'}, /* HID_KEY_V               */
    {'w', 'W'}, /* HID_KEY_W               */
    {'x', 'X'}, /* HID_KEY_X               */
    {'y', 'Y'}, /* HID_KEY_Y               */
    {'z', 'Z'}, /* HID_KEY_Z               */
    {'1', '!'}, /* HID_KEY_1               */
    {'2', '@'}, /* HID_KEY_2               */
    {'3', '#'}, /* HID_KEY_3               */
    {'4', '$'}, /* HID_KEY_4               */
    {'5', '%'}, /* HID_KEY_5               */
    {'6', '^'}, /* HID_KEY_6               */
    {'7', '&'}, /* HID_KEY_7               */
    {'8', '*'}, /* HID_KEY_8               */
    {'9', '('}, /* HID_KEY_9               */
    {'0', ')'}, /* HID_KEY_0               */
    {'\r', '\r'}, /* HID_KEY_ENTER           */
    {0, 0}, /* HID_KEY_ESC             */
    {'\b', 0}, /* HID_KEY_DEL             */
    {0, 0}, /* HID_KEY_TAB             */
    {' ', ' '}, /* HID_KEY_SPACE           */
    {'-', '_'}, /* HID_KEY_MINUS           */
    {'=', '+'}, /* HID_KEY_EQUAL           */
    {'[', '{'}, /* HID_KEY_OPEN_BRACKET    */
    {']', '}'}, /* HID_KEY_CLOSE_BRACKET   */
    {'\\', '|'}, /* HID_KEY_BACK_SLASH      */
    {'\\', '|'}, /* HID_KEY_SHARP           */  // HOTFIX: for NonUS Keyboards repeat HID_KEY_BACK_SLASH
    {';', ':'}, /* HID_KEY_COLON           */
    {'\'', '"'}, /* HID_KEY_QUOTE           */
    {'`', '~'}, /* HID_KEY_TILDE           */
    {',', '<'}, /* HID_KEY_LESS            */
    {'.', '>'}, /* HID_KEY_GREATER         */
    {'/', '?'} /* HID_KEY_SLASH           */
};

/* -------------------------------------------------------------------------- */
/*                                   Config                                   */
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

/* -------------------------------- UART Init ------------------------------- */

static void uart_init(void){ //static - internal function for this file only. Void - no return
    uart_config_t uart_config = {
        .baud_rate = 38400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_ODD,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, //RTS and CTS can't be sent through laser, so don't use
    };
    //load configuration above
    ESP_ERROR_CHECK(uart_param_config(uart_num,&uart_config));
    //Install driver: Driver Num, TX / RX Buffer, 10 is length of queue, &uart_queue is pointer to queue (rather than new variable), 0 no flags
    ESP_ERROR_CHECK(uart_driver_install(uart_num,uart_buffer_size,uart_buffer_size,10, &uart_queue, 0)); 
    //invert the UART so laser is not on for as long
    ESP_ERROR_CHECK(uart_set_line_inverse(uart_num, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV));
    //Rx, Tx, RTS, CTS (IO number and NO CHANGE for default / unused)
    ESP_ERROR_CHECK(uart_set_pin(uart_num,44,43,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
}

/* -------------------------------------------------------------------------- */
/*                               Main Callbacks                               */
/* -------------------------------------------------------------------------- */

static void hid_keyboard_report_callback(const uint8_t *const data, const int length){
    hid_keyboard_report_t *t_report = (hid_keyboard_report_t *)data;

    // Defined in hid.h
    // typedef struct TU_ATTR_PACKED{
    //     uint8_t modifier;   /**< Keyboard modifier (KEYBOARD_MODIFIER_* masks). */
    //     uint8_t reserved;   /**< Reserved for OEM use, always set to 0. */
    //     uint8_t keycode[6]; /**< Key codes of the currently pressed keys. */
    // } hid_keyboard_report_t;

    if (length < sizeof(hid_keyboard_report_t)){
        return; //either an error in code or corruption of data
    }

    if (tud_hid_ready()){
        tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, t_report->modifier, t_report->keycode);
    }
}

static void hid_mouse_report_callback(const uint8_t *const data, const int length){
    hid_mouse_report_t *m_report = (hid_mouse_report_t *)data;

    //Defined in hid.h
    // typedef struct TU_ATTR_PACKED{
    //     uint8_t buttons; /**< buttons mask for currently pressed buttons in the mouse. */
    //     int8_t  x;       /**< Current delta x movement of the mouse. */
    //     int8_t  y;       /**< Current delta y movement on the mouse. */
    //     int8_t  wheel;   /**< Current delta wheel movement on the mouse. */
    //     int8_t  pan;     // using AC Pan
    // } hid_mouse_report_t;

    if (length < sizeof(hid_mouse_report_t)){
        return; //either an error in code or corruption of data.
    }
    if (tud_hid_ready()) {
        tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, m_report->buttons, m_report->x, m_report->y, m_report->wheel, 0);
    }
}

static void app_send_hid_data(uint8_t *data, int length){
    // uint8_t keycode[6] = {0};
    if (data[0] == 1){ //keyboard
        hid_keyboard_report_callback(&data[1],length-1);
    }else if (data[0] == 2){ //mouse
        hid_mouse_report_callback(&data[1],length-1);
    }
}

static void uart_event_task(void *params){
    uart_event_t event; //holds UART event
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE); //allocate memory for temp buffer

    for(;;){
        if(xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)){
            bzero(dtmp,BUF_SIZE); //Clear Buffer
            switch(event.type){
                case UART_DATA:
                    int len = uart_read_bytes(uart_num,dtmp,event.size,portMAX_DELAY);
                    if (len>0){
                        app_send_hid_data(dtmp,len);
                    }
                    // app_send_hid_data(dtmp,len);
                    break;
                case UART_FIFO_OVF:
                    uart_flush_input(uart_num);
                    break;
                case UART_BUFFER_FULL:
                    uart_flush_input(uart_num);
                    break;
                default:
                    break;
            }
        }
    }

    free(dtmp);
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */

void app_main(void){

    gpio_reset_pin(led_gpio);
    gpio_set_direction(led_gpio,GPIO_MODE_OUTPUT);

    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL, //not defined yet
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false, //using esp32s3 internal phy
        .configuration_descriptor = hid_configuration_descriptor,
    };

    //install hid device with pointer to configuration
    gpio_set_level(led_gpio,1);
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    gpio_set_level(led_gpio,0);

    //variables for UART reading
    // uart_init();

    // xTaskCreate(uart_event_task, "uart_event_task", UART_TASK_STACK_SZ, NULL, 12, NULL);

    while(1){
        vTaskDelay(10);
        if (tud_mounted()){ //if device properly connected
            gpio_set_level(led_gpio,1);
        }else{
            gpio_set_level(led_gpio,0);
        }
    }
}