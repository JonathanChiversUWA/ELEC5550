//USB Functionality as device
#include "tinyusb.h" //general functionality
#include "class/hid/hid_device.h" //for functionality as HID device
#include "usb_device.h"
#include "freertos/task.h" //For Task Delay and pdMS TO TICKS
#include "globals.h" //global variables and includes

/* ------------------------------- Definitions ------------------------------ */

#define UART_TASK_STACK_SZ 8192
static bool g_device_installed = false;
volatile bool uart_device_task_running = false;
static TaskHandle_t uart_task_handle = NULL;

/* ----------------------------- USB Descriptors ---------------------------- */

//Descriptors for Mouse/Keyboard reports
const uint8_t hid_report_descriptor[] = 
{
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))
};

//string descriptor for device (from template)
const char* hid_string_descriptor[5] = 
{
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "TinyUSB",             // 1: Manufacturer
    "TinyUSB Device",      // 2: Product
    "123456",              // 3: Serials, should use chip ID
    "UART HID to HOST",  // 4: HID
};

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN) //required for config descriptor
//configuration descriptor for device
static const uint8_t hid_configuration_descriptor[] = 
{
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/* ------------------------- HID DEVICE DEFINITIONS ------------------------- */
// Structure to hold key event information
typedef struct {
    enum key_state {
        KEY_STATE_PRESSED = 0x00,
        KEY_STATE_RELEASED = 0x01
    } state;
    uint8_t modifier;
    uint8_t key_code;
} key_event_t;

// Lookup table to convert HID keycodes to ASCII characters
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
/*                             Required USB Config                            */
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
    uart_device_task_running = true;

    while(uart_device_task_running){
        if (module_status == STATUS_CALIBRATION){
            continue;
        }else if(xQueueReceive(uart_queue, (void *)&event, pdMS_TO_TICKS(100))){ // 100ms timeout
            bzero(dtmp,BUF_SIZE); //Clear Buffer
            switch(event.type){
                case UART_DATA:
                    int len = uart_read_bytes(uart_num,dtmp,event.size,portMAX_DELAY);
                    if (len>0){
                        gpio_set_level(LED_GPIO_COM,1); //COM LED on
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
        }else{
            gpio_set_level(LED_GPIO_COM,0); //Power LED on after 200ms
        }
    }

    free(dtmp);
    gpio_set_level(LED_GPIO_COM,0); //Turn off COM LED
    uart_task_handle = NULL;
    vTaskDelete(NULL);
}



/* ---------------------------------- Main ---------------------------------- */

void usb_device_start(void)
{
    if (g_device_installed) return;

    //configuration to act as a device
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL, //not defined yet
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false, //using esp32s3 internal phy
        .configuration_descriptor = hid_configuration_descriptor
    };
    //install hid device with pointer to configuration
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    g_device_installed = true;

}

void usb_device_uart_start(void)
{
    if (uart_task_handle != NULL || module_status == STATUS_CALIBRATION) return;
    xTaskCreate(uart_event_task, "uart_event_task", UART_TASK_STACK_SZ, NULL, 12, &uart_task_handle);
}

void usb_device_stop(void)
{
    if (!g_device_installed) return;

    uart_device_task_running = false;
    while (uart_task_handle != NULL) {vTaskDelay(pdMS_TO_TICKS(10));}

    tinyusb_driver_uninstall();
    g_device_installed = false;
}

bool usb_device_is_connected(void)
{
    // tinyusb provides tud_mounted() to indicate device is enumerated by host
    if (g_device_installed){
        return tud_mounted();
    }
    return false;
}