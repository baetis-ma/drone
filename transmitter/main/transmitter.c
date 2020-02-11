#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ringbuf.h"

#include "esp8266/spi_struct.h"
#include "esp8266/gpio_struct.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/spi.h"
#include <esp_timer.h>

//globals
int cnt = 0;
int throttle = 1000; int yaw = 1500; int pitch = 1500; int roll = 1500; int mode = 7;

//spi dependancies
#define NRF24L01_CE_GPIO    16
#define NRF24L01_CE_MASK    (1ULL<<NRF24L01_CE_GPIO)
#include "./subs/rflink_task.h"

//i2c depenencies
#define I2C_SCL_IO           5                /*!< gpio number for I2C master clock */
#define I2C_SDA_IO           4                /*!< gpio number for I2C master data  */
#include "./subs/i2c.h"
#include "./subs/ssd1306.h"

//joystick to commands
#include "./subs/ads1115.h"

void app_main(void)
{
    spi_initialize(); //setup transmitter radio
    uint8_t data[33] = { 'H', 'e', 'l', 'l', 'o', '\n' }; //max length nrf20l01 packet
    //nrf24_transmit_pkt ( data, 6 );
    //for (int a = 0; a < 0x1d; a++) printf(" reg 0x%02x  data 0x%02x\n", a, spi_read_bytes ( a, data, 1));

    i2c_init();   //setup and detect devices on i2c interface
    i2c_detect();

    ssd1305_init();   //setup oled display
    ssd1305_blank(0x00);
    char disp_str[128] = "4 Hi There";
    ssd1305_text(disp_str);

    //sends transmitter commands every 50 msec
    xTaskCreate (rflink_task, "rflink_task", 2048, NULL, 4, NULL);
    //reads joystick adcs every 40msec
    xTaskCreate (read_joysticks, "read_joysticks", 2048, NULL, 4, NULL);

    TickType_t xLoopStart = xTaskGetTickCount();
    while(1) {
	sprintf (disp_str, "4 F-450 Xmit|||1 height=%3d| 0x%02x",cnt, data[1]);
        ssd1305_text(disp_str);
	vTaskDelay(100);
    }

}
