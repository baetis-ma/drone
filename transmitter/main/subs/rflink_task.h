//spi dependancies
#define NRF24L01_CE_GPIO    16
#define NRF24L01_CE_MASK    (1ULL<<NRF24L01_CE_GPIO)
#include "./spi.h"


int nrf24_transmit_pkt ( uint8_t *data, int length) {
    //setup nrf24l01 transmitter
    gpio_set_level (NRF24L01_CE_GPIO, 0);
    spi_write_byte ( 0x20 | 0x00, 0x00); //no crc, tx mode
    spi_write_byte ( 0x20 | 0x01, 0x00); //no auto ack
    spi_write_byte ( 0x20 | 0x02, 0x01); //pipe0
    spi_write_byte ( 0x20 | 0x03, 0x03);
    spi_write_byte ( 0x20 | 0x04, 0x04);
    spi_write_byte ( 0x20 | 0x05, 0x05); //freq channel 5
    spi_write_byte ( 0x20 | 0x06, 0x06); //low power, 1MB/sec
    spi_write_byte ( 0x20 | 0x11, 0x20); //use all 32 bytes

    //turn on and flush fifo
    spi_write_byte ( 0x20 | 0x00, 0x02); //turn on
    spi_write_byte ( 0xe1, 0x00);        //flush tx fifo
    spi_write_byte ( 0x20 | 0x07, 0x70);
    ets_delay_us(10);                          //busy-wait

    //vTaskDelay(1);
    //printf("   post flush  = 0x%02x  0x%02x\n", 
    //      spi_read_bytes ( 0x07, data, 1), spi_read_bytes ( 0x17, data, 1));

    //send packet to nrf24l01 transmitter
    spi_write_bytes ( 0xa0, data, length);

    //vTaskDelay(1);
    //printf("   post fifow  = 0x%02x  0x%02x\n", 
    //      spi_read_bytes ( 0x07, data, 1), spi_read_bytes ( 0x17, data, 1));

    //ce chip radio enable
    //with jumper wires on bread board ce pulse 45us was about 50%
    gpio_set_level (NRF24L01_CE_GPIO, 1);
    ets_delay_us(500);                          //busy-wait
    gpio_set_level (NRF24L01_CE_GPIO, 0);

    //vTaskDelay(1);
    //printf("   post trans  = 0x%02x  0x%02x\n", 
    //      spi_read_bytes ( 0x07, data, 1), spi_read_bytes ( 0x17, data, 1));

    spi_write_byte ( 0x20 | 0x00, 0x00); //turn off

    return(0);
}

int wait_rcv_pkt ( uint8_t *data, int timeout) {
    //setup nrf24l01 transmitter
    gpio_set_level (NRF24L01_CE_GPIO, 0);
    spi_write_byte ( 0x20 | 0x00, 0x00); //no crc, tx mode
    spi_write_byte ( 0x20 | 0x00, 0x01); //no crc, rx mode
    spi_write_byte ( 0x20 | 0x01, 0x00); //no auto ack
    spi_write_byte ( 0x20 | 0x02, 0x01); //pipe0
    spi_write_byte ( 0x20 | 0x03, 0x03);
    spi_write_byte ( 0x20 | 0x04, 0x00);
    spi_write_byte ( 0x20 | 0x05, 0x05); //freq channel 5
    spi_write_byte ( 0x20 | 0x06, 0x06); //low power, 1MB/sec
    spi_write_byte ( 0x20 | 0x11, 0x20); //use all 32 bytes

    //turn on and flush fifo
    spi_write_byte ( 0x20 | 0x00, 0x03); //turn on
    spi_write_byte ( 0xe2, 0x00); //flush rx fifo
    spi_write_byte ( 0x20 | 0x07, 0x70);

    gpio_set_level (NRF24L01_CE_GPIO, 1);
    ets_delay_us(100);                          //busy-wait

    int waitcnt = 0;
    int timestart = esp_timer_get_time();
    while(1){
	spi_read_bytes ( 0x07, data, 1);
        //ets_delay_us(1000);                          //busy-wait
	vTaskDelay(1);
        if( (data[0] & 0x40) > 1 || waitcnt > timeout) break;
        ++waitcnt;
	//vTaskDelay(1);
    }
    //if (waitcnt > timeout) printf("wait timed out\n");
    if(waitcnt < timeout) {
        spi_read_bytes ( 0x61, data, 32+1);
    }

    //set ce = 0
    gpio_set_level (NRF24L01_CE_GPIO, 0);

    //printf("waited %8.4fsec   ", (float)(esp_timer_get_time()-timestart)/1000000);
    return(waitcnt);
}

void rflink_task () {
    uint8_t data[33];
    TickType_t xLoopStart = xTaskGetTickCount();
    while(1) {
	sprintf((char*) data, "%4d,%4d,%4d,%4d,%4d,%2d", cnt, throttle, yaw, pitch, roll, mode);
	//printf("%s", data);
        nrf24_transmit_pkt ( (uint8_t*)data, 32 );
	++cnt;

	int timeout = 5;  //x10ms - sending out packet - turning on rx, wait up to 50msec
        int waitlen = wait_rcv_pkt ( (uint8_t*)data, timeout); 
	if (waitlen < timeout) for(int a =0; a<32; a++) printf("0x%02x ", data[a]); printf("\n");
	if (waitlen < timeout){
	    fccnt = 256 * data[0] +data[1];
            height = data[2]-128 ;
            heightprog = data[3]-128;
            heading = 2*data[4];
            headingprog = 2*data[5];
            xdisp = data[6]-128;
            ydisp = data[7]-128;
            theta = (256 * data[8] + data[9]) -18000;
            phi = (256 * data[10] + data[11]) -18000;
	    motor1 = 256 * data[12] + data[13];
	    motor2 = 256 * data[14] + data[15];
	    motor3 = 256 * data[16] + data[17];
	    motor4 = 256 * data[18] + data[19];
            voltage= 100+data[20] ;
	
	}

	vTaskDelay(1);
	vTaskDelayUntil(&xLoopStart, 10);
    }
}

