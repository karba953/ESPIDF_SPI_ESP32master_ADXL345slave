
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"


#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5


spi_device_handle_t spi;

void spi_master_init()
{
    esp_err_t ret;
    
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000*1000,           // SPI clock frequency
        .mode = 3,                              // CPOL, CPHA
        .address_bits = 8,
        .command_bits = 0,
        .dummy_bits = 0,
        .spics_io_num = PIN_NUM_CS,              // CS pin
        .queue_size = 1,                        // We want to be able to queue 1 transaction at a time
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .flags = 0,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    
    // Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    
    // Add the SPI device to the bus
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}



void SPI_write(spi_device_handle_t spi, uint8_t ADDR, const uint8_t DATA)
{
    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));           // Zero out the transaction
    t.length = 8;                       // Command is 8 bits
    t.addr =  ADDR & 0x7f;                // Point to the address buffer
    t.flags = 0;
    t.cmd = 0;
    t.user = NULL;
    t.length = 8;                       // Data is 8 bits
    t.tx_buffer = &DATA;                // Point to the data buffer
    t.rx_buffer = NULL;
    t.rxlength = 0;
    ret = spi_device_polling_transmit(spi, &t);  // Transmit the data
    assert(ret==ESP_OK);                // Should have had no issues

    
}



uint8_t SPI_Read(uint8_t ADDR){
 
    uint8_t byteData = 0;
    spi_transaction_t t;
    esp_err_t ret;

    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.addr = ADDR | 0x80;
    t.flags = 0;
    t.cmd = 0;
    t.length = 8;
    t.rxlength = 8;
    t.rx_buffer = &byteData;

    ret = spi_device_polling_transmit(spi, &t);  //Transmit
    assert(ret==ESP_OK);            //Should have had no issues.
    
    
    return byteData;


}
void slave_init()
{
    printf("slave init\n");

    SPI_write(spi ,0x2D, 0x08);
    SPI_write(spi, 0x31, 0x0B);
    SPI_write(spi, 0x38, 0x80);
   

}


void app_main()
{
    // Initialize the SPI bus
    spi_master_init();

    //Initialize slave
    slave_init();

    while(1){

        printf("test : 0x%X\n",SPI_Read(0x00));
        printf("0x2D : 0x%02X\n",SPI_Read(0x2D));
        printf("0x31 : 0x%02X\n",SPI_Read(0x31));
        printf("0x38 : 0x%02X\n",SPI_Read(0x38));

        vTaskDelay(100);
    }
}

