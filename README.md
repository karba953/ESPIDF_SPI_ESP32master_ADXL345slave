# ESPIDF_SPI_ESP32master_ADXL345slave

transaction
addr : slave register address,
cmd : write/read bit,
buffer : address,

*SPI write/read bit : read -> addr|0x80, write -> addr&0x7f
