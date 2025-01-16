**Konfigurasi SPI Flash Memory**
Clock 64 MHz
Mode : Full Duplex Master
Data Size : 8 Bits
Prescaler : 16

//Agar pembacaan memory tidak kosong (-nan)
for (int i = 0; i < 3; i++) {
		 ReadData(address, sizeof(float));
		  if (!isnan(read_data_float)) {
			  break;
		  }
	tx_thread_sleep(50);
 }

//Contoh menulis data 
uint32_t address = 0x000000;
float nilai = 5.1234567;
write_value(nilai, address);

/Contoh membaca data
uint32_t address = 0x000000;
ReadData(address, sizeof(float));
printf("Read Data: %.2f |", read_data_float);

**Konvigurasi UART untuk transmit dengan single wire**
Clock 64 MHz
Baud Rate : 9600
Enable NVIC Settings

//Callback Receive
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
    }
}

//Contoh transmit
HAL_HalfDuplex_EnableReceiver(&huart2); //Mengaktifkan mode receive pada half duplex
HAL_UART_Receive(&huart2, requestBuffer, 1, 1000); //Memulai menerima data
tx_thread_sleep(30); //Jeda 30ms
int len = snprintf(buffer, sizeof(buffer), "Temp: %.2f C\n", temperature); //Data yang akan dikirim
HAL_HalfDuplex_EnableTransmitter(&huart2); //Mengaktifkan mode transmit pada half duplex 
HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 1000); //Memulai mengirim data
