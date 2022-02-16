/*
 * File: main.cpp
 * File Created: Wednesday, 5th January 2022 8:20 pm
 * Author: Lia Murphy
 */

#include "main.h"

using namespace daisy;
using namespace daisy::seed;

#define ADC			A0

#define BUFFER_SIZE_AUDIO	1024
#define BUFFER_SIZE_PARAM	10





#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 



DaisySeed hw;

uint8_t rxBuf[BUFFER_SIZE_PARAM];


void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	for (size_t i = 0; i < size; i++)
	{
		out[0][i] = in[0][i];
		out[1][i] = in[1][i];
	}
}


int main(void)
{
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAudio(AudioCallback);
	hw.StartLog(true);
	//HAL_Init();
	//UART4_Init();
	

	// AdcChannelConfig adc_config;
	// adc_config.InitSingle(A0);
	// hw.adc.Init(&adc_config, 1);

	// I2CHandle::Config i2c_conf;
	// i2c_conf.periph = I2CHandle::Config::Peripheral::I2C_1;
	// i2c_conf.speed = I2CHandle::Config::Speed::I2C_1MHZ;
	// i2c_conf.mode = I2CHandle::Config::Mode::I2C_SLAVE;
	// i2c_conf.address = 10;
	// i2c_conf.pin_config.scl = {DSY_GPIOB, 8};
	// i2c_conf.pin_config.sda = {DSY_GPIOB, 9};
	// I2CHandle i2c;
	// i2c.Init(i2c_conf);
	//i2c.ReceiveBlocking(0x00, rxBuf, BUFFER_SIZE_PARAM, 10);

	//char* ch;

	// daisy::Pin rx;
	// daisy::Pin tx;
	// rx.port = D30;
	// tx.port = D29;

	
	hw.PrintLine("hello 1");

	UartHandler::Config usart1_conf;
	usart1_conf.periph = UartHandler::Config::Peripheral::USART_1;
	usart1_conf.mode = UartHandler::Config::Mode::TX_RX;
	usart1_conf.wordlength = UartHandler::Config::WordLength::BITS_8;
	usart1_conf.parity = UartHandler::Config::Parity::NONE;
	usart1_conf.stopbits = UartHandler::Config::StopBits::BITS_1;
	usart1_conf.baudrate = (uint32_t)9600;
	// usart1_conf.pin_config.rx = {DSY_GPIOB, 7};  // (USART_1 RX) Daisy pin 15
    // usart1_conf.pin_config.tx = {DSY_GPIOB, 6};  // (USART_1 TX) Daisy pin 14
	//uart4_conf.pin_config = {D29, D30};
	usart1_conf.pin_config.rx = Pin(PORTB, 7);//D29
	usart1_conf.pin_config.tx = Pin(PORTB, 6);//D30
	UartHandler usart1;
	//uart4.Init(uart4_conf);

	hw.PrintLine("hello 2");
	//hw.PrintLine("rx = %d, tx = %d", uart4_conf.pin_config.rx.pin, uart4_conf.pin_config.tx.pin);

	// unsigned int error = uart4.CheckError();


	//hw.StartLog();

	//hw.PrintLine("hello 3");

	if (usart1.Init(usart1_conf) == UartHandler::Result::ERR) {
		hw.PrintLine("Init error");
	}
	else
		hw.PrintLine("Init success");

	// if (uart4.StartRx() == UartHandler::Result::ERR) {
	// 	hw.PrintLine("StartRx error");
	// }
	// else
	// 	hw.PrintLine("StartRx success");
	#define buffer_size 300

	usart1.Init(usart1_conf);
	uint8_t buffer[buffer_size];

	for (int i=0; i < buffer_size; i++)
  	{
    	buffer[i] = 'x';
  	}

	hw.PrintLine("hello 3");
	usart1.StartRx();
	hw.PrintLine("hello 4");
	uint8_t val;

	while(true) {
		//hw.PrintLine("hello 4");
		//val = usart1.PollReceive(buffer, buffer_size, 10);
		usart1.PollReceive(buffer, buffer_size, 10);
		//val = usart1.PopRx();
		//hw.Print("%u  ", buffer[1]);
		for (int i = 0; i < buffer_size; i++) {
			hw.Print("%c", buffer[i]);
		}
		

		//usart1.PollReceive(&val, 1, 10);
		// val = usart1.PopRx();
		//hw.PrintLine("val = %u", val);

		hw.DelayMs(10);

	}
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {	
// 	for (int i = 0; i < 10; i++)
// 		hw.PrintLine("%u", data[i]);
// 	HAL_UART_Receive_IT(&huart4, (uint8_t*)data, 10);
// }
