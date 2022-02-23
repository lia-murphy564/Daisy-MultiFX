/*
 * File: main.cpp
 * File Created: Wednesday, 5th January 2022 8:20 pm
 * Author: Lia Murphy
 */

#include <iostream>
#include <vector>

#include "daisy.h"
#include "daisysp.h"
#include "daisy_seed.h"

#include "json.hpp"
#include "parameter.h"
#include "parameter_controller.h"

using namespace daisy;
using namespace daisy::seed;
using json = nlohmann::json;

#define PARAM_BUFFER_SIZE	10
#define AUDIO_BUFFER_SIZE	128

daisy::DaisySeed hw;

daisy::UartHandler usart1;

// void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
// {
// 	for (size_t i = 0; i < size; i++)
// 	{
// 		out[0][i] = in[0][i];
// 		out[1][i] = in[1][i];
// 	}
// }

void UART_Init(daisy::UartHandler &uart) {
	daisy::UartHandler::Config usart1_conf;
	usart1_conf.periph = UartHandler::Config::Peripheral::USART_1;
	usart1_conf.mode = UartHandler::Config::Mode::TX_RX;
	usart1_conf.wordlength = UartHandler::Config::WordLength::BITS_8;
	usart1_conf.parity = UartHandler::Config::Parity::NONE;
	usart1_conf.stopbits = UartHandler::Config::StopBits::BITS_1;
	usart1_conf.baudrate = (uint32_t)9600;
	usart1_conf.pin_config.rx = Pin(PORTB, 7); // (USART_1 RX) Daisy pin 15
	usart1_conf.pin_config.tx = Pin(PORTB, 6); // (USART_1 TX) Daisy pin 14

	if (uart.Init(usart1_conf) == UartHandler::Result::ERR) {
		hw.PrintLine("Init error");
		//return daisy::UartHandler::CheckError

		
	}


	if (uart.StartRx() == UartHandler::Result::ERR) {
		hw.PrintLine("StartRx error");
	}


}

ParameterTree pt;

int main(void)
{
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	//hw.StartAudio(AudioCallback);
	hw.StartLog(true);

	UART_Init(usart1);


	uint8_t buffer[buffer_size];

	for (int i=0; i < buffer_size; i++)
    	buffer[i] = 0;

	while (true) {

		// recieve parameter json from uart
		usart1.PollReceive(buffer, buffer_size, 10);
		// for (int i = 0; i < buffer_size; i++) {
		// 	hw.Print("%c", buffer[i]);
		// }

		// decode from json into parameter struct
		json data = buffer;
		param param_uart;
		param_uart.label = data.value("label", "");
		param_uart.type = data.value("type", "");
		param_uart.val = data.value("val", 0);

		// read hardware gpio parameters
		#define HW_PARAM_1 17 // this is the daisy pin number
		dsy_gpio hw_param;
		hw_param.pin  = hw.GetPin(HW_PARAM_1);
		hw_param.mode = DSY_GPIO_MODE_ANALOG;
		hw_param.pull = DSY_GPIO_NOPULL;
		dsy_gpio_init(&hw_param);
		int val_hw = dsy_gpio_read(&hw_param);
		// dsy_gpio_write(&gate_output, true);  // set high
		// dsy_gpio_write(&gate_output, false); // set low
		param param_gpio;
		param_gpio.label = "pot0";
		param_gpio.type = "pot";
		param_gpio.val = 50;

		// check if parameters match
		if (param_gpio.label != param_uart.label && param_gpio.type != param_uart.type && param_gpio.val != param_uart.val) {
			// if not match then update parameter tree
			param update = param_gpio;// || param_uart;
			//parameterTree.push_parameter(param_uart, data.value);
			//parameterTree.update();
		}



		//hw.DelayMs(10);
	}
}
