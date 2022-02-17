/*
 * File: main.cpp
 * File Created: Wednesday, 5th January 2022 8:20 pm
 * Author: Lia Murphy
 */

#include <iostream>

#include "daisy.h"
#include "daisysp.h"
#include "daisy_seed.h"

#include "json.hpp"
#include "parameter.h"

using namespace daisy;
using namespace daisy::seed;
using json = nlohmann::json;

DaisySeed hw;

// void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
// {
// 	for (size_t i = 0; i < size; i++)
// 	{
// 		out[0][i] = in[0][i];
// 		out[1][i] = in[1][i];
// 	}
// }

struct param {
	std::string label;
	std::string type;
	uint16_t val;
};

void JSON_Handler() {

	json data = 
	{
		{"label", "pot1"},
		{"type", "pot"},
		{"val", 50}
	};
	
	param p;
	p.label = data.value("label");
	p.type = data.value("type");
	p.val = data.value("val");

	std::cout << p.label << " " << p.type << " " << p.val << "\n";
}


int main(void)
{
	JSON_Handler();
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAudio(AudioCallback);
	hw.StartLog(true);

	UartHandler::Config usart1_conf;
	usart1_conf.periph = UartHandler::Config::Peripheral::USART_1;
	usart1_conf.mode = UartHandler::Config::Mode::TX_RX;
	usart1_conf.wordlength = UartHandler::Config::WordLength::BITS_8;
	usart1_conf.parity = UartHandler::Config::Parity::NONE;
	usart1_conf.stopbits = UartHandler::Config::StopBits::BITS_1;
	usart1_conf.baudrate = (uint32_t)9600;
	usart1_conf.pin_config.rx = Pin(PORTB, 7); // (USART_1 RX) Daisy pin 15
	usart1_conf.pin_config.tx = Pin(PORTB, 6); // (USART_1 TX) Daisy pin 14

	UartHandler usart1;

	if (usart1.Init(usart1_conf) == UartHandler::Result::ERR) {
		hw.PrintLine("Init error");
	}

	if (usart1.StartRx() == UartHandler::Result::ERR) {
		hw.PrintLine("StartRx error");
	}

	#define buffer_size 300

	uint8_t buffer[buffer_size];

	for (int i=0; i < buffer_size; i++)
  	{
    	buffer[i] = 0;
  	}


	while(true) {

		usart1.PollReceive(buffer, buffer_size, 10);

		for (int i = 0; i < buffer_size; i++) {
			hw.Print("%c", buffer[i]);
		}
		

		hw.DelayMs(10);
	}
}
