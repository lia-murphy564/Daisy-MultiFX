/*
 * File: main.cpp
 * File Created: Wednesday, 5th January 2022 8:20 pm
 * Author: Lia Murphy
 */

//#include <iostream>
#include <vector>

#include "daisy.h"
#include "daisysp.h"
#include "daisy_seed.h"
#include "stm32h7xx_hal.h

//#include "inc/json.hpp"
//#include "parameter.h"
#include "parameter_controller.h"
#include "audiohandler.h"

using namespace daisy;
using namespace daisy::seed;
//using json = nlohmann::json;

#define PARAM_BUFFER_SIZE	10
#define AUDIO_BUFFER_SIZE	128
#define HTIM1_MS	100

daisy::DaisySeed hw;

daisy::UartHandler usart1;
daisy::TimerHandle htim1;
daisy::GPIO gpio;

static uint32_t DSY_SDRAM_BSS audio_buffer[AUDIO_BUFFER_SIZE];
static uint16_t DSY_SDRAM_BSS param_buffer[PARAM_BUFFER_SIZE];

AudioHandler audio;

void TIM1_Init(daisy::TimerHandle& tim) {
	daisy::TimerHandle::Config conf;
	conf.periph = daisy::TimerHandle::Config::Peripheral::TIM_2;
	conf.dir = daisy::TimerHandle::Config::CounterDir::UP;
	tim.Init(conf);
	tim.DelayMs(HTIM1_MS);
	tim.Start();
}

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

void AudioCallback(daisy::AudioHandle::InputBuffer in, daisy::AudioHandle::OutputBuffer out, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
		out[0][i] = audio.processAudioSample(in[0][i]);
		out[1][i] = audio.processAudioSample(in[1][i]);

    }
} 

int main(void)
{
	hw.Init();
	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAudio(AudioCallback);
	hw.StartLog(true);
	
	UART_Init(usart1);
	TIM1_Init(htim1);
	std::vector<uint32_t> paramBuf;
	ParameterTree pt;

	while (true) {

		while (usart1.Readable()) {

			uint32_t val = usart1.PopRx();
			paramBuf.push_back(val);
			//hw.Print(val);
	
			if (!usart1.Readable()) {
				usart1.FlushRx();
			}

			hw.DelayMs(10);
		}

		while (!paramBuf.empty()) {
			uint32_t curr = paramBuf.back();
			paramBuf.pop_back();
		}

	}
}
