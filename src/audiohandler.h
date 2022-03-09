#ifndef _AUDIOHANDLER_H_
#define _AUDIOHANDLER_H_

#include "daisy.h"
#include "daisysp.h"
#include "daisy_seed.h"
#include "stm32h7xx_hal.h"

#include "parameter_controller.h"

class AudioHandler {
    private:
        ParameterTree* pt;
    public:
        AudioHandler() {}
        ~AudioHandler() {}

        float processAudioSample(float xn);

        void updateParameters(ParameterTree* pt);

        float AudioHandler::getParameterValue(int idx);


};



#endif