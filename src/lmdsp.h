//
//  LMdsp.h
//
//  Created by Lia Murphy on 12/8/21.
//  Copyright © 2021 Lia Murphy. All rights reserved.
//

#ifndef _LMDSP_H_
#define _LMDSP_H_

#include <vector>
#include <algorithm>
#include <cmath>

#define M_PI 3.1415

template <typename T>
T l_clamp(T input, T low, T high) {
    return std::min(std::max(input, low), high);
}

enum filterType {
    kLPF,
    kHPF,
    kAPF,
};

class BiquadParameters {
    private:

    public:
        void setParameters() {

        }

        void getParameters() {

        }
};

class Biquad {
    private:
        double xn, xnm1, xnm2, ynm1, ynm2;
        double a0, a1, a2, b0, b1, b2;
        float num, den;
        enum filterType _type;

        double _res, _fc;
        double alpha;
        double _fs;
        double two_pi_fs;

    public:
        Biquad() {};
        ~Biquad() {};

        void Init(double fs) {

            _fs = fs;
            //sample_rate_ = sample_rate;
            two_pi_fs = (2 * M_PI) / _fs;
            //two_pi_d_sr_ = TWOPI_F / sample_rate_;

            _fc = 500;
            _res  = 0.7;

            Reset();

            xnm1 = xnm2 = 0.0;
            ynm1 = ynm2 = 0.0;
        }

        void Reset() {
            // xnm1 = xnm2 = 0.0;
            // ynm1 = ynm2 = 0.0;

            float w = _fc * two_pi_fs;

            alpha = sin(w) / (2 * _q);

            switch (_type) {
                case filterType::kLPF:
                    b0 = (1 - cos(w)) / 2;
                    b1 = 1 - cos(w);
                    b2 = (1 - cos(w)) / 2;
                    a0 = 1 + alpha;
                    a1 = -2 * cos(w);
                    a2 = 1 - alpha;

                case filterType::kHPF:


                case filterType::kAPF:
            }
        }

        void Init() {
            
        }

        void setType(enum filterType type) {
            _type = type;
        }

        void setParameters(float q, float fc) {
            _q = q;
            _fc = fc;
        }

        float processAudioSample(float in) {
            xn = in;

            float yn = (b0* xn + b1 * xnm1 + b2 * xnm2) / (1 + a1 * ynm1 + a2 * ynm2);

            xnm2 = xnm1;
            xnm1 = xn;
            ynm2 = ynm1;
            ynm1 = yn;

            return yn;
        }
};

//template <typename T, size_t buffer_size>
template <size_t buffer_size>
class RingBuffer {
    private:
        float buffer[buffer_size];
        size_t w_ptr;
        size_t delay;
        const size_t capacity;
        
        float frac;
        
    public:
        RingBuffer() {};
        ~RingBuffer() {};

        void Reset() {
            for (int i = 0; i < buffer_size; i++) {
                buffer[i] = 0.0f;
            }
            w_ptr = 0;
            delay = 1;
        }

        size_t size() {
            return buffer_size;
        }

        size_t capacity() {
            
        }

        bool empty() {
            
        }

        bool full() {

        }
};

class CircularBuffer
{
private:
    std::vector<float> buffer;
    int fs;
    int size;
    int idx;
    double time;
    
public:
    CircularBuffer() {}
    
    CircularBuffer(int sample_rate) {
        idx = 0;
        fs = sample_rate;
        size = fs;
    }
    
    ~CircularBuffer() {
        buffer.clear();
    }
    
    void set_samplerate(double _fs) {
        fs = _fs;
    }
    
    void set_time(double time_ms) {
        time = time_ms;
        size = time * fs;
        buffer.resize(size);
    }
    
    void clear_buffer() {
        idx = 0;
        for (int i = 0; i < size; i++)
            buffer.push_back(0.0);
    }
    
    void write(float x) {
        buffer[idx] = x;
        if (++idx >= size)
            idx = 0;
    }
    
    float read() {
        return buffer[idx];
    }
};

class LFO
{
private:
    enum type { kSin };
    double rate;
    int fs;
    double lfo_val;
    
public:
    LFO() {
        lfo_val = 0;
    }
    ~LFO() {}
    
    void set_samplerate(double _fs) {
        fs = _fs;
    }
    
    double read() {
        return sin(lfo_val);
    }
    
    double read(double freq)
    {
        rate = 2 * M_PI / (freq / fs);
        lfo_val += rate;

        if (lfo_val > M_PI)
            lfo_val -= 2 * M_PI;
        
        return l_clamp(sin(lfo_val), -0.5, 0.5);
    }
};

class AudioDelay
{
private:
    int fs;
    //float x1, y, fb;
    CircularBuffer buffer; // or should i statically allocate buffer? what is the purpose of dynamically allocating it?
                            // is dsp performance better on stack or heap?
    
public:
    AudioDelay() {}
    
    AudioDelay(int sample_rate) {
        sample_rate = fs;
        //buffer = CircularBuffer(fs);
        //buffer = new CircularBuffer(fs);
        buffer.clear_buffer();
    }
    
    ~AudioDelay() {
        //delete buffer;
    }
    
    float process(float x_in, float time_ms, float feedback_pct) {
        float x1, y, fb;
        //buffer.set_time(2000);
        x1 = buffer.read();
        //y = x_in + x1;
        //fb = y * feedback_pct;
        buffer.write(x_in);
        y = x1 * 10;
        return y;
    }
    
    void set_time(float time_ms) {
        buffer.set_time(time_ms);
    }
};

class Waveshaper
{
private:


    float y;
    
public:
    Waveshaper() {}
    ~Waveshaper() {}
    
    enum type { kSin, kArctan, kHardclip };
    type curr;
    
    void set_type(type t) {
        if (t == kSin) {
            curr = t;
        }
        else if (t == kArctan) {
            curr = t;
        }
        else if (t == kHardclip) {
            curr = t;
        }
    }
    
    float process(float x_in, float pre_gain, float post_gain) {
        
        if (curr == kSin) {
                y = post_gain * sin(pre_gain * x_in);
        }
        
        else if (curr == kArctan) {
                y = post_gain * atan(pre_gain * x_in);
        }
        
        else if (curr == kHardclip) {
            x_in *= pre_gain;
            if (x_in > 0.75)
                y = 0.75;
            else if (x_in < -0.75)
                y = -0.75;
            y *= post_gain;
        }
        return y;
    }
};


#endif /* LMdsp_h */
