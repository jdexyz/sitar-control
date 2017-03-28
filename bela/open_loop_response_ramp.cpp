/*
 ____  _____ _        _    
| __ )| ____| |      / \   
|  _ \|  _| | |     / _ \  
| |_) | |___| |___ / ___ \ 
|____/|_____|_____/_/   \_\

The platform for ultra-low latency audio and sensor processing

http://bela.io

A project of the Augmented Instruments Laboratory within the
Centre for Digital Music at Queen Mary University of London.
http://www.eecs.qmul.ac.uk/~andrewm

(c) 2016 Augmented Instruments Laboratory: Andrew McPherson,
    Astrid Bin, Liam Donovan, Christian Heinrichs, Robert Jack,
    Giulio Moro, Laurel Pardue, Victor Zappi. All rights reserved.

The Bela software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt


////////////////////////////////////////////////////////////////////


.d8888. d888888b d888888b  .d8b.  d8888b.       d88888b .88b  d88.
88'  YP   `88'   `~~88~~' d8' `8b 88  `8D       88'     88'YbdP`88
`8bo.      88       88    88ooo88 88oobY'       88ooooo 88  88  88
  `Y8b.    88       88    88~~~88 88`8b         88~~~~~ 88  88  88
db   8D   .88.      88    88   88 88 `88.       88.     88  88  88
`8888Y' Y888888P    YP    YP   YP 88   YD       Y88888P YP  YP  YP


# Control algorithm for the Sitar EM
Jeremy Dahan for Centre d'Automatique et Systemes @ Mines ParisTech, and Trublion
Jan 2017


## State sensor
This program anaylises the signal coming from the piezo sensor assuming the signal is roughly periodic, with a known fundamental frequency.
It outputs the detected amplitude and phase of the selected harmonics with a constant complexity, which allows the values to be updated in real-time, at the sampling rate of the ADC.


### Notation

In variable names, 
`_M#_`    corresponds to Mode #
`_R`      corresponds to Real value
`_I`      corresponds to Imaginary value
`##_Sum`  corresponds to the sum of the elements in array `##`

Output values are `amplitude_squared_M#` and `phase_M#`


## Basic Damping


*/


// For now, fixed frequency : 
#define FUNDAMENTAL_FREQUENCY   74.0
#define SAMPLING_FREQUENCY      44100.0
#define NUMBER_OF_SAMPLES       595
#define PI                      3.14159265358979323846


#define SEC 44100
#define DURATION 30*SEC
#define NON_AUDIO_FREQ 100
#define NON_AUDIO_FREQ_S "441"


#include <Bela.h>
#include <array>
#include <cmath>
#include <iostream>
#include <math_neon.h>
#include <WriteFile.h>
#include <sndfile.h>

#include <fstream>
using namespace std;


float audioArray[DURATION*2];
float commandSignalArray[DURATION*2];


ofstream commandAmplitude_M1_File;
// ofstream amplitudeObserver_M1_File;
ofstream amplitude_M1_File;
ofstream phase_M1_File;
float commandAmplitude_M1_Array[DURATION];
// float amplitudeObserver_M1_Array[DURATION];
float amplitude_M1_Array[DURATION];
float phase_M1_Array[DURATION];

ofstream commandAmplitude_M2_File;
// ofstream amplitudeObserver_M2_File;
ofstream amplitude_M2_File;
ofstream phase_M2_File;
float commandAmplitude_M2_Array[DURATION];
// float amplitudeObserver_M2_Array[DURATION];
float amplitude_M2_Array[DURATION];
float phase_M2_Array[DURATION];

ofstream commandAmplitude_M3_File;
// ofstream amplitudeObserver_M3_File;
ofstream amplitude_M3_File;
ofstream phase_M3_File;
float commandAmplitude_M3_Array[DURATION];
// float amplitudeObserver_M3_Array[DURATION];
float amplitude_M3_Array[DURATION];
float phase_M3_Array[DURATION];

ofstream commandAmplitude_M4_File;
// ofstream amplitudeObserver_M4_File;
ofstream amplitude_M4_File;
ofstream phase_M4_File;
float commandAmplitude_M4_Array[DURATION];
// float amplitudeObserver_M4_Array[DURATION];
float amplitude_M4_Array[DURATION];
float phase_M4_Array[DURATION];

ofstream commandAmplitude_M5_File;
// ofstream amplitudeObserver_M5_File;
ofstream amplitude_M5_File;
ofstream phase_M5_File;
float commandAmplitude_M5_Array[DURATION];
// float amplitudeObserver_M5_Array[DURATION];
float amplitude_M5_Array[DURATION];
float phase_M5_Array[DURATION];



const float FREQUENCY_OF_INTEREST_M1 = 1*FUNDAMENTAL_FREQUENCY;
const float FREQUENCY_OF_INTEREST_M2 = 2*FUNDAMENTAL_FREQUENCY;
const float FREQUENCY_OF_INTEREST_M3 = 3*FUNDAMENTAL_FREQUENCY;
const float FREQUENCY_OF_INTEREST_M4 = 4*FUNDAMENTAL_FREQUENCY;
const float FREQUENCY_OF_INTEREST_M5 = 5*FUNDAMENTAL_FREQUENCY;

const float PHASE_INCREMENT_M1 = 2*PI*FREQUENCY_OF_INTEREST_M1/SAMPLING_FREQUENCY;
const float PHASE_INCREMENT_M2 = 2*PI*FREQUENCY_OF_INTEREST_M2/SAMPLING_FREQUENCY;
const float PHASE_INCREMENT_M3 = 2*PI*FREQUENCY_OF_INTEREST_M3/SAMPLING_FREQUENCY;
const float PHASE_INCREMENT_M4 = 2*PI*FREQUENCY_OF_INTEREST_M4/SAMPLING_FREQUENCY;
const float PHASE_INCREMENT_M5 = 2*PI*FREQUENCY_OF_INTEREST_M5/SAMPLING_FREQUENCY;



float latestSample = 0;
int sampleIndex = 0;
float ONE_OVER_NUMBER_OF_SAMPLES_SQUARED =  1000000.0/float(NUMBER_OF_SAMPLES*NUMBER_OF_SAMPLES);
float ONE_OVER_NUMBER_OF_SAMPLES         =  1000.0/float(NUMBER_OF_SAMPLES);

int printIndex = 0;


float COMPLEX_EXPONENTIAL_TABLE_M1_R[NUMBER_OF_SAMPLES];
float COMPLEX_EXPONENTIAL_TABLE_M1_I[NUMBER_OF_SAMPLES];
float exponentialTimesSamples_M1_R[NUMBER_OF_SAMPLES];
float exponentialTimesSamples_M1_I[NUMBER_OF_SAMPLES];

float exponentialTimesSamples_M1_I_Sum = 0;
float exponentialTimesSamples_M1_R_Sum = 0;

float amplitude_squared_M1 = 0;
float phase_M1 = 0;

float COMPLEX_EXPONENTIAL_TABLE_M2_R[NUMBER_OF_SAMPLES];
float COMPLEX_EXPONENTIAL_TABLE_M2_I[NUMBER_OF_SAMPLES];
float exponentialTimesSamples_M2_R[NUMBER_OF_SAMPLES];
float exponentialTimesSamples_M2_I[NUMBER_OF_SAMPLES];

float exponentialTimesSamples_M2_I_Sum = 0;
float exponentialTimesSamples_M2_R_Sum = 0;

float amplitude_squared_M2 = 0;
float phase_M2 = 0;

float COMPLEX_EXPONENTIAL_TABLE_M3_R[NUMBER_OF_SAMPLES];
float COMPLEX_EXPONENTIAL_TABLE_M3_I[NUMBER_OF_SAMPLES];
float exponentialTimesSamples_M3_R[NUMBER_OF_SAMPLES];
float exponentialTimesSamples_M3_I[NUMBER_OF_SAMPLES];

float exponentialTimesSamples_M3_I_Sum = 0;
float exponentialTimesSamples_M3_R_Sum = 0;

float amplitude_squared_M3 = 0;
float phase_M3 = 0;

float COMPLEX_EXPONENTIAL_TABLE_M4_R[NUMBER_OF_SAMPLES];
float COMPLEX_EXPONENTIAL_TABLE_M4_I[NUMBER_OF_SAMPLES];
float exponentialTimesSamples_M4_R[NUMBER_OF_SAMPLES];
float exponentialTimesSamples_M4_I[NUMBER_OF_SAMPLES];

float exponentialTimesSamples_M4_I_Sum = 0;
float exponentialTimesSamples_M4_R_Sum = 0;

float amplitude_squared_M4 = 0;
float phase_M4 = 0;

float COMPLEX_EXPONENTIAL_TABLE_M5_R[NUMBER_OF_SAMPLES];
float COMPLEX_EXPONENTIAL_TABLE_M5_I[NUMBER_OF_SAMPLES];
float exponentialTimesSamples_M5_R[NUMBER_OF_SAMPLES];
float exponentialTimesSamples_M5_I[NUMBER_OF_SAMPLES];

float exponentialTimesSamples_M5_I_Sum = 0;
float exponentialTimesSamples_M5_R_Sum = 0;

float amplitude_squared_M5 = 0;
float phase_M5 = 0;


float outputCommand = 0;

float rampFactor = 0;
int rampSteps = 1*SEC / 100;

// float estimated_amplitude_M1 = 0;
// float estimated_amplitude_M2 = 0;
// float estimated_amplitude_M3 = 0;
// float estimated_amplitude_M4 = 0;
// float estimated_amplitude_M5 = 0;


long testTimingIndex = 0;
int nonAudioTimingIndex = 0;

#define TESTED_GAIN_M1    0.0
#define TESTED_GAIN_M1_S "0.0"
#define TESTED_GAIN_M2    0.0
#define TESTED_GAIN_M2_S "0.0"
#define TESTED_GAIN_M3 	  0.0
#define TESTED_GAIN_M3_S "0.0"
#define TESTED_GAIN_M4    0.0
#define TESTED_GAIN_M4_S "0.0"
#define TESTED_GAIN_M5    0.0
#define TESTED_GAIN_M5_S "0.0"	

float gain_M1 = 0;
float gain_M2 = 0;
float gain_M3 = 0;
float gain_M4 = 0;
float gain_M5 = 0;

float PHASE_OFFSET_M1 = -(1*SEC%NUMBER_OF_SAMPLES) * PHASE_INCREMENT_M1;
float PHASE_OFFSET_M2 = -(1*SEC%NUMBER_OF_SAMPLES) * PHASE_INCREMENT_M2;
float PHASE_OFFSET_M3 = -(1*SEC%NUMBER_OF_SAMPLES) * PHASE_INCREMENT_M3;
float PHASE_OFFSET_M4 = -(1*SEC%NUMBER_OF_SAMPLES) * PHASE_INCREMENT_M4;
float PHASE_OFFSET_M5 = -(1*SEC%NUMBER_OF_SAMPLES) * PHASE_INCREMENT_M5;
// So that the sin starts at its zero point when t = 1*SEC

void writeWAV(float * buffer, int bufferSize, char* path){
    SF_INFO sfinfo ;
    sfinfo.channels = 1;
    sfinfo.samplerate = 44100;
    sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;

    SNDFILE * outfile = sf_open(path, SFM_WRITE, &sfinfo);
    sf_count_t count = sf_write_float(outfile, &buffer[0], bufferSize) ;
    sf_write_sync(outfile);
    sf_close(outfile);
}


bool setup(BelaContext *context, void *userData)
{

    system("mkdir open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S );

    for(unsigned int n = 0; n<NUMBER_OF_SAMPLES; n++){
        COMPLEX_EXPONENTIAL_TABLE_M1_R[n] =  cos(n*PHASE_INCREMENT_M1);
        COMPLEX_EXPONENTIAL_TABLE_M1_I[n] = -sin(n*PHASE_INCREMENT_M1);
        COMPLEX_EXPONENTIAL_TABLE_M2_R[n] =  cos(n*PHASE_INCREMENT_M2);
        COMPLEX_EXPONENTIAL_TABLE_M2_I[n] = -sin(n*PHASE_INCREMENT_M2);
        COMPLEX_EXPONENTIAL_TABLE_M3_R[n] =  cos(n*PHASE_INCREMENT_M3);
        COMPLEX_EXPONENTIAL_TABLE_M3_I[n] = -sin(n*PHASE_INCREMENT_M3);
        COMPLEX_EXPONENTIAL_TABLE_M4_R[n] =  cos(n*PHASE_INCREMENT_M4);
        COMPLEX_EXPONENTIAL_TABLE_M4_I[n] = -sin(n*PHASE_INCREMENT_M4);
        COMPLEX_EXPONENTIAL_TABLE_M5_R[n] =  cos(n*PHASE_INCREMENT_M5);
        COMPLEX_EXPONENTIAL_TABLE_M5_I[n] = -sin(n*PHASE_INCREMENT_M5);
    }

    commandAmplitude_M1_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/commandAmplitude_M1_File.txt"); //set the file name to write to
    commandAmplitude_M1_File << NON_AUDIO_FREQ_S "Hz\n\n";

    // amplitudeObserver_M1_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/amplitudeObserver_M1_File.txt"); //set the file name to write to
    // amplitudeObserver_M1_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitude_M1_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/amplitude_M1_File.txt"); //set the file name to write to
    amplitude_M1_File << NON_AUDIO_FREQ_S "Hz\n\n";

    phase_M1_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/phase_M1_File.txt"); //set the file name to write to
    phase_M1_File << NON_AUDIO_FREQ_S "Hz\n\n";


    commandAmplitude_M2_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/commandAmplitude_M2_File.txt"); //set the file name to write to
    commandAmplitude_M2_File << NON_AUDIO_FREQ_S "Hz\n\n";

    // amplitudeObserver_M2_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/amplitudeObserver_M2_File.txt"); //set the file name to write to
    // amplitudeObserver_M2_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitude_M2_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/amplitude_M2_File.txt"); //set the file name to write to
    amplitude_M2_File << NON_AUDIO_FREQ_S "Hz\n\n";

    phase_M2_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/phase_M2_File.txt"); //set the file name to write to
    phase_M2_File << NON_AUDIO_FREQ_S "Hz\n\n";


    commandAmplitude_M3_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/commandAmplitude_M3_File.txt"); //set the file name to write to
    commandAmplitude_M3_File << NON_AUDIO_FREQ_S "Hz\n\n";

    // amplitudeObserver_M3_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/amplitudeObserver_M3_File.txt"); //set the file name to write to
    // amplitudeObserver_M3_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitude_M3_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/amplitude_M3_File.txt"); //set the file name to write to
    amplitude_M3_File << NON_AUDIO_FREQ_S "Hz\n\n";

    phase_M3_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/phase_M3_File.txt"); //set the file name to write to
    phase_M3_File << NON_AUDIO_FREQ_S "Hz\n\n";


    commandAmplitude_M4_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/commandAmplitude_M4_File.txt"); //set the file name to write to
    commandAmplitude_M4_File << NON_AUDIO_FREQ_S "Hz\n\n";

    // amplitudeObserver_M4_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/amplitudeObserver_M4_File.txt"); //set the file name to write to
    // amplitudeObserver_M4_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitude_M4_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/amplitude_M4_File.txt"); //set the file name to write to
    amplitude_M4_File << NON_AUDIO_FREQ_S "Hz\n\n";

    phase_M4_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/phase_M4_File.txt"); //set the file name to write to
    phase_M4_File << NON_AUDIO_FREQ_S "Hz\n\n";


    commandAmplitude_M5_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/commandAmplitude_M5_File.txt"); //set the file name to write to
    commandAmplitude_M5_File << NON_AUDIO_FREQ_S "Hz\n\n";

    // amplitudeObserver_M5_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/amplitudeObserver_M5_File.txt"); //set the file name to write to
    // amplitudeObserver_M5_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitude_M5_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/amplitude_M5_File.txt"); //set the file name to write to
    amplitude_M5_File << NON_AUDIO_FREQ_S "Hz\n\n";

    phase_M5_File.open("open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/phase_M5_File.txt"); //set the file name to write to
    phase_M5_File << NON_AUDIO_FREQ_S "Hz\n\n";

    rt_printf("Running test");

    return true;
}

void render(BelaContext *context, void *userData)
{

    for(unsigned int n = 0; n < context->audioFrames; n++) {

        

        exponentialTimesSamples_M1_R_Sum -= exponentialTimesSamples_M1_R[sampleIndex];
        exponentialTimesSamples_M1_I_Sum -= exponentialTimesSamples_M1_I[sampleIndex];

        exponentialTimesSamples_M2_R_Sum -= exponentialTimesSamples_M2_R[sampleIndex];
        exponentialTimesSamples_M2_I_Sum -= exponentialTimesSamples_M2_I[sampleIndex];

        exponentialTimesSamples_M3_R_Sum -= exponentialTimesSamples_M3_R[sampleIndex];
        exponentialTimesSamples_M3_I_Sum -= exponentialTimesSamples_M3_I[sampleIndex];

        exponentialTimesSamples_M4_R_Sum -= exponentialTimesSamples_M4_R[sampleIndex];
        exponentialTimesSamples_M4_I_Sum -= exponentialTimesSamples_M4_I[sampleIndex];

        exponentialTimesSamples_M5_R_Sum -= exponentialTimesSamples_M5_R[sampleIndex];
        exponentialTimesSamples_M5_I_Sum -= exponentialTimesSamples_M5_I[sampleIndex];

        latestSample = audioRead(context, n, 0);
        
        exponentialTimesSamples_M1_R[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M1_R[sampleIndex];
        exponentialTimesSamples_M1_I[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M1_I[sampleIndex];
        
        exponentialTimesSamples_M2_R[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M2_R[sampleIndex];
        exponentialTimesSamples_M2_I[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M2_I[sampleIndex];
        
        exponentialTimesSamples_M3_R[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M3_R[sampleIndex];
        exponentialTimesSamples_M3_I[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M3_I[sampleIndex];
        
        exponentialTimesSamples_M4_R[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M4_R[sampleIndex];
        exponentialTimesSamples_M4_I[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M4_I[sampleIndex];
        
        exponentialTimesSamples_M5_R[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M5_R[sampleIndex];
        exponentialTimesSamples_M5_I[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M5_I[sampleIndex];
        
        
        exponentialTimesSamples_M1_R_Sum += exponentialTimesSamples_M1_R[sampleIndex];
        exponentialTimesSamples_M1_I_Sum += exponentialTimesSamples_M1_I[sampleIndex];

        exponentialTimesSamples_M2_R_Sum += exponentialTimesSamples_M2_R[sampleIndex];
        exponentialTimesSamples_M2_I_Sum += exponentialTimesSamples_M2_I[sampleIndex];

        exponentialTimesSamples_M3_R_Sum += exponentialTimesSamples_M3_R[sampleIndex];
        exponentialTimesSamples_M3_I_Sum += exponentialTimesSamples_M3_I[sampleIndex];

        exponentialTimesSamples_M4_R_Sum += exponentialTimesSamples_M4_R[sampleIndex];
        exponentialTimesSamples_M4_I_Sum += exponentialTimesSamples_M4_I[sampleIndex];

        exponentialTimesSamples_M5_R_Sum += exponentialTimesSamples_M5_R[sampleIndex];
        exponentialTimesSamples_M5_I_Sum += exponentialTimesSamples_M5_I[sampleIndex];

        amplitude_squared_M1 = (exponentialTimesSamples_M1_R_Sum*exponentialTimesSamples_M1_R_Sum + exponentialTimesSamples_M1_I_Sum*exponentialTimesSamples_M1_I_Sum)*ONE_OVER_NUMBER_OF_SAMPLES_SQUARED;
        amplitude_squared_M2 = (exponentialTimesSamples_M2_R_Sum*exponentialTimesSamples_M2_R_Sum + exponentialTimesSamples_M2_I_Sum*exponentialTimesSamples_M2_I_Sum)*ONE_OVER_NUMBER_OF_SAMPLES_SQUARED;
        amplitude_squared_M3 = (exponentialTimesSamples_M3_R_Sum*exponentialTimesSamples_M3_R_Sum + exponentialTimesSamples_M3_I_Sum*exponentialTimesSamples_M3_I_Sum)*ONE_OVER_NUMBER_OF_SAMPLES_SQUARED;
        amplitude_squared_M4 = (exponentialTimesSamples_M4_R_Sum*exponentialTimesSamples_M4_R_Sum + exponentialTimesSamples_M4_I_Sum*exponentialTimesSamples_M4_I_Sum)*ONE_OVER_NUMBER_OF_SAMPLES_SQUARED;
        amplitude_squared_M5 = (exponentialTimesSamples_M5_R_Sum*exponentialTimesSamples_M5_R_Sum + exponentialTimesSamples_M5_I_Sum*exponentialTimesSamples_M5_I_Sum)*ONE_OVER_NUMBER_OF_SAMPLES_SQUARED;
        
        phase_M1 = atan2(exponentialTimesSamples_M1_I_Sum,exponentialTimesSamples_M1_R_Sum);
        phase_M2 = atan2(exponentialTimesSamples_M2_I_Sum,exponentialTimesSamples_M2_R_Sum);
        phase_M3 = atan2(exponentialTimesSamples_M3_I_Sum,exponentialTimesSamples_M3_R_Sum);
        phase_M4 = atan2(exponentialTimesSamples_M4_I_Sum,exponentialTimesSamples_M4_R_Sum);
        phase_M5 = atan2(exponentialTimesSamples_M5_I_Sum,exponentialTimesSamples_M5_R_Sum);


        sampleIndex = (sampleIndex + 1)%NUMBER_OF_SAMPLES;

        //Observer


        // estimated_amplitude_M1 = estimated_amplitude_M1*DAMPING_FACTOR - OBSERVER_FACTOR*(estimated_amplitude_M1*DAMPING_FACTOR - sqrtf_neon(amplitude_squared_M1));
        // estimated_amplitude_M2 = estimated_amplitude_M2*DAMPING_FACTOR - OBSERVER_FACTOR*(estimated_amplitude_M2*DAMPING_FACTOR - sqrtf_neon(amplitude_squared_M2));
        // estimated_amplitude_M3 = estimated_amplitude_M3*DAMPING_FACTOR - OBSERVER_FACTOR*(estimated_amplitude_M3*DAMPING_FACTOR - sqrtf_neon(amplitude_squared_M3));
        // estimated_amplitude_M4 = estimated_amplitude_M4*DAMPING_FACTOR - OBSERVER_FACTOR*(estimated_amplitude_M4*DAMPING_FACTOR - sqrtf_neon(amplitude_squared_M4));
        // estimated_amplitude_M5 = estimated_amplitude_M5*DAMPING_FACTOR - OBSERVER_FACTOR*(estimated_amplitude_M5*DAMPING_FACTOR - sqrtf_neon(amplitude_squared_M5));

        


        if(testTimingIndex<1*SEC){ 
            gain_M1 = 0;
            gain_M2 = 0;
            gain_M3 = 0;
            gain_M4 = 0;
            gain_M5 = 0;
        }
        else{
            if(testTimingIndex<DURATION){
                if(testTimingIndex-1*SEC<rampSteps){
                    rampFactor = (testTimingIndex-1*SEC)*1.0/rampSteps;
                    gain_M1 = rampFactor*TESTED_GAIN_M1;
                    gain_M2 = rampFactor*TESTED_GAIN_M2;
                    gain_M3 = rampFactor*TESTED_GAIN_M3;
                    gain_M4 = rampFactor*TESTED_GAIN_M4;
                    gain_M5 = rampFactor*TESTED_GAIN_M5;
                }
                else{
                    gain_M1 = TESTED_GAIN_M1;
                    gain_M2 = TESTED_GAIN_M2;
                    gain_M3 = TESTED_GAIN_M3;
                    gain_M4 = TESTED_GAIN_M4;
                    gain_M5 = TESTED_GAIN_M5;
                }
            }
            else{
                gShouldStop = true;
            }
        }



        outputCommand =  gain_M1  *sin(sampleIndex*PHASE_INCREMENT_M1 + PHASE_OFFSET_M1)
                      +  gain_M2  *sin(sampleIndex*PHASE_INCREMENT_M2 + PHASE_OFFSET_M2)
                      +  gain_M3  *sin(sampleIndex*PHASE_INCREMENT_M3 + PHASE_OFFSET_M3)
                      +  gain_M4  *sin(sampleIndex*PHASE_INCREMENT_M4 + PHASE_OFFSET_M4)
                      +  gain_M5  *sin(sampleIndex*PHASE_INCREMENT_M5 + PHASE_OFFSET_M5)
                        ;


        audioArray[testTimingIndex] = latestSample;
        commandSignalArray[testTimingIndex] = outputCommand;

        audioWrite(context, n, 0, outputCommand);
        audioWrite(context, n, 1, outputCommand);





        if(testTimingIndex%NON_AUDIO_FREQ == 0){

            commandAmplitude_M1_Array[nonAudioTimingIndex] = gain_M1;
            // amplitudeObserver_M1_Array[nonAudioTimingIndex] = estimated_amplitude_M1;
            amplitude_M1_Array[nonAudioTimingIndex] = sqrtf_neon(amplitude_squared_M1);
            phase_M1_Array[nonAudioTimingIndex] = phase_M1;

            commandAmplitude_M2_Array[nonAudioTimingIndex] = gain_M2;
            // amplitudeObserver_M2_Array[nonAudioTimingIndex] = estimated_amplitude_M2;
            amplitude_M2_Array[nonAudioTimingIndex] = sqrtf_neon(amplitude_squared_M2);
            phase_M2_Array[nonAudioTimingIndex] = phase_M2;

            commandAmplitude_M3_Array[nonAudioTimingIndex] = gain_M3;
            // amplitudeObserver_M3_Array[nonAudioTimingIndex] = estimated_amplitude_M3;
            amplitude_M3_Array[nonAudioTimingIndex] = sqrtf_neon(amplitude_squared_M3);
            phase_M3_Array[nonAudioTimingIndex] = phase_M3;

            commandAmplitude_M4_Array[nonAudioTimingIndex] = gain_M4;
            // amplitudeObserver_M4_Array[nonAudioTimingIndex] = estimated_amplitude_M4;
            amplitude_M4_Array[nonAudioTimingIndex] = sqrtf_neon(amplitude_squared_M4);
            phase_M4_Array[nonAudioTimingIndex] = phase_M4;

            commandAmplitude_M5_Array[nonAudioTimingIndex] = gain_M5;
            // amplitudeObserver_M5_Array[nonAudioTimingIndex] = estimated_amplitude_M5;
            amplitude_M5_Array[nonAudioTimingIndex] = sqrtf_neon(amplitude_squared_M5);
            phase_M5_Array[nonAudioTimingIndex] = phase_M5;
            nonAudioTimingIndex++;
        }

        testTimingIndex = testTimingIndex+1;


    }

    
}

void cleanup(BelaContext *context, void *userData)
{
    
    rt_printf("Saving");


    writeWAV(audioArray, DURATION, "open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/audioFile.wav");
    writeWAV(commandSignalArray, DURATION, "open_loop_response_ramp_" TESTED_GAIN_M1_S "_" TESTED_GAIN_M2_S "_" TESTED_GAIN_M3_S "_" TESTED_GAIN_M4_S "_" TESTED_GAIN_M5_S "/commandSignalFile.wav");




    rt_printf("Audio OK\n");
    


    for(int i=0; i<nonAudioTimingIndex; i++){
        commandAmplitude_M1_File << commandAmplitude_M1_Array[i] << endl;
        // amplitudeObserver_M1_File << amplitudeObserver_M1_Array[i] << endl;
        amplitude_M1_File << amplitude_M1_Array[i] << endl;
        phase_M1_File << phase_M1_Array[i] << endl;
    }
    commandAmplitude_M1_File.close();
    // amplitudeObserver_M1_File.close();
    amplitude_M1_File.close();
    phase_M1_File.close();

    rt_printf("M1 OK\n");

    for(int i=0; i<nonAudioTimingIndex; i++){
        commandAmplitude_M2_File << commandAmplitude_M2_Array[i] << endl;
        // amplitudeObserver_M2_File << amplitudeObserver_M2_Array[i] << endl;
        amplitude_M2_File << amplitude_M2_Array[i] << endl;
        phase_M2_File << phase_M2_Array[i] << endl;
    }
    commandAmplitude_M2_File.close();
    // amplitudeObserver_M2_File.close();
    amplitude_M2_File.close();
    phase_M2_File.close();

    rt_printf("M2 OK\n");

    for(int i=0; i<nonAudioTimingIndex; i++){
        commandAmplitude_M3_File << commandAmplitude_M3_Array[i] << endl;
        // amplitudeObserver_M3_File << amplitudeObserver_M3_Array[i] << endl;
        amplitude_M3_File << amplitude_M3_Array[i] << endl;
        phase_M3_File << phase_M3_Array[i] << endl;
    }
    commandAmplitude_M3_File.close();
    // amplitudeObserver_M3_File.close();
    amplitude_M3_File.close();
    phase_M3_File.close();

    rt_printf("M3 OK\n");

    for(int i=0; i<nonAudioTimingIndex; i++){
        commandAmplitude_M4_File << commandAmplitude_M4_Array[i] << endl;
        // amplitudeObserver_M4_File << amplitudeObserver_M4_Array[i] << endl;
        amplitude_M4_File << amplitude_M4_Array[i] << endl;
        phase_M4_File << phase_M4_Array[i] << endl;
    }
    commandAmplitude_M4_File.close();
    // amplitudeObserver_M4_File.close();
    amplitude_M4_File.close();
    phase_M4_File.close();

    rt_printf("M4 OK\n");

    for(int i=0; i<nonAudioTimingIndex; i++){
        commandAmplitude_M5_File << commandAmplitude_M5_Array[i] << endl;
        // amplitudeObserver_M5_File << amplitudeObserver_M5_Array[i] << endl;
        amplitude_M5_File << amplitude_M5_Array[i] << endl;
        phase_M5_File << phase_M5_Array[i] << endl;
    }
    commandAmplitude_M5_File.close();
    // amplitudeObserver_M5_File.close();
    amplitude_M5_File.close();
    phase_M5_File.close();
    
    rt_printf("M5 OK\n");

}

