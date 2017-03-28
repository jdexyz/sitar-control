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

#define xstr(a) str(a)
#define str(a) #a

#include <Bela.h>
#include <array>
#include <cmath>
#include <iostream>
#include <math_neon.h>
#include <WriteFile.h>
#include <sndfile.h>

#include <fstream>
using namespace std;


#define NUMBER_OF_AVERAGING_PHASES 100
#define NUMBER_OF_AVERAGING_AMPLITUDES 300
#define NUMBER_OF_AVERAGING_DERIVATIVES 150


const float ONE_OVER_NUMBER_OF_AVERAGING_PHASES = 1.0/NUMBER_OF_AVERAGING_PHASES;
const float ONE_OVER_NUMBER_OF_AVERAGING_AMPLITUDES = 1.0/NUMBER_OF_AVERAGING_AMPLITUDES;
const float ONE_OVER_NUMBER_OF_AVERAGING_DERIVATIVES = 1.0/NUMBER_OF_AVERAGING_DERIVATIVES;

float audioArray[DURATION*2];
float commandSignalArray[DURATION*2];
float outputCurrentArray[DURATION*2];
ofstream commandSignalFile;
ofstream outputCurrentFile;


ofstream commandAmplitude_M1_File;
ofstream target_M1_File;
ofstream amplitudeObserver_M1_File;
ofstream amplitude_M1_File;
ofstream phase_M1_File;
float commandAmplitude_M1_Array[DURATION];
float target_M1_Array[DURATION];
float amplitudeObserver_M1_Array[DURATION];
float amplitude_M1_Array[DURATION];
float phase_M1_Array[DURATION];

ofstream commandAmplitude_M2_File;
ofstream target_M2_File;
ofstream amplitudeObserver_M2_File;
ofstream amplitude_M2_File;
ofstream phase_M2_File;
float commandAmplitude_M2_Array[DURATION];
float target_M2_Array[DURATION];
float amplitudeObserver_M2_Array[DURATION];
float amplitude_M2_Array[DURATION];
float phase_M2_Array[DURATION];

ofstream commandAmplitude_M3_File;
ofstream target_M3_File;
ofstream amplitudeObserver_M3_File;
ofstream amplitude_M3_File;
ofstream phase_M3_File;
float commandAmplitude_M3_Array[DURATION];
float target_M3_Array[DURATION];
float amplitudeObserver_M3_Array[DURATION];
float amplitude_M3_Array[DURATION];
float phase_M3_Array[DURATION];

ofstream commandAmplitude_M4_File;
ofstream target_M4_File;
ofstream amplitudeObserver_M4_File;
ofstream amplitude_M4_File;
ofstream phase_M4_File;
float commandAmplitude_M4_Array[DURATION];
float target_M4_Array[DURATION];
float amplitudeObserver_M4_Array[DURATION];
float amplitude_M4_Array[DURATION];
float phase_M4_Array[DURATION];

ofstream commandAmplitude_M5_File;
ofstream target_M5_File;
ofstream amplitudeObserver_M5_File;
ofstream amplitude_M5_File;
ofstream phase_M5_File;
float commandAmplitude_M5_Array[DURATION];
float target_M5_Array[DURATION];
float amplitudeObserver_M5_Array[DURATION];
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

const float PHASE_OFFSET_M1 = - NUMBER_OF_AVERAGING_PHASES*PHASE_INCREMENT_M1*0.5;
const float PHASE_OFFSET_M2 = - NUMBER_OF_AVERAGING_PHASES*PHASE_INCREMENT_M2*0.5;
const float PHASE_OFFSET_M3 = 0;
const float PHASE_OFFSET_M4 = 0;
const float PHASE_OFFSET_M5 = 0;



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


long testTimingIndex = 0;
int nonAudioTimingIndex = 0;


float gain_M1 = 0;
float gain_M2 = 0;
float gain_M3 = 0;
float gain_M4 = 0;
float gain_M5 = 0;

float drivingSyncedSine_M1 = 0;
float drivingSyncedSine_M2 = 0;
float drivingSyncedSine_M3 = 0;
float drivingSyncedSine_M4 = 0;
float drivingSyncedSine_M5 = 0;

float phaseAverageArray_M1[NUMBER_OF_AVERAGING_PHASES];
float phaseAverageArray_M2[NUMBER_OF_AVERAGING_PHASES];
float phaseAverageArray_M3[NUMBER_OF_AVERAGING_PHASES];
float phaseAverageArray_M4[NUMBER_OF_AVERAGING_PHASES];
float phaseAverageArray_M5[NUMBER_OF_AVERAGING_PHASES];

float amplitudeAverageArray_M1[NUMBER_OF_AVERAGING_AMPLITUDES];
float amplitudeAverageArray_M2[NUMBER_OF_AVERAGING_AMPLITUDES];
float amplitudeAverageArray_M3[NUMBER_OF_AVERAGING_AMPLITUDES];
float amplitudeAverageArray_M4[NUMBER_OF_AVERAGING_AMPLITUDES];
float amplitudeAverageArray_M5[NUMBER_OF_AVERAGING_AMPLITUDES];

float phaseAverageSum_M1 = 0;
float phaseAverageSum_M2 = 0;
float phaseAverageSum_M3 = 0;
float phaseAverageSum_M4 = 0;
float phaseAverageSum_M5 = 0;

float amplitudeAverageArray_M1_Sum = 0;
float amplitudeAverageArray_M2_Sum = 0;
float amplitudeAverageArray_M3_Sum = 0;
float amplitudeAverageArray_M4_Sum = 0;
float amplitudeAverageArray_M5_Sum = 0;


float ratioArray[128];
float ratioArray_Sum = 0;
int ratioIndex = 0;

int sign_I = 1;

#define Z_COEFF  0.101617
#define ALPHA    12.013104
#define Z_OFFSET 6.012744
#define I_FACTOR -0.596115

#define D1 -0.554466
#define D2 1.896533 
#define N_ESSAI "1"

float B_0 = - 41.6042;
float k_0 = 0.0748411;
float b_I = 62.4368;
float k_I = 0.18605;

//float D1 =  0.8;
//float D2 =  2;
//
//float Z_COEFF =   0.110;
//float ALPHA   =   100.0;
//float Z_OFFSET =  6;
//float I_FACTOR =  1.0/2.34;
int averagingPhasesIndex = 0;
int averagingAmplitudesIndex = 0;

float z = 6;
float I = 0;


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

    system("mkdir Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI);


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

    commandSignalFile.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/commandSignalFile.txt");
    commandSignalFile << "44100Hz\n\n";

    outputCurrentFile.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/outputCurrentFile.txt");
    outputCurrentFile << "44100Hz\n\n";


    commandAmplitude_M1_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/commandAmplitude_M1_File.txt");
    commandAmplitude_M1_File << NON_AUDIO_FREQ_S "Hz\n\n";

    target_M1_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/target_M1_File.txt");
    target_M1_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitudeObserver_M1_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/amplitudeObserver_M1_File.txt");
    amplitudeObserver_M1_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitude_M1_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/amplitude_M1_File.txt");
    amplitude_M1_File << NON_AUDIO_FREQ_S "Hz\n\n";

    phase_M1_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/phase_M1_File.txt");
    phase_M1_File << NON_AUDIO_FREQ_S "Hz\n\n";


    commandAmplitude_M2_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/commandAmplitude_M2_File.txt");
    commandAmplitude_M2_File << NON_AUDIO_FREQ_S "Hz\n\n";

    target_M2_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/target_M2_File.txt");
    target_M2_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitudeObserver_M2_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/amplitudeObserver_M2_File.txt");
    amplitudeObserver_M2_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitude_M2_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/amplitude_M2_File.txt");
    amplitude_M2_File << NON_AUDIO_FREQ_S "Hz\n\n";

    phase_M2_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/phase_M2_File.txt");
    phase_M2_File << NON_AUDIO_FREQ_S "Hz\n\n";


    commandAmplitude_M3_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/commandAmplitude_M3_File.txt");
    commandAmplitude_M3_File << NON_AUDIO_FREQ_S "Hz\n\n";

    target_M3_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/target_M3_File.txt");
    target_M3_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitudeObserver_M3_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/amplitudeObserver_M3_File.txt");
    amplitudeObserver_M3_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitude_M3_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/amplitude_M3_File.txt");
    amplitude_M3_File << NON_AUDIO_FREQ_S "Hz\n\n";

    phase_M3_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/phase_M3_File.txt");
    phase_M3_File << NON_AUDIO_FREQ_S "Hz\n\n";


    commandAmplitude_M4_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/commandAmplitude_M4_File.txt");
    commandAmplitude_M4_File << NON_AUDIO_FREQ_S "Hz\n\n";

    target_M4_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/target_M4_File.txt");
    target_M4_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitudeObserver_M4_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/amplitudeObserver_M4_File.txt");
    amplitudeObserver_M4_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitude_M4_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/amplitude_M4_File.txt");
    amplitude_M4_File << NON_AUDIO_FREQ_S "Hz\n\n";

    phase_M4_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/phase_M4_File.txt");
    phase_M4_File << NON_AUDIO_FREQ_S "Hz\n\n";


    commandAmplitude_M5_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/commandAmplitude_M5_File.txt");
    commandAmplitude_M5_File << NON_AUDIO_FREQ_S "Hz\n\n";

    target_M5_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/target_M5_File.txt");
    target_M5_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitudeObserver_M5_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/amplitudeObserver_M5_File.txt");
    amplitudeObserver_M5_File << NON_AUDIO_FREQ_S "Hz\n\n";

    amplitude_M5_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/amplitude_M5_File.txt");
    amplitude_M5_File << NON_AUDIO_FREQ_S "Hz\n\n";

    phase_M5_File.open("Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/phase_M5_File.txt");
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

        // exponentialTimesSamples_M3_R_Sum -= exponentialTimesSamples_M3_R[sampleIndex];
        // exponentialTimesSamples_M3_I_Sum -= exponentialTimesSamples_M3_I[sampleIndex];

        // exponentialTimesSamples_M4_R_Sum -= exponentialTimesSamples_M4_R[sampleIndex];
        // exponentialTimesSamples_M4_I_Sum -= exponentialTimesSamples_M4_I[sampleIndex];

        // exponentialTimesSamples_M5_R_Sum -= exponentialTimesSamples_M5_R[sampleIndex];
        // exponentialTimesSamples_M5_I_Sum -= exponentialTimesSamples_M5_I[sampleIndex];

        latestSample = audioRead(context, n, 0);
        
        exponentialTimesSamples_M1_R[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M1_R[sampleIndex];
        exponentialTimesSamples_M1_I[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M1_I[sampleIndex];
        
        exponentialTimesSamples_M2_R[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M2_R[sampleIndex];
        exponentialTimesSamples_M2_I[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M2_I[sampleIndex];
        
        // exponentialTimesSamples_M3_R[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M3_R[sampleIndex];
        // exponentialTimesSamples_M3_I[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M3_I[sampleIndex];
        
        // exponentialTimesSamples_M4_R[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M4_R[sampleIndex];
        // exponentialTimesSamples_M4_I[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M4_I[sampleIndex];
        
        // exponentialTimesSamples_M5_R[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M5_R[sampleIndex];
        // exponentialTimesSamples_M5_I[sampleIndex] = latestSample*COMPLEX_EXPONENTIAL_TABLE_M5_I[sampleIndex];
        
        
        exponentialTimesSamples_M1_R_Sum += exponentialTimesSamples_M1_R[sampleIndex];
        exponentialTimesSamples_M1_I_Sum += exponentialTimesSamples_M1_I[sampleIndex];

        exponentialTimesSamples_M2_R_Sum += exponentialTimesSamples_M2_R[sampleIndex];
        exponentialTimesSamples_M2_I_Sum += exponentialTimesSamples_M2_I[sampleIndex];

        // exponentialTimesSamples_M3_R_Sum += exponentialTimesSamples_M3_R[sampleIndex];
        // exponentialTimesSamples_M3_I_Sum += exponentialTimesSamples_M3_I[sampleIndex];

        // exponentialTimesSamples_M4_R_Sum += exponentialTimesSamples_M4_R[sampleIndex];
        // exponentialTimesSamples_M4_I_Sum += exponentialTimesSamples_M4_I[sampleIndex];

        // exponentialTimesSamples_M5_R_Sum += exponentialTimesSamples_M5_R[sampleIndex];
        // exponentialTimesSamples_M5_I_Sum += exponentialTimesSamples_M5_I[sampleIndex];

        amplitude_squared_M1 = (exponentialTimesSamples_M1_R_Sum*exponentialTimesSamples_M1_R_Sum + exponentialTimesSamples_M1_I_Sum*exponentialTimesSamples_M1_I_Sum)*ONE_OVER_NUMBER_OF_SAMPLES_SQUARED;
        amplitude_squared_M2 = (exponentialTimesSamples_M2_R_Sum*exponentialTimesSamples_M2_R_Sum + exponentialTimesSamples_M2_I_Sum*exponentialTimesSamples_M2_I_Sum)*ONE_OVER_NUMBER_OF_SAMPLES_SQUARED;
        // amplitude_squared_M3 = (exponentialTimesSamples_M3_R_Sum*exponentialTimesSamples_M3_R_Sum + exponentialTimesSamples_M3_I_Sum*exponentialTimesSamples_M3_I_Sum)*ONE_OVER_NUMBER_OF_SAMPLES_SQUARED;
        // amplitude_squared_M4 = (exponentialTimesSamples_M4_R_Sum*exponentialTimesSamples_M4_R_Sum + exponentialTimesSamples_M4_I_Sum*exponentialTimesSamples_M4_I_Sum)*ONE_OVER_NUMBER_OF_SAMPLES_SQUARED;
        // amplitude_squared_M5 = (exponentialTimesSamples_M5_R_Sum*exponentialTimesSamples_M5_R_Sum + exponentialTimesSamples_M5_I_Sum*exponentialTimesSamples_M5_I_Sum)*ONE_OVER_NUMBER_OF_SAMPLES_SQUARED;

        phase_M1 =  (amplitude_squared_M1>0.1) ? atan2(exponentialTimesSamples_M1_I_Sum,exponentialTimesSamples_M1_R_Sum) : 0;
        phase_M2 =  (amplitude_squared_M2>0.1) ? atan2(exponentialTimesSamples_M2_I_Sum,exponentialTimesSamples_M2_R_Sum) : 0;
        // phase_M3 =  (amplitude_squared_M3>0.1) ? atan2(exponentialTimesSamples_M3_I_Sum,exponentialTimesSamples_M3_R_Sum) : 0;
        // phase_M4 =  (amplitude_squared_M4>0.1) ? atan2(exponentialTimesSamples_M4_I_Sum,exponentialTimesSamples_M4_R_Sum) : 0;
        // phase_M5 =  (amplitude_squared_M5>0.1) ? atan2(exponentialTimesSamples_M5_I_Sum,exponentialTimesSamples_M5_R_Sum) : 0;


        sampleIndex = (sampleIndex + 1)%NUMBER_OF_SAMPLES;

        // Smoothing the phase with a moving average

        phaseAverageSum_M1 -= phaseAverageArray_M1[averagingPhasesIndex];
        phaseAverageArray_M1[averagingPhasesIndex] = sin(sampleIndex*PHASE_INCREMENT_M1 - PI/2 + phase_M1);// - NUMBER_OF_AVERAGING_PHASES*PI*FREQUENCY_OF_INTEREST_M1/SAMPLING_FREQUENCY);
        phaseAverageSum_M1 += sin(sampleIndex*PHASE_INCREMENT_M1 - PI/2 + phase_M1);

        phaseAverageSum_M2 -= phaseAverageArray_M2[averagingPhasesIndex];
        phaseAverageArray_M2[averagingPhasesIndex] = sin(sampleIndex*PHASE_INCREMENT_M2 - PI/2 + phase_M2);// - NUMBER_OF_AVERAGING_PHASES*PI*FREQUENCY_OF_INTEREST_M2/SAMPLING_FREQUENCY);
        phaseAverageSum_M2 += sin(sampleIndex*PHASE_INCREMENT_M2 - PI/2 + phase_M2);

        // phaseAverageSum_M3 -= phaseAverageArray_M3[averagingPhasesIndex];
        // phaseAverageArray_M3[averagingPhasesIndex] = sin(sampleIndex*PHASE_INCREMENT_M3 - PI/2 + phase_M3);
        // phaseAverageSum_M3 += sin(sampleIndex*PHASE_INCREMENT_M3 - PI/2 + phase_M3);

        // phaseAverageSum_M4 -= phaseAverageArray_M4[averagingPhasesIndex];
        // phaseAverageArray_M4[averagingPhasesIndex] = sin(sampleIndex*PHASE_INCREMENT_M4 - PI/2 + phase_M4);
        // phaseAverageSum_M4 += sin(sampleIndex*PHASE_INCREMENT_M4 - PI/2 + phase_M4);

        // phaseAverageSum_M5 -= phaseAverageArray_M5[averagingPhasesIndex];
        // phaseAverageArray_M5[averagingPhasesIndex] = sin(sampleIndex*PHASE_INCREMENT_M5 - PI/2 + phase_M5);
        // phaseAverageSum_M5 += sin(sampleIndex*PHASE_INCREMENT_M5 - PI/2 + phase_M5);

        averagingPhasesIndex = (averagingPhasesIndex+1)%NUMBER_OF_AVERAGING_PHASES;


        amplitudeAverageArray_M1_Sum -= amplitudeAverageArray_M1[averagingAmplitudesIndex];
        amplitudeAverageArray_M1[averagingAmplitudesIndex] = sqrt(amplitude_squared_M1);
        amplitudeAverageArray_M1_Sum += amplitudeAverageArray_M1[averagingAmplitudesIndex];

        amplitudeAverageArray_M2_Sum -= amplitudeAverageArray_M2[averagingAmplitudesIndex];
        amplitudeAverageArray_M2[averagingAmplitudesIndex] = sqrt(amplitude_squared_M2);
        amplitudeAverageArray_M2_Sum += amplitudeAverageArray_M2[averagingAmplitudesIndex];

        // amplitudeAverageArray_M3_Sum -= amplitudeAverageArray_M3[averagingAmplitudesIndex];
        // amplitudeAverageArray_M3[averagingAmplitudesIndex] = sqrt(amplitude_squared_M3);
        // amplitudeAverageArray_M3_Sum += amplitudeAverageArray_M3[averagingAmplitudesIndex];

        // amplitudeAverageArray_M4_Sum -= amplitudeAverageArray_M4[averagingAmplitudesIndex];
        // amplitudeAverageArray_M4[averagingAmplitudesIndex] = sqrt(amplitude_squared_M4);
        // amplitudeAverageArray_M4_Sum += amplitudeAverageArray_M4[averagingAmplitudesIndex];

        // amplitudeAverageArray_M5_Sum -= amplitudeAverageArray_M5[averagingAmplitudesIndex];
        // amplitudeAverageArray_M5[averagingAmplitudesIndex] = sqrt(amplitude_squared_M5);
        // amplitudeAverageArray_M5_Sum += amplitudeAverageArray_M5[averagingAmplitudesIndex];

        averagingAmplitudesIndex = (averagingAmplitudesIndex+1)%NUMBER_OF_AVERAGING_AMPLITUDES;



        // derivativeAverageSum_M1 -= derivativeAverageArray_M1[averagingDerivativesIndex];
        // derivativeAverageArray_M1[averagingDerivativesIndex] = (amplitudeAverageArray_M1[averagingAmplitudesIndex] - amplitudeAverageArray_M1[averagingAmplitudesIndex-10]);
        // derivativeAverageSum_M1 += derivativeAverageArray_M1[averagingDerivativesIndex];

        // derivativeAverageSum_M2 -= derivativeAverageArray_M2[averagingDerivativesIndex];
        // derivativeAverageArray_M2[averagingDerivativesIndex] = sqrt(derivative_squared_M2);
        // derivativeAverageSum_M2 += sqrt(derivative_squared_M2);

        // derivativeAverageSum_M3 -= derivativeAverageArray_M3[averagingDerivativesIndex];
        // derivativeAverageArray_M3[averagingDerivativesIndex] = sqrt(derivative_squared_M3);
        // derivativeAverageSum_M3 += sqrt(derivative_squared_M3);

        // derivativeAverageSum_M4 -= derivativeAverageArray_M4[averagingDerivativesIndex];
        // derivativeAverageArray_M4[averagingDerivativesIndex] = sqrt(derivative_squared_M4);
        // derivativeAverageSum_M4 += sqrt(derivative_squared_M4);

        // derivativeAverageSum_M5 -= derivativeAverageArray_M5[averagingDerivativesIndex];
        // derivativeAverageArray_M5[averagingDerivativesIndex] = sqrt(derivative_squared_M5);
        // derivativeAverageSum_M5 += sqrt(derivative_squared_M5);

        // averagingDerivativesIndex = (averagingDerivativesIndex+1)%NUMBER_OF_AVERAGING_DERIVATIVES;


        // gain_M1 = PROPORTIONAL_GAIN_M1*(target_M1 - (amplitudeAverageArray_M1_Sum*ONE_OVER_NUMBER_OF_AVERAGING_AMPLITUDES));
        // gain_M1 = (abs(gain_M1) > 1) ? ((0 < gain_M1) - (gain_M1 < 0))*1 : gain_M1;
        // gain_M2 = -PROPORTIONAL_GAIN_M2*(target_M2 - (amplitudeAverageArray_M2_Sum*ONE_OVER_NUMBER_OF_AVERAGING_AMPLITUDES));
        // gain_M2 = (abs(gain_M2) > 1) ? ((0 < gain_M2) - (gain_M2 < 0))*1 : gain_M2;
        //gain_M3 = PROPORTIONAL_GAIN_M3*((amplitudeAverageArray_M3_Sum*ONE_OVER_NUMBER_OF_AVERAGING_AMPLITUDES) - target_M3);
        //gain_M4 = PROPORTIONAL_GAIN_M4*((amplitudeAverageArray_M4_Sum*ONE_OVER_NUMBER_OF_AVERAGING_AMPLITUDES) - target_M4);
        //gain_M5 = PROPORTIONAL_GAIN_M5*((amplitudeAverageArray_M5_Sum*ONE_OVER_NUMBER_OF_AVERAGING_AMPLITUDES) - target_M5);



        // drivingSyncedSine_M1 = +sin(sampleIndex*PHASE_INCREMENT_M1 - PI/2 + phaseAverageSum_M1*ONE_OVER_NUMBER_OF_AVERAGING_PHASES);
        drivingSyncedSine_M1 = phaseAverageSum_M1*ONE_OVER_NUMBER_OF_AVERAGING_PHASES;
        // drivingSyncedSine_M2 = +sin(sampleIndex*PHASE_INCREMENT_M2 - PI/2 + phaseAverageSum_M2*ONE_OVER_NUMBER_OF_AVERAGING_PHASES);
        drivingSyncedSine_M2 = phaseAverageSum_M2*ONE_OVER_NUMBER_OF_AVERAGING_PHASES;
        // drivingSyncedSine_M3 = +sin(sampleIndex*PHASE_INCREMENT_M3 - PI/2 + phaseAverageSum_M3*ONE_OVER_NUMBER_OF_AVERAGING_PHASES);
        // drivingSyncedSine_M3 = phaseAverageSum_M3*ONE_OVER_NUMBER_OF_AVERAGING_PHASES;
        // // drivingSyncedSine_M4 = +sin(sampleIndex*PHASE_INCREMENT_M4 - PI/2 + phaseAverageSum_M4*ONE_OVER_NUMBER_OF_AVERAGING_PHASES);
        // drivingSyncedSine_M4 = phaseAverageSum_M4*ONE_OVER_NUMBER_OF_AVERAGING_PHASES;
        // // drivingSyncedSine_M5 = +sin(sampleIndex*PHASE_INCREMENT_M5 - PI/2 + phaseAverageSum_M5*ONE_OVER_NUMBER_OF_AVERAGING_PHASES);
        // drivingSyncedSine_M5 = phaseAverageSum_M5*ONE_OVER_NUMBER_OF_AVERAGING_PHASES;





        outputCommand =  1 * drivingSyncedSine_M1; //0.3*sin(sampleIndex*PHASE_INCREMENT_M1);
                         // +  1 * drivingSyncedSine_M2
                         // +  gain_M3 * drivingSyncedSine_M3
                         // +  gain_M4 * drivingSyncedSine_M4
                         // +  gain_M5 * drivingSyncedSine_M5
                            //;

        //  I = \frac{\alpha f - k_0 B_0^2 \exp(-2k_0 z)} {B_0 b_I(k_0+k_I)\exp(-(k_0+k_I)z)} 
        z = Z_COEFF*(D1*(amplitudeAverageArray_M1_Sum*ONE_OVER_NUMBER_OF_AVERAGING_AMPLITUDES)*drivingSyncedSine_M1 + D2*(amplitudeAverageArray_M2_Sum*ONE_OVER_NUMBER_OF_AVERAGING_AMPLITUDES)*drivingSyncedSine_M2) + Z_OFFSET;

        
        I = I_FACTOR*(ALPHA*outputCommand + k_0*B_0*B_0*exp(-2*k_0*(Z_OFFSET)) - k_0*B_0*B_0*exp(-2*k_0*z)) / (B_0*b_I*(k_0+k_I)*exp(-(k_0+k_I)*z));


        audioArray[testTimingIndex] = latestSample;
        commandSignalArray[testTimingIndex] = 0.1*outputCommand;
        outputCurrentArray[testTimingIndex] = 0.1*I;


        audioWrite(context, n, 0, I);
        audioWrite(context, n, 1, I);

        ratioArray_Sum -= ratioArray[ratioIndex];
        ratioArray[ratioIndex] = amplitudeAverageArray_M2_Sum/amplitudeAverageArray_M1_Sum;
        ratioArray_Sum += amplitudeAverageArray_M2_Sum/amplitudeAverageArray_M1_Sum;
        ratioIndex = (ratioIndex+1) & 0b1111111;

        if(printIndex == 0){        
            //Z_COEFF       = tan((analogRead(context, 0, 1)*2.4-1)*0.4);
            //D1            = tan((analogRead(context, 0, 2)*2.4-1)*1.10);
            //D2            = tan((analogRead(context, 0, 3)*2.4-1)*1.10);
            //Z_OFFSET      = analogRead(context, 0, 4)*1.2*8;
            //ALPHA         = tan((analogRead(context, 0, 5)*2.4-1)*1.3)*100;
            //I_FACTOR      = tan((analogRead(context, 0, 0)*2.4-1)*1.3)*10;
            rt_printf("M1 : %f \t\t M2 : %f \t\t  M2/M1 : %f \n\n  Z_COEFF %f \t D1 %f \t D2 %f \t Z_OFFSET %f \t ALPHA %f \t I_FACTOR %f \n", amplitudeAverageArray_M1_Sum*ONE_OVER_NUMBER_OF_AVERAGING_AMPLITUDES, amplitudeAverageArray_M2_Sum*ONE_OVER_NUMBER_OF_AVERAGING_AMPLITUDES, ratioArray_Sum/128.0 , Z_COEFF,D1,D2,Z_OFFSET,ALPHA, I_FACTOR);
        }
        
        printIndex = (printIndex + 1)%1000;


        if(testTimingIndex%NON_AUDIO_FREQ == 0){


            commandAmplitude_M1_Array[nonAudioTimingIndex] = gain_M1;
            //target_M1_Array[nonAudioTimingIndex] = target_M1;
            amplitude_M1_Array[nonAudioTimingIndex] = sqrtf_neon(amplitude_squared_M1);
            phase_M1_Array[nonAudioTimingIndex] = phase_M1;

            commandAmplitude_M2_Array[nonAudioTimingIndex] = gain_M2;
            //target_M2_Array[nonAudioTimingIndex] = target_M2;
            amplitude_M2_Array[nonAudioTimingIndex] = sqrtf_neon(amplitude_squared_M2);
            phase_M2_Array[nonAudioTimingIndex] = phase_M2;

            // // commandAmplitude_M3_Array[nonAudioTimingIndex] = gain_M3;
            // // target_M3_Array[nonAudioTimingIndex] = (testTimingIndex>=1*SEC) ? target_M3 : 0;
            // // amplitudeObserver_M3_Array[nonAudioTimingIndex] = estimated_amplitude_M3;
            // amplitude_M3_Array[nonAudioTimingIndex] = sqrtf_neon(amplitude_squared_M3);
            // phase_M3_Array[nonAudioTimingIndex] = phase_M3;

            // // commandAmplitude_M4_Array[nonAudioTimingIndex] = gain_M4;
            // // target_M4_Array[nonAudioTimingIndex] = (testTimingIndex>=1*SEC) ? target_M4 : 0;
            // // amplitudeObserver_M4_Array[nonAudioTimingIndex] = estimated_amplitude_M4;
            // amplitude_M4_Array[nonAudioTimingIndex] = sqrtf_neon(amplitude_squared_M4);
            // phase_M4_Array[nonAudioTimingIndex] = phase_M4;

            // // commandAmplitude_M5_Array[nonAudioTimingIndex] = gain_M5;
            // // target_M5_Array[nonAudioTimingIndex] = (testTimingIndex>=1*SEC) ? target_M5 : 0;
            // // amplitudeObserver_M5_Array[nonAudioTimingIndex] = estimated_amplitude_M5;
            // amplitude_M5_Array[nonAudioTimingIndex] = sqrtf_neon(amplitude_squared_M5);
            // phase_M5_Array[nonAudioTimingIndex] = phase_M5;

            nonAudioTimingIndex++;
        }

        testTimingIndex = testTimingIndex+1;


    }

    
}

void cleanup(BelaContext *context, void *userData)
{

 if(true){
    
     rt_printf("Saving Audio");


    writeWAV(audioArray, DURATION, "Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/audioFile.wav");

    rt_printf("Saving Command");

    writeWAV(commandSignalArray, DURATION, "Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/commandSignalFile.wav");

    rt_printf("Saving Current");

    writeWAV(outputCurrentArray, DURATION, "Linearising__Z_COEFF_" xstr(Z_COEFF) "__ALPHA_" xstr(ALPHA) "__I_FACTOR_" xstr(I_FACTOR) "__Z_OFFSET_" xstr(Z_OFFSET) "__D1_" xstr(D1) "__D2_" xstr(D2) "__ESSAI_" N_ESSAI "/outputCurrentFile.wav");


    rt_printf("Saving Command in TXT");


    for(int i=0; i<DURATION; i++){
        commandSignalFile << 10*commandSignalArray[i] << endl;
    }
    commandSignalFile.close();


    rt_printf("Saving Current in TXT");


    for(int i=0; i<DURATION; i++){
        outputCurrentFile << 10*outputCurrentArray[i] << endl;
    }
    outputCurrentFile.close();



    rt_printf("Audio OK\n");
    


    for(int i=0; i<nonAudioTimingIndex; i++){
        commandAmplitude_M1_File << commandAmplitude_M1_Array[i] << endl;
        target_M1_File << target_M1_Array[i] << endl;
        amplitudeObserver_M1_File << amplitudeObserver_M1_Array[i] << endl;
        amplitude_M1_File << amplitude_M1_Array[i] << endl;
        phase_M1_File << phase_M1_Array[i] << endl;
    }
    commandAmplitude_M1_File.close();
    target_M1_File.close();
    amplitudeObserver_M1_File.close();
    amplitude_M1_File.close();
    phase_M1_File.close();

    rt_printf("M1 OK\n");

    for(int i=0; i<nonAudioTimingIndex; i++){
        commandAmplitude_M2_File << commandAmplitude_M2_Array[i] << endl;
        target_M2_File << target_M2_Array[i] << endl;
        amplitudeObserver_M2_File << amplitudeObserver_M2_Array[i] << endl;
        amplitude_M2_File << amplitude_M2_Array[i] << endl;
        phase_M2_File << phase_M2_Array[i] << endl;
    }
    commandAmplitude_M2_File.close();
    target_M2_File.close();
    amplitudeObserver_M2_File.close();
    amplitude_M2_File.close();
    phase_M2_File.close();

    rt_printf("M2 OK\n");

    // for(int i=0; i<nonAudioTimingIndex; i++){
    //     commandAmplitude_M3_File << commandAmplitude_M3_Array[i] << endl;
    //     target_M3_File << target_M3_Array[i] << endl;
    //     amplitudeObserver_M3_File << amplitudeObserver_M3_Array[i] << endl;
    //     amplitude_M3_File << amplitude_M3_Array[i] << endl;
    //     phase_M3_File << phase_M3_Array[i] << endl;
    // }
    // commandAmplitude_M3_File.close();
    // target_M3_File.close();
    // amplitudeObserver_M3_File.close();
    // amplitude_M3_File.close();
    // phase_M3_File.close();

    // rt_printf("M3 OK\n");

    // for(int i=0; i<nonAudioTimingIndex; i++){
    //     commandAmplitude_M4_File << commandAmplitude_M4_Array[i] << endl;
    //     target_M4_File << target_M4_Array[i] << endl;
    //     amplitudeObserver_M4_File << amplitudeObserver_M4_Array[i] << endl;
    //     amplitude_M4_File << amplitude_M4_Array[i] << endl;
    //     phase_M4_File << phase_M4_Array[i] << endl;
    // }
    // commandAmplitude_M4_File.close();
    // target_M4_File.close();
    // amplitudeObserver_M4_File.close();
    // amplitude_M4_File.close();
    // phase_M4_File.close();

    // rt_printf("M4 OK\n");

    // for(int i=0; i<nonAudioTimingIndex; i++){
    //     commandAmplitude_M5_File << commandAmplitude_M5_Array[i] << endl;
    //     target_M5_File << target_M5_Array[i] << endl;
    //     amplitudeObserver_M5_File << amplitudeObserver_M5_Array[i] << endl;
    //     amplitude_M5_File << amplitude_M5_Array[i] << endl;
    //     phase_M5_File << phase_M5_Array[i] << endl;
    // }
    // commandAmplitude_M5_File.close();
    // target_M5_File.close();
    // amplitudeObserver_M5_File.close();
    // amplitude_M5_File.close();
    // phase_M5_File.close();
    
    // rt_printf("M5 OK\n");

}

}
