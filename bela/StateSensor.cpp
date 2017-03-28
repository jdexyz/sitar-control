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





*/


#include <Bela.h>
#include <array>
#include <cmath>
#include <iostream>


// For now, fixed frequency : 
#define FUNDAMENTAL_FREQUENCY   74
#define SAMPLING_FREQUENCY      88200.0
#define NUMBER_OF_SAMPLES       1191
#define PI                      3.14159265358979323846


const float FREQUENCY_OF_INTEREST_M1 = FUNDAMENTAL_FREQUENCY;
const float FREQUENCY_OF_INTEREST_M2 = 2*FUNDAMENTAL_FREQUENCY;
const float FREQUENCY_OF_INTEREST_M3 = 3*FUNDAMENTAL_FREQUENCY;
const float FREQUENCY_OF_INTEREST_M4 = 4*FUNDAMENTAL_FREQUENCY;
const float FREQUENCY_OF_INTEREST_M5 = 5*FUNDAMENTAL_FREQUENCY;

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



bool setup(BelaContext *context, void *userData)
{

    for(unsigned int n = 0; n<NUMBER_OF_SAMPLES; n++){
        COMPLEX_EXPONENTIAL_TABLE_M1_R[n] = cos(n*2*PI*FREQUENCY_OF_INTEREST_M1/SAMPLING_FREQUENCY);
        COMPLEX_EXPONENTIAL_TABLE_M1_I[n] = -sin(n*2*PI*FREQUENCY_OF_INTEREST_M1/SAMPLING_FREQUENCY);
        COMPLEX_EXPONENTIAL_TABLE_M2_R[n] = cos(n*2*PI*FREQUENCY_OF_INTEREST_M2/SAMPLING_FREQUENCY);
        COMPLEX_EXPONENTIAL_TABLE_M2_I[n] = -sin(n*2*PI*FREQUENCY_OF_INTEREST_M2/SAMPLING_FREQUENCY);
        COMPLEX_EXPONENTIAL_TABLE_M3_R[n] = cos(n*2*PI*FREQUENCY_OF_INTEREST_M3/SAMPLING_FREQUENCY);
        COMPLEX_EXPONENTIAL_TABLE_M3_I[n] = -sin(n*2*PI*FREQUENCY_OF_INTEREST_M3/SAMPLING_FREQUENCY);
        COMPLEX_EXPONENTIAL_TABLE_M4_R[n] = cos(n*2*PI*FREQUENCY_OF_INTEREST_M4/SAMPLING_FREQUENCY);
        COMPLEX_EXPONENTIAL_TABLE_M4_I[n] = -sin(n*2*PI*FREQUENCY_OF_INTEREST_M4/SAMPLING_FREQUENCY);
        COMPLEX_EXPONENTIAL_TABLE_M5_R[n] = cos(n*2*PI*FREQUENCY_OF_INTEREST_M5/SAMPLING_FREQUENCY);
        COMPLEX_EXPONENTIAL_TABLE_M5_I[n] = -sin(n*2*PI*FREQUENCY_OF_INTEREST_M5/SAMPLING_FREQUENCY);
    }
    return true;
}

void render(BelaContext *context, void *userData)
{

    for(unsigned int n = 0; n < context->analogFrames; n++) {
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

        latestSample = analogRead(context, n, 0);
        
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

        printIndex = (printIndex + 1)%8820;

        if(printIndex == 0){
            rt_printf("M1 : %f \t\t M2 : %f \t\t M3 : %f \t\t M4 : %f \t\t M5 : %f \n", 20*log(amplitude_squared_M1), 20*log(amplitude_squared_M2), 20*log(amplitude_squared_M3), 20*log(amplitude_squared_M4), 20*log(amplitude_squared_M5));
        }
        

    }

    
}

void cleanup(BelaContext *context, void *userData)
{

}
