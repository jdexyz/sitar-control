% Fast Fourier Series Coefficients
% Targets specific frequencies and samples on the period of the lowest harmonic
% 2016, Jeremy Dahan at Centre Automatique et Systemes, and at Trublion

% Notes 
% - To test against a sudden increase in amplitude, like in a plucked string configuration,
%   expressions like (2.+ (T>0.0045)) are useful.
% - With a non constant coefficient like (2.+ 100*T), abs(s) seems to be close to the
%   average value of the coefficient.
% - I tried to remove the different harmonics and reaplying the algorithm
%   on the rest, the precision is not improved. I also tried to reduce the
%   number of samples needed on the rest of the signal, with N0 = floor(fs/f0)
%   but the presence of the third harmonic prevents from giving the correct
%   result for the second harmonic.

f = 70.; % frequency of the lowest mode
fs = 44100; % Sampling frequency
m = 1; % Mode of interest
f0 = m*f; % frequency of interest
N0 = floor(fs/f); % Number of samples

DURATION = 20;
T = 0:1/fs:DURATION*(N0/fs);

signal1 =   1.*sin(2.*pi*f*T + 1.);
signal2 =   0*sin(2.*pi*(2.*f)*T + 2);
signal3 =   0*2.*sin(2.*pi*(3.*f)*T + 0.5);
noise1  =   0*1.0*sin(2.*pi*(7.231*f)*T + normrnd(0,1,1,1));
noise2  =   0*2.0*sin(2.*pi*(30.231*f)*T + normrnd(0,1,1,1));
noise3  =   0.*normrnd(0,1, 1, length(T));
hum1    =   0.1*sin(2.*pi*(50)*T + normrnd(0,1,1,1));
hum2    =   0.*sin(2.*pi*(100)*T + normrnd(0,1,1,1));

totalSignal = signal1 + signal2 + signal3  + noise1 + noise2 + noise3 + hum1 + hum2;


sequence = 1:1:(DURATION)*N0+1;


YAmplitude = zeros(1,(DURATION-1)*N0);
YPhase = zeros(1,(DURATION-1)*N0);

expSignal = exp(-1i*2.*pi*f0*sequence/fs);

signalTimesComplexExponential = totalSignal .* expSignal;

s = sum(signalTimesComplexExponential(1:(N0)));

for index = sequence(1:(DURATION-1)*N0)
    s = s - signalTimesComplexExponential(index) + signalTimesComplexExponential(index+N0);
    YAmplitude(index) = abs(s)*2/N0;
    YPhase(index) = angle(s) + pi/2;
end

plot(YAmplitude)
%plot(YPhase)

NUMBER_OF_MODES = 5;

T = 0:1/fs:((N0-1)/fs);
expSignal = exp(-1i*2.*pi*f0*T);


    
% Using an audio file
[y,fs] = audioread(url);
signal = transpose(y(:,1));



f = 440; % frequency of the lowest mode
M = 1:1:NUMBER_OF_MODES;
N0 = floor(fs/f);
DURATION = mod(length(signal),N0);



% Construct a smoothing window
gaussFilter = gausswin(1000);
gaussFilter = gaussFilter / sum(gaussFilter); % Normalize.


YAmplitude_matrix = zeros(length(M),2*(DURATION-1)*(N0));
% Y_sum = zeros(1,length(sequence));  
hold on
for m=M 

        f0 = m*f;

        T = 0:1/fs:((N0-1)/fs);
        expSignal = exp(-1i*2.*pi*f0*T);

        signalTimesComplexExponential = totalSignal .* expSignal;


        
        T = 0:1/fs:DURATION*(N0/fs);


        sequence = 1:1:(DURATION-1)*N0;
        YAmplitude = zeros(1,length(sequence));
        YPhase = zeros(1,length(sequence));

    for index = sequence
        s = 2*sum(signal(index:(index+N0-1)) .* expSignal)/(N0);
        YAmplitude(index) = abs(s);
        YPhase(index) = angle(s) + pi/2;
    end
    %plot(log(YAmplitude))
    %plot(YAmplitude)
    smoothedVector = conv(YAmplitude,gaussFilter);%conv(60+20*log(YAmplitude), gaussFilter);
    plot(smoothedVector)%80000:length(smoothedVector)))
    NC = length(smoothedVector);
    
    %YAmplitude_matrix(m,1:length(YAmplitude)) = YAmplitude;%conv(YAmplitude, gaussFilter);
    YAmplitude_matrix(m,1:NC) = smoothedVector;%conv(log(YAmplitude), gaussFilter);
    %Conv the signal with a gaussian window to get smoother results
    %Y_sum = Y_sum + YAmplitude;
    %plot(smoothedVector)
end
