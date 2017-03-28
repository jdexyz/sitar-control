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
%   number of samples needed on the rest of the signal, with N0 = floor(fs/f)
%   but the presence of the third harmonic prevents from giving the correct
%   result for the second harmonic.

f0 = 70.; % frequency of the lowest mode
fs = 441000; % Sampling frequency
m = 1; % Mode of interest
f = m*f0; % frequency of interest
N0 = floor(fs/f0); % Number of samples
T = 0:1/fs:((N0-1)/fs);

% Test signals

signal1 =   1.*sin(2.*pi*f0*T);
signal2 =   0*sin(2.*pi*(2.*f0)*T + 2);
signal3 =   0*2.*sin(2.*pi*(3.*f0)*T + 0.5);
noise1  =   0*1.0*sin(2.*pi*(7.231*f0)*T + normrnd(0,1,1,1));
noise2  =   0*2.0*sin(2.*pi*(30.231*f0)*T + normrnd(0,1,1,1));
noise3  =   0.*normrnd(0,1, 1, length(T));
hum1    =   0.1*sin(2.*pi*(50)*T + normrnd(0,1,1,1));

totalSignal = signal1 + signal2 + signal3  + signal10 + noise1 + noise2 + noise3 + hum1;


expSignal = exp(-1i*2.*pi*f*T);
s = 2*sum(totalSignal .* expSignal)/N0;

detectedCoeff = abs(s);
detectedAngle = angle(s) + pi/2;

plot(T,totalSignal)
display(detectedCoeff);
display(detectedAngle);
