% Fast Fourier Series Coefficients
% Targets specific frequencies and samples on the period of the lowest harmonic
% 2016, Jeremy Dahan at Centre Automatique et Systemes, and at Trublion

f0 = 70.; % frequency of the lowest mode
fs = 44100; % Sampling frequency
m = 1; % Mode of interest
f = 1*m*f0; % frequency of interest
N0 = floor(fs/f0); % Number of samples

DURATION = 20;
T = 0:1/fs:DURATION*(N0/fs);

signal1 =   1.*sin(2.*pi*f0*T + 1.);
signal2 =   2*sin(2.*pi*(2.*f0)*T + 2);
signal3 =   0*2.*sin(2.*pi*(3.*f0)*T + 0.5);
noise1  =   0*1.0*sin(2.*pi*(7.231*f0)*T + normrnd(0,1,1,1));
noise2  =   0*2.0*sin(2.*pi*(30.231*f0)*T + normrnd(0,1,1,1));
noise3  =   0.*normrnd(0,1, 1, length(T));
hum1    =   0.*sin(2.*pi*(50)*T + normrnd(0,1,1,1));
hum2    =   0.*sin(2.*pi*(100)*T + normrnd(0,1,1,1));

totalSignal = signal1 + signal2 + signal3  + noise1 + noise2 + noise3 + hum1 + hum2;


sequence = 1:1:(DURATION)*N0+1;


YAmplitude = zeros(1,(DURATION-1)*N0);
YPhase = zeros(1,(DURATION-1)*N0);

expSignal = exp(-1i*2.*pi*f*sequence/fs);

signalTimesComplexExponential = totalSignal .* expSignal;

s = sum(signalTimesComplexExponential(1:(N0)));

for index = sequence(1:(DURATION-1)*N0)
    s = s - signalTimesComplexExponential(index) + signalTimesComplexExponential(index+N0);
    YAmplitude(index) = abs(s)*2/N0;
    YPhase(index) = angle(s) + pi/2;
end


%plot(YAmplitude)
%plot(YPhase)

% Useful to check that the output can be used as a phase lock mechanism

resynthedSignal = zeros(1,(DURATION-1)*N0);

for index = sequence(1:(DURATION-1)*N0)
    resynthedSignal(index) = YAmplitude(index)*sin(2.*pi*f*index/fs + YPhase(index));
end

plot(resynthedSignal);
hold on
plot(totalSignal);