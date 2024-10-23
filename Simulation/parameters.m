BUFFER_SIZE = 200;
numPulses = 1;

% Define the range of decades
start_freq = 0.1;  % Starting frequency 
end_freq = 5000;   % Ending frequency 

ADC_SamplingRate = 5000;
executionRate = 5*ADC_SamplingRate;
R_Shunt = 38e-3;


% Initialize an empty array to hold the frequencies
frequencies = [];

clear frequencies
index = 0;
for exponent = -1:ceil(log10(end_freq)) % Adjust this range for more decades if needed
    for i = [1 2 3 4 5 6 7 8 9]
        index = index + 1;
        frequency =  i * 10^exponent;
        frequencies(index) =  frequency;
    end
end


Batt.OCV = 3.7;
Batt.L_R = 3.41e-7;
Batt.R_0 = 3.82e-2;
Batt.R_1 = 1.81e-3;
Batt.C_1 = 1.75e1;
Batt.R_2 = 5.53e-3; 
Batt.C_2 = 5.38e-1;



frequencies = frequencies(frequencies >= start_freq &  frequencies <= end_freq );
num_frequencies = length(frequencies);
numBuffers = num_frequencies-9;
clear decades decade index frequency i start_freq end_freq exponent freqPerDec


% Known parameters
VCCS.Vcc = 3.7;               % Supply voltage (V)
VCCS.V_DAC = 3.3;             % Maximum Desired DAC Output
VCCS.Ic = 50e-3;              % Desired Collector current (A)
VCCS.Vbe = 1.2;               % for BC239C