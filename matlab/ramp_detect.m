% ramp detect filter: result is in change of units per sample
%   where signal is in units
%   i.e. sample is inch Hg, then filter output will be in
%   delta inch Hg per sample

% technique fully described here --
% Slope Filtering: An FIR Approach to Linear Regression - Clay S. Turner 
% http://www.claysturner.com/dsp/fir_regression.pdf

N=21;
b = zeros(N,1);

for i=0:N-1
  b(i+1) = -(12*i-6*(N-1)) / (N*(N^2-1));
end

start_pressure = 977;  % pressure at 1000 ft. in mbar
launch_slope = -0.53;    % 5 m/s rise rate in Hg/sec

% flight length 2000 seconds; 1 samp/sec
t=1:2000;

sig1 = start_pressure + [zeros(1,500) t(1:500)*launch_slope t(500:-1:1)*launch_slope zeros(1,500)];  % simulated launch and flight pressure curve
sig1 = sig1 + 2*rand(1,length(t));  % add some noise

% apply slope detecting filter
sig_fir = filter(b,[1],sig1);

subplot(2,1,1)
plot(t(101:length(t)),sig1(101:length(t)))
title("simulated ballon flight; launch, balloon burst, and descent");
xlabel("time (sec)");
ylabel("pressure (mbar)");

subplot(2,1,2)
plot(t(101:length(t)),sig_fir(101:length(t)));
title("slope detecting filter (N=21)");
xlabel("time (sec)");
ylabel("slope (in. Hg/sec)");

