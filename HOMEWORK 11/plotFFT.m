function plotFFT(x)
if mod(length(x),2) == 1
    x = [x 0];
end
N = length(x);
X = fft(x);
mag(1) = abs(X(1))/N;
mag(2:N/2) = 2*abs(X(2:N/2))/N;
freqs = linspace(0, 1, N/2+1);
stem(mag);
%axis([-.05 1.05 -0.1*max(mag) max(mag)+0.1*max(mag)]);
xlabel('Frequency');
ylabel('Magnitude');
title('Single-Sided FFT Magnitude');
set(gca,'FontSize',18);