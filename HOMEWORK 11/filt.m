close all
a = load('C:\Users\Christopher\Documents\GitHub\hidapi\windows\accels.txt');
%fir1
%filtfilt
plotFFT(a(:,3));

figure
hold on
plot(a(:,3))
% b = fir1(7,.2);
% array1=filtfilt(b,1,a(:,3))
% hold on
% plot(array1)

plot(a(:,4),'r')
%title('MAF data')

plot(a(:,5),'g')
%title('FIR data')
legend('Raw','MAF','FIR')
xlabel('sample number (time axis)')
ylabel('acceleration reading')
title('No shake MAF and FIR filtering')