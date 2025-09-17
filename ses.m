%% initialize MCU instance
clc
clear mcu
mcu = ardreg('COM4')
%% setup control
i = 0; % regulator index
mcu = mcu.control(true,i,[1,0,0,0],300,50,1,100);
mcu.read('coef',i)
mcu.read('thresholdx',i)
mcu.read('discrepancyx',i)
mcu.read('thresholdy',i)
mcu.read('deadtime',i)
%% enable monitor
mcu.monitor(index=0) % `ESC` key to safely terminate loop
%% write FIR coefficients
mcu.write('coef',0,[40,30,20,10])
%% read FIR coefficients
mcu.read('coef',0)
%% write pressure threshold
mcu.write('thresholdx',2,30)
mcu.read('thresholdx',2)
%% write pressure discrepency
mcu.write('discrepancyx',0,30)
mcu.read('discrepancyx',0)
%% write FIR result threshold
mcu.write('thresholdy',2,10)
mcu.read('thresholdy',2)
%%