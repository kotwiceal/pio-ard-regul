%% initialize MCU instance
clc
clear mcu
mcu = ardreg('COM4')
%% enable control
i = 0;
mcu.control(true,i,[1,0,0,0],700,50,1)
mcu.read('coef',i)
mcu.read('thresholdx',i)
mcu.read('discrepancyx',i)
mcu.read('thresholdy',i)
%% enable monitor
mcu.monitor(index=0)
%% write coef
mcu.write('coef',3,[40,30,20,10])
%% read coef
mcu.read('coef',0)
%% write threshold x
mcu.write('thresholdx',2,30)
mcu.read('thresholdx',2)
%% write discrepency x
mcu.write('discrepancyx',0,30)
mcu.read('discrepancyx',0)
%% write threshold y
mcu.write('thresholdy',2,10)
mcu.read('thresholdy',2)