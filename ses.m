%% show port list
serialportlist
%% initialize MCU instance
clc
clear mcu
mcu = ardreg('COM4')
%% setup & eneble control
mcu = mcu.control(0,true,threshold=220,discrepancy=5,deadtime=20);
%%
mcu = mcu.control(1,true,threshold=500,discrepancy=100,deadtime=100);
%% enable monitor
mcu.monitor(channel=[0,1]) % press `ESC` key to safely terminate loop
%% disable contol
mcu = mcu.control(0,false);
mcu = mcu.control(1,false);
%% open/close valve
mcu.write('valve',0,0)
%% read data
mcu.read('thresholdx',0)
mcu.read('discrepancyx',0)
%% read ADC
mcu.read('sensor',0)