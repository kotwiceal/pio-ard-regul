%% show port list
serialportlist
%% initialize MCU instance
clc
clear mcu
mcu = ardreg('COM4')
%% setup & eneble control
mcu = mcu.control(0,true,threshold=300,discrepancy=50,deadtime=100);
mcu = mcu.control(1,true,threshold=500,discrepancy=100,deadtime=100);
%% enable monitor
mcu.monitor(channel=[0,1]) % press `ESC` key to safely terminate loop
%% disable contol
mcu = mcu.control(0,false);
mcu = mcu.control(1,false);
%%