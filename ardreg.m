classdef ardreg
    properties
        device
        regnum
        regbuf
        threshold_x (1,:)
        discrepency_x (1,:)
        threshold_y (1,:)
        DIGITAL_OUT_WRITE = 1
        DIGITAL_OUT_READ = 2
        ANALOG_READ = 3
        DIGITAL_IN_READ = 4
        REGULATOR_ENABLE = 5
        REGULATOR_COEF_WRITE = 6
        REGULATOR_COEF_READ = 7
        REGULATOR_THRESHOLD_X_WRITE = 8
        REGULATOR_THRESHOLD_X_READ = 9
        REGULATOR_DISCREPANCY_X_WRITE = 10
        REGULATOR_DISCREPANCY_X_READ = 11
        REGULATOR_THRESHOLD_Y_WRITE = 12
        REGULATOR_THRESHOLD_Y_READ = 13
        REGULATOR_X_READ = 14
        REGULATOR_Y_READ = 15
        REGULATOR_FIR_READ = 16
        REGULATOR_NUMBER = 17
        REGULATOR_BUFFER_LENGTH = 18
        REGULATOR_DEADTIME_READ = 19
        REGULATOR_DEADTIME_WRITE = 20
        buflen = 1000
    end

    methods
        function obj = ardreg(port, param)
            arguments
                port {mustBeA(port, {'char', 'string'})}
                param.BoardRate (1,1) {mustBeInteger} = 115200
            end
            % initialize COM
            obj.device = serialport(port, param.BoardRate, Timeout = 5);

            % dummy transcation
            obj.send(100, 100, 100);

            [~, ~, obj.regnum] = obj.send(obj.REGULATOR_NUMBER, 0, 0);
            [~, ~, obj.regbuf] = obj.send(obj.REGULATOR_BUFFER_LENGTH, 0, 0);

            obj.threshold_x = zeros(1, obj.regnum);
            obj.discrepency_x = zeros(1, obj.regnum);
            obj.threshold_y = zeros(1, obj.regnum);
        end

        function obj = monitor(obj, param)
            arguments
                obj
                param.channel (1,:) = 0
                param.pause = 0.1
                param.docked (1,1) = false
                param.buflen (1,1) = 1024
                param.ylim (1,2) = [0, 1024]
            end

            if (param.channel > obj.regnum - 1); error(strcat("`channel` must be less or equal ", num2str(obj.regnum-1))); end
            
            thrx = []; discx = [];
            for i = 1:numel(param.channel)
                [~, ~, thrx(i)] = obj.send(obj.REGULATOR_THRESHOLD_X_READ, param.channel(i), 0);
                [~, ~, discx(i)] = obj.send(obj.REGULATOR_DISCREPANCY_X_READ, param.channel(i), 0);
            end

            cnt = 1; x = cell(1, numel(param.channel)); y = cell(1, numel(param.channel)); 
            lb = num2cell(zeros(1, numel(param.channel))); 
            rb = num2cell(zeros(1, numel(param.channel)));

            if param.docked; figure(WindowStyle = 'docked'); else; clf; end
            f = gcf; t = tiledlayout(f, 'flow');
            ax = cellfun(@(~) nexttile(t), num2cell(param.channel), UniformOutput = false);
            colors = colororder('gem');
            flag = true;
            set(f, 'KeyPressFcn', @keyhandler);
            
            % loop
            while flag
                for i = 1:numel(param.channel)
                    % transaction
                    pause(param.pause)
                    [~, ~, x{i}(cnt)] = obj.send(obj.REGULATOR_X_READ,param.channel(i),0);
                    [~, ~, y{i}(cnt)] = obj.send(obj.REGULATOR_Y_READ,param.channel(i),0);
                    
                    % visualize
                    cla(ax{i}); hold(ax{i}, 'on'); grid(ax{i}, 'on'); box(ax{i}, 'on');
                    plot(ax{i}, 1:cnt, x{i})
                    yline(ax{i}, thrx(i),'-','threshold')
                    yregion(ax{i}, thrx(i) - discx(i), thrx(i) + discx(i))
                    ylim(ax{i}, param.ylim);
    
                    if cnt > 1
                        if (y{i}(end) == 1)
                            if (y{i}(end-1) == 1)
                                rb{i}(end) = cnt;
                            else
                                rb{i} = [rb{i}, cnt];
                                lb{i} = [lb{i}, rb{i}(end)];
                            end
                        else
    
                        end
                        xregion(ax{i}, lb{i}, rb{i}, FaceColor = colors(2,:), FaceAlpha = 0.25)
                    end
  
                    xlabel(ax{i}, 'time, counts'); ylabel(ax{i}, 'amplitude, counts');
                    legend(ax{i}, ["pressure", "threshold", "discrepancy", "control"], Location = 'Best')
                    title(ax{i}, strcat("channel=", num2str(param.channel(i))), FontWeight = 'normal')
                    if (cnt > 50); xlim(ax{i}, [cnt-50, cnt]); end
                end
                % update step
                cnt = cnt + 1;
                if (cnt > param.buflen)
                    cnt = 1;
                    x = cell(1, numel(param.channel)); y = cell(1, numel(param.channel)); 
                    lb = num2cell(zeros(1, numel(param.channel))); 
                    rb = num2cell(zeros(1, numel(param.channel)));
                    for i = 1:numel(param.channel)
                        xlim(ax{i}, [0, 50])
                    end
                end
            end
            
            function keyhandler(~, evt)
                flag = ~strcmp(evt.Key, "escape");
            end
        end

        function [command, address, data] = send(obj, command, address, data)
            data = bin2dec(bin(fi(data,1,16,0)));
            packet = [command, address, bitshift(data, -8), bitand(data, 255)];
            write(obj.device, packet, 'uint8');
            try
                packet = read(obj.device, numel(packet), 'uint8');
                command = packet(1);
                address = packet(2);
                data = bitor(bitshift(packet(3), 8), packet(4));
            catch
                command = [];
                address = [];
                data = [];
            end
        end

        function write(obj, type, index, value)
            arguments
                obj 
                type {mustBeMember(type, {'coef', 'thresholdx', 'discrepancyx', 'thresholdy', 'deadtime'})}
                index (1,1) {mustBeInteger}
                value (1,:)
            end
        
            if (index > obj.regnum - 1); error(strcat("`index` must be less or equal ", num2str(obj.regnum-1))); end

            switch type
                % FIR filter coefficient
                case 'coef'
                    command = obj.REGULATOR_COEF_WRITE;
                    for i = 1:numel(value)
                        obj.send(command, bitor(bitshift(index, 4), i), value(i));
                    end
                case 'thresholdx'
                    command = obj.REGULATOR_THRESHOLD_X_WRITE;
                    obj.send(command, index, value);
                % discrepancy of ADC signal
                case 'discrepancyx'
                    command = obj.REGULATOR_DISCREPANCY_X_WRITE;
                    obj.send(command, index, value);
                % threshold of filtered ADC signal by FIR
                case 'thresholdy'
                    command = obj.REGULATOR_THRESHOLD_Y_WRITE;
                    obj.send(command, index, value);
                % trigger schimitt dead time
                case 'deadtime'
                    command = obj.REGULATOR_DEADTIME_WRITE;
                    obj.send(command, index, value);
            end

        end

        function data = read(obj, type, index)
            arguments
                obj 
                type {mustBeMember(type, {'coef', 'thresholdx', 'discrepancyx', 'thresholdy', 'deadtime'})}
                index (1,1) {mustBeInteger}
            end

            if (index > obj.regnum - 1); error(strcat("`index` must be less or equal ", num2str(obj.regnum-1))); end

            data = [];
            switch type
                % FIR filter coefficient
                case 'coef'
                    command = obj.REGULATOR_COEF_READ;
                    for i = 1:obj.regbuf
                        [~, ~, data(i)] = obj.send(command, bitor(bitshift(index, 4), i), 0);
                    end
                % threshold of ADC signal
                case 'thresholdx'
                    command = obj.REGULATOR_THRESHOLD_X_READ;
                    [~, ~, data] = obj.send(command, index, 0);
                % discrepancy of ADC signal
                case 'discrepancyx'
                    command = obj.REGULATOR_DISCREPANCY_X_READ;
                    [~, ~, data] = obj.send(command, index, 0);
                % threshold of filtered ADC signal by FIR
                case 'thresholdy'
                    command = obj.REGULATOR_THRESHOLD_Y_READ;
                    [~, ~, data] = obj.send(command, index, 0);
                % trigger schimitt dead time
                case 'deadtime'
                    command = obj.REGULATOR_DEADTIME_READ;
                    [~, ~, data] = obj.send(command, index, 0);
            end

        end

        function obj = control(obj, channel, state, param)
            arguments
                obj 
                channel (1,1) {mustBeInteger}
                state (1,1) logical
                param.coef (1,:) double = []
                param.threshold (1,:) = []
                param.discrepancy (1,:) = []
                param.deadtime (1,:) = []
            end
            if ~isempty(param.coef) 
                if (numel(param.coef) ~= obj.regbuf); error(strcat("`numel(coef)` must be equal ", num2str(obj.regbuf))); end
            end
            if state
                if ~isempty(param.coef); obj.write('coef',channel,param.coef); end
                if ~isempty(param.threshold); obj.write('thresholdx',channel,param.threshold); end
                if ~isempty(param.discrepancy); obj.write('discrepancyx',channel,param.discrepancy); end
                if ~isempty(param.deadtime); obj.write('deadtime',channel,param.deadtime); end
            end
            obj.send(obj.REGULATOR_ENABLE, channel, double(state));
        end

        function delete(obj)
            delete(obj.device)
        end
    end
end