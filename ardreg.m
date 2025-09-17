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
                param.pause = 0.1
                param.index (1,1) = 0
                param.docked (1,1) = false
            end

            if (param.index > obj.regnum - 1); error(strcat("`index` must be less or equal ", num2str(obj.regnum-1))); end
            [~, ~, thrx] = obj.send(obj.REGULATOR_THRESHOLD_X_READ, param.index, 0);
            [~, ~, discx] = obj.send(obj.REGULATOR_DISCREPANCY_X_READ, param.index, 0);

            cnt = 1; x = []; y = []; z = []; lb = 0; rb = 0;
            if param.docked; figure(WindowStyle = 'docked'); else; clf; end
            f = gcf; t = tiledlayout(f, 'flow');
            ax = nexttile(t);
            colors = colororder('gem');
            flag = true;
            set(f, 'KeyPressFcn', @keyhandler);
            while flag
                pause(param.pause)
                [~, ~, x(cnt)] = obj.send(obj.REGULATOR_X_READ,param.index,0);
                [~, ~, y(cnt)] = obj.send(obj.REGULATOR_Y_READ,param.index,0);
                [~, ~, z(cnt)] = obj.send(obj.REGULATOR_FIR_READ,param.index,0);
                
                cla(ax); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
                plot(ax, 1:cnt, x)
                yline(ax, thrx,'-','threshold')
                yregion(ax, thrx - discx, thrx + discx)
                ylim(ax, [0, 1024]);

                if cnt > 1
                    if (y(end) == 1)
                        if (y(end-1) == 1)
                            rb(end) = cnt;
                        else
                            rb = [rb, cnt];
                            lb = [lb, rb(end)];
                        end
                    else

                    end
                    xregion(ax,lb,rb,FaceColor = colors(2,:), FaceAlpha = 0.25)
                end

                xlabel(ax, 'time, counts'); ylabel(ax, 'amplitude, counts');
                legend(ax, ["pressure", "threshold", "discrepancy", "control"], Location = 'Best')
                title(ax, strcat("channel=", num2str(param.index)), FontWeight = 'normal')
                % update step
                cnt = cnt + 1;
                if (cnt > 50); xlim(ax, [cnt-50, cnt]); end
                if (cnt > obj.buflen)
                    cnt = 1; x = []; y = []; z = []; lb = []; rb = [];
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
                case 'coef'
                    command = obj.REGULATOR_COEF_WRITE;
                    for i = 1:numel(value)
                        obj.send(command, bitor(bitshift(index, 4), i), value(i));
                    end
                case 'thresholdx'
                    command = obj.REGULATOR_THRESHOLD_X_WRITE;
                    obj.send(command, index, value);
                case 'discrepancyx'
                    command = obj.REGULATOR_DISCREPANCY_X_WRITE;
                    obj.send(command, index, value);
                case 'thresholdy'
                    command = obj.REGULATOR_THRESHOLD_Y_WRITE;
                    obj.send(command, index, value);
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
                case 'coef'
                    command = obj.REGULATOR_COEF_READ;
                    for i = 1:obj.regbuf
                        [~, ~, data(i)] = obj.send(command, bitor(bitshift(index, 4), i), 0);
                    end
                case 'thresholdx'
                    command = obj.REGULATOR_THRESHOLD_X_READ;
                    [~, ~, data] = obj.send(command, index, 0);
                case 'discrepancyx'
                    command = obj.REGULATOR_DISCREPANCY_X_READ;
                    [~, ~, data] = obj.send(command, index, 0);
                case 'thresholdy'
                    command = obj.REGULATOR_THRESHOLD_Y_READ;
                    [~, ~, data] = obj.send(command, index, 0);
                case 'deadtime'
                    command = obj.REGULATOR_DEADTIME_READ;
                    [~, ~, data] = obj.send(command, index, 0);
            end

        end

        function obj = control(obj, state, index, param)
            arguments
                obj 
                state (1,1) logical
                index (1,1) {mustBeInteger}
                param.coef (1,:) double = []
                param.threshold_x (1,1) = []
                param.discrepency_x (1,1) = []
                param.threshold_y (1,1) = []
                param.deadtime (1,1) = []
            end
            if (numel(coef) ~= obj.regbuf); error(strcat("`numel(coef)` must be equal", num2str(obj.regbuf))); end
            if state
                if ~isempty(param.coef); obj.write('coef',index,param.coef); end
                if ~isempty(param.threshold_x); obj.write('thresholdx',index,param.threshold_x); end
                if ~isempty(param.discrepency_x); obj.write('discrepencyx',index,param.discrepency_x); end
                if ~isempty(param.threshold_y); obj.write('thresholdy',index,param.threshold_y); end
                if ~isempty(param.deadtime); obj.write('deadtime',index,param.deadtime); end
            end
            obj.send(obj.REGULATOR_ENABLE, index, double(state));
        end

        function delete(obj)
            delete(obj.device)
        end
    end
end