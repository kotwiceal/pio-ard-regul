classdef ardreg
    properties
        device
        regnum
        regbuf
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
    end

    methods
        function obj = ardreg(port, param)
            arguments
                port {mustBeA(port, {'char', 'string'})}
                param.BoardRate (1,1) {mustBeInteger} = 115200
            end
            % initialize COM
            obj.device = serialport(port, param.BoardRate, Timeout = 20);

            % dummy transcation
            obj.send(100, 100, 100);

            [~, ~, obj.regnum] = obj.send(obj.REGULATOR_NUMBER, 0, 0);
            [~, ~, obj.regbuf] = obj.send(obj.REGULATOR_BUFFER_LENGTH, 0, 0);
        end

        function monitor(obj, param)
            arguments
                obj
                param.pause = 0.1
                param.index (1,1) = 0
            end

            if (param.index > obj.regnum - 1); error(strcat("`index` must be less or equal ", num2str(obj.regnum-1))); end

            x = []; y = []; z = []; cnt = 1;
            clf;
            while 1
                pause(param.pause)
                [~, ~, x(cnt)] = obj.send(obj.REGULATOR_X_READ,param.index,0);
                [~, ~, y(cnt)] = obj.send(obj.REGULATOR_Y_READ,param.index,0);
                [~, ~, z(cnt)] = obj.send(obj.REGULATOR_FIR_READ,param.index,0);
                plot(1:cnt,[x;y*500;z]')
                cnt = cnt + 1;
                ylim([0, 1024]);
                % xregion()
                legend(["pressure", "control", "fir"])
                if (cnt > 50); xlim([cnt-50, cnt]); end
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
                type {mustBeMember(type, {'coef', 'thresholdx', 'discrepancyx', 'thresholdy'})}
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
            end

        end

        function data = read(obj, type, index)
            arguments
                obj 
                type {mustBeMember(type, {'coef', 'thresholdx', 'discrepancyx', 'thresholdy'})}
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
            end

        end

        function control(obj, state, index, coef, threshold_x, discrepency_x, threshold_y)
            arguments
                obj 
                state (1,1) logical
                index (1,1) {mustBeInteger}
                coef (1,:) double
                threshold_x (1,1)
                discrepency_x (1,1)
                threshold_y (1,1)
            end
            if (numel(coef) ~= obj.regbuf); error(strcat("`numel(coef)` must be equal", num2str(obj.regbuf))); end
            if state
                % send coef
                for i = 1:numel(coef)
                    obj.send(obj.REGULATOR_COEF_WRITE, bitor(bitshift(index, 4), i), coef(i));
                end
                % send threshold_x
                obj.send(obj.REGULATOR_THRESHOLD_X_WRITE, index, threshold_x);
                % send discrepency_x
                obj.send(obj.REGULATOR_DISCREPANCY_X_WRITE, index, discrepency_x);
                % send threshold_y
                obj.send(obj.REGULATOR_THRESHOLD_Y_WRITE, index, threshold_y);
            end
            obj.send(obj.REGULATOR_ENABLE, index, double(state));
        end

        function delete(obj)
            delete(obj.device)
        end
    end
end