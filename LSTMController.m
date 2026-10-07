classdef LSTMController < matlab.System
    % LSTMController  LSTM-based controller that replaces the PI block.
    %   Input : e(k)  = desired water level - measured water level
    %   Output: u(k)  = controller output (pump voltage before saturation)
    %
    %   At every sample k the controller builds a 2x4 sequence from the
    %   previous 4 memory samples
    %       x = [ e(k-3)  e(k-2)  e(k-1)  e(k)
    %            ie(k-3) ie(k-2) ie(k-1) ie(k) ]
    %   where ie is the running (forward Euler) integral of the error, and
    %   feeds it to the trained LSTM network to obtain u(k).

    properties (Nontunable)
        NetFile = 'lstmController.mat'   % file holding net + scaling
        Ts = 1                           % controller sample time (s)
    end

    properties (Access = private)
        Net
        Scale
        Buffer      % 2 x 4 window of [e; ie]
        IntErr      % integral of error (state)
    end

    methods (Access = protected)
        function setupImpl(obj)
            S = load(obj.NetFile,'net','scale');
            obj.Net   = S.net;
            obj.Scale = S.scale;
        end

        function resetImpl(obj)
            obj.Buffer = zeros(2,4);
            obj.IntErr = 0;
        end

        function u = stepImpl(obj,e)
            % shift the memory window and append the newest sample
            obj.Buffer = [obj.Buffer(:,2:end) [e; obj.IntErr]];
            x = obj.Buffer ./ obj.Scale.x;                 % normalise
            X = dlarray(single(x),'CT');                   % channels x time
            u = double(extractdata(predict(obj.Net,X))) * obj.Scale.u;
            % forward Euler integral (same convention as discrete PI block)
            obj.IntErr = obj.IntErr + obj.Ts*e;
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj,'Type','Discrete','SampleTime',obj.Ts);
        end

        function out = getOutputSizeImpl(~),     out = [1 1];    end
        function out = getOutputDataTypeImpl(~), out = 'double'; end
        function out = isOutputComplexImpl(~),   out = false;    end
        function out = isOutputFixedSizeImpl(~), out = true;     end
        function flag = isInputSizeMutableImpl(~,~), flag = false; end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end
        function flag = showSimulateUsingImpl
            flag = false;
        end
    end
end
