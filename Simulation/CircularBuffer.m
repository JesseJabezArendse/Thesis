classdef CircularBuffer < matlab.System
    % CircularBuffer Circular Buffer for a signal
    %
    % This class implements a circular buffer using the MATLAB System
    % framework, allowing integration with Simulink.
    
    %#codegen
    
    properties (Nontunable)
        BufferLength = 10; % Length of the buffer
    end
    
    properties
        InitialCondition = single(0); % Initial condition for the buffer (single type)
    end
    
    properties(Access = private)
        Buffer; % Persistent buffer (single type)
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            % Initialize the buffer during the first call
            if isequal(numel(obj.InitialCondition), obj.BufferLength)
                obj.Buffer = single(obj.InitialCondition); % Ensure buffer is single type
            elseif isscalar(obj.InitialCondition)
                obj.Buffer = single(obj.InitialCondition) * ones(1, obj.BufferLength, 'single'); % Single precision buffer
            else
                error('InitialCondition must either be scalar or the same dimensions as BufferLength');
            end
        end
        
        function out = stepImpl(obj, signal)
            % Output the current state of the buffer
            out = obj.Buffer;
            
            % Update the buffer with the new signal (ensure single type)
            obj.Buffer = [single(signal), obj.Buffer(1:end-1)];
        end
        
        function resetImpl(obj)
            % Reset the buffer to initial condition (single type)
            if isequal(numel(obj.InitialCondition), obj.BufferLength)
                obj.Buffer = single(obj.InitialCondition);
            elseif isscalar(obj.InitialCondition)
                obj.Buffer = single(obj.InitialCondition) * ones(1, obj.BufferLength, 'single');
            end
        end
        
    end
    
    methods(Access = protected)
        function num = getNumInputsImpl(~)
            num = 1; % Number of inputs
        end
        
        function num = getNumOutputsImpl(~)
            num = 1; % Number of outputs
        end
    end
end
