classdef DeviceParameterString < DeviceParameter
    
    methods
        function self = DeviceParameterString(varargin)
            self@DeviceParameter(varargin{:});
        end
        
        
        function self = checkLimits(self,~)
            %CHECKLIMITS Checks the limits on the set value
            
            
        end
        
    end
    
end