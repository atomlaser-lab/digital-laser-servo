classdef LaserServoScanControl < handle
    %LASERSERVOSCANCONTROL Defines a class for handling laser servo
    %scanning
    
    properties(SetAccess = immutable)
        enable
        amplitude
        offset
        stepTime
        stepSize
    end
    
    properties(SetAccess = protected)
        parent
        
        duration
    end
    
    methods
        function self = LaserServoScanControl(parent,regs)
            self.parent = parent;
            
            self.enable = DeviceParameter([16,16],self.parent.topReg)...
                .setLimits('lower',0,'upper',1);
            self.offset = DeviceParameter([0,15],regs(1))...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
            self.amplitude = DeviceParameter([31,16],regs(1))...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
            self.stepTime = DeviceParameter([15,0],regs(2))...
                .setLimits('lower',0,'upper',self.parent.CLK^-1*2^16)...
                .setFunctions('to',@(x) x*self.parent.CLK,'from',@(x) x/self.parent.CLK);
            self.stepSize = DeviceParameter([31,16],regs(2))...
                .setLimits('lower',0,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
            
        end
        
        function self = setDefaults(self)
            self.enable.set(0);
            self.offset.set(0);
            self.amplitude.set(0);
            
            self.duration = 100e-3;
            self.setScanSteps;
        end
        
        function self = setScanSteps(self,T)
            if nargin > 1
                self.duration = T;
            end
            
            self.stepTime.set(self.duration/1000);
            self.stepSize.set(2*self.amplitude.value/self.stepTime.value);
        end
        
        function self = get(self)
            self.enable.get;
            self.offset.get;
            self.amplitude.get;
            self.stepTime.get;
            self.stepSize.get;
            self.duration = 2*self.amplitude.value/self.stepSize.value*self.stepTime.value;
        end
    end
    
end