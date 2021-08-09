classdef LaserServoPID < handle
    %LASERSERVOPID Defines a class for handling the PID modules in the
    %laser servo
    
    properties(SetAccess = immutable)
        Kp
        Ki
        Kd
        divisor
        polarity
        enable
        control
        lowerLimit
        upperLimit
    end
    
    properties(SetAccess = protected)
        parent
    end
    
    methods
        function self = LaserServoPID(parent,reg)
            self.parent = parent;
            
            self.enable = DeviceParameter([0,0],reg(1))...
                .setLimits('lower',0,'upper',1);
            self.polarity = DeviceParameter([1,1],reg(1))...
                .setLimits('lower',0,'upper',1);
            self.control = DeviceParameter([31,16],reg(1))...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
            self.Kp = DeviceParameter([0,15],reg(2))...
                .setLimits('lower',0,'upper',2^16-1);
            self.Ki = DeviceParameter([31,16],reg(2))...
                .setLimits('lower',0,'upper',2^16-1);
            self.Kd = DeviceParameter([0,15],reg(3))...
                .setLimits('lower',0,'upper',2^16-1);
            self.divisor = DeviceParameter([31,16],reg(3))...
                .setLimits('lower',0,'upper',2^16-1);
            self.lowerLimit = DeviceParameter([0,15],reg(4))...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
            self.upperLimit = DeviceParameter([31,16],reg(4))...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
        end
        
        function self = setDefaults(self)
            if numel(self) > 1
                for nn = 1:numel(self)
                    self(nn).setDefaults;
                end
            else
                self.enable.set(0);
                self.polarity.set(0);
                self.control.set(0);
                self.Kp.set(0);
                self.Ki.set(0);
                self.Kd.set(0);
                self.divisor.set(0);
                self.lowerLimit.set(-1);
                self.upperLimit.set(1);
            end
        end
        
        function self = get(self)
            if numel(self) > 1
                for nn = 1:numel(self)
                    self(nn).get
                end
            else
                self.enable.get;
                self.polarity.get;
                self.control.get;
                self.Kp.get;
                self.Ki.get;
                self.Kd.get;
                self.divisor.get;
                self.lowerLimit.get;
                self.upperLimit.get;
            end
        end
        
    end
    
end