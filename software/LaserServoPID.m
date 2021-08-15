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
        scanEnable
        control
        lowerLimit
        upperLimit
    end
    
    properties(SetAccess = protected)
        parent
    end
    
    methods
        function self = LaserServoPID(parent,regs)
            self.parent = parent;
            
            self.enable = DeviceParameter([0,0],regs(1))...
                .setLimits('lower',0,'upper',1);
            self.polarity = DeviceParameter([1,1],regs(1))...
                .setLimits('lower',0,'upper',1);
            self.scanEnable = DeviceParameter([2,2],regs(1))...
                .setLimits('lower',0,'upper',1);
            self.control = DeviceParameter([16,31],regs(1),'int16')...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
            self.Kp = DeviceParameter([0,15],regs(2))...
                .setLimits('lower',0,'upper',2^16-1);
            self.Ki = DeviceParameter([16,31],regs(2))...
                .setLimits('lower',0,'upper',2^16-1);
            self.Kd = DeviceParameter([0,15],regs(3))...
                .setLimits('lower',0,'upper',2^16-1);
            self.divisor = DeviceParameter([16,31],regs(3))...
                .setLimits('lower',0,'upper',2^16-1);
            self.lowerLimit = DeviceParameter([0,15],regs(4),'int16')...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
            self.upperLimit = DeviceParameter([16,31],regs(4),'int16')...
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
                self.scanEnable.set(0);
                self.control.set(0);
                self.Kp.set(0);
                self.Ki.set(0);
                self.Kd.set(0);
                self.divisor.set(3);
                self.lowerLimit.set(-1);
                self.upperLimit.set(1);
            end
        end
        
        function self = get(self)
            if numel(self) > 1
                for nn = 1:numel(self)
                    self(nn).get;
                end
            else
                self.enable.get;
                self.polarity.get;
                self.scanEnable.get;
                self.control.get;
                self.Kp.get;
                self.Ki.get;
                self.Kd.get;
                self.divisor.get;
                self.lowerLimit.get;
                self.upperLimit.get;
            end
        end
        
        function [Kp,Ki,Kd] = calculateRealGains(self)
            Kp = self.Kp.value*2^(-self.divisor.value);
            Ki = self.Ki.value*2^(-self.divisor.value)*self.parent.dt();
            Kd = self.Kd.value*2^(-self.divisor.value)/self.parent.dt();
        end
        
        function ss = print(self,width)
            s{1} = self.enable.print('Enable',width,'%d');
            s{2} = self.polarity.print('Polarity',width,'%d');
            s{3} = self.scanEnable.print('Scan Enable',width,'%d');
            s{4} = self.control.print('Control',width,'%.3f','V');
            s{5} = self.Kp.print('Kp',width,'%d');
            s{6} = self.Ki.print('Ki',width,'%d');
            s{7} = self.Kd.print('Kd',width,'%d');
            s{8} = self.divisor.print('Divisor',width,'%d');
            s{9} = self.lowerLimit.print('Lower Limit',width,'%.3f','V');
            s{10} = self.upperLimit.print('Upper Limit',width,'%.3f','V');
            
            ss = '';
            for nn = 1:numel(s)
                ss = [ss,s{nn}]; %#ok<*AGROW>
            end
            if nargout == 0
                fprintf(1,ss);
            end
        end
        
    end
    
end