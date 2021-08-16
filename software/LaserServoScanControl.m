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
            self.offset = DeviceParameter([0,15],regs(1),'int16')...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
            self.amplitude = DeviceParameter([16,31],regs(1),'int16')...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
            self.stepSize = DeviceParameter([0,15],regs(2))...
                .setLimits('lower',0,'upper',1)...
                .setFunctions('to',@(x) self.parent.convert2int(x),'from',@(x) self.parent.convert2volts(x));
            self.stepTime = DeviceParameter([0,31],regs(3))...
                .setLimits('lower',0,'upper',self.parent.CLK^-1*(2^32 - 1))...
                .setFunctions('to',@(x) x*self.parent.CLK,'from',@(x) x/self.parent.CLK);
            
            
        end
        
        function self = setDefaults(self)
            self.enable.set(0);
            self.offset.set(0);
            self.amplitude.set(1);
            
            self.duration = 100e-3;
            self.setScanSteps;
        end
        
        function self = setScanSteps(self,T)
            if nargin > 1
                self.duration = T;
            end
            
            self.stepTime.set(min(0.5*self.duration/200,self.stepTime.upperLimit));
            self.stepSize.set(2*self.amplitude.get*self.stepTime.get/(0.5*self.duration));
            
        end
        
        function [t,v] = estimateScan(self)
            t = 0:self.stepTime.get:(self.duration/2);
            v = (self.offset.get - self.amplitude.get) + self.stepSize.get*(0:(numel(t) - 1));
            if self.parent.pid(1).scanEnable.value
                v = max(v,self.parent.pid(1).lowerLimit.get);
                v = min(v,self.parent.pid(1).upperLimit.get);
            elseif self.parent.pid(2).scanEnable.value
                v = max(v,self.parent.pid(2).lowerLimit.get);
                v = min(v,self.parent.pid(2).upperLimit.get);
            end
        end
        
        function self = get(self)
            self.enable.get;
            self.offset.get;
            self.amplitude.get;
            self.stepTime.get;
            self.stepSize.get;
            self.duration = 2*2*self.amplitude.value/self.stepSize.value*self.stepTime.value;
        end
        
        function ss = print(self,width)
            s{1} = self.enable.print('Enable',width,'%d');
            s{2} = self.offset.print('Offset',width,'%.3f','V');
            s{3} = self.amplitude.print('Amplitude',width,'%.3f','V');
            s{4} = self.stepTime.print('Step Time',width,'%.3e','s');
            s{5} = self.stepSize.print('Step Size',width,'%.3e','V');
            s{6} = sprintf(['% ',num2str(width),'s: %.3e %s\n'],'Duration',self.duration,'s');
            
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