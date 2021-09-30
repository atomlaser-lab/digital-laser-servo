classdef LaserServoScanControl < handle
    %LASERSERVOSCANCONTROL Defines a class for handling laser servo
    %scanning
    
    properties(SetAccess = immutable)
        enable          %Enable scanning
        amplitude       %Amplitude of the scan in volts
        offset          %Offset of the scan in volts
        stepTime        %Step time of the scan in seconds
        stepSize        %Step size of the scan in volts
    end
    
    properties(SetAccess = protected)
        parent          %Parent object for the scan module
        duration        %Scan duration in seconds
    end
    
    methods
        function self = LaserServoScanControl(parent,regs)
            %LASERSERVOSCANCONTROL Creates an instance of the object
            %
            %   SELF = LASERSERVOSCANCONTROL(PARENT,REGS) creates an
            %   instance SELF with parent object PARENT and registers REGS
            
            self.parent = parent;
            
            self.enable = DeviceParameter([2,2],self.parent.topReg)...
                .setLimits('lower',0,'upper',1);
            self.offset = DeviceParameter([0,15],regs(1),'int16')...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) x/self.parent.CONV_DAC,'from',@(x) x*self.parent.CONV_DAC);
            self.amplitude = DeviceParameter([16,31],regs(1),'int16')...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) x/self.parent.CONV_DAC,'from',@(x) x*self.parent.CONV_DAC);
            self.stepSize = DeviceParameter([0,15],regs(2))...
                .setLimits('lower',0,'upper',1)...
                .setFunctions('to',@(x) x/self.parent.CONV_DAC,'from',@(x) x*self.parent.CONV_DAC);
            self.stepTime = DeviceParameter([0,31],regs(3))...
                .setLimits('lower',0,'upper',self.parent.CLK^-1*(2^32 - 1))...
                .setFunctions('to',@(x) x*self.parent.CLK,'from',@(x) x/self.parent.CLK);
        end
        
        function self = setDefaults(self)
            %SETDEFAULTS Sets the default values for the scan module
            %
            %   SELF = SETDEFAULTS(SELF) sets the default values for object
            %   SELF
            self.enable.set(0);
            self.offset.set(0);
            self.amplitude.set(1);
            
            self.duration = 100e-3;
            self.setScanSteps;
        end
        
        function self = setScanSteps(self,T)
            %SETSCANSTEPS Sets the scan steps and time using an input
            %duration
            %
            %   SELF = SETSCANSTEPS(SELF,T) sets the scan steps and times
            %   using duration T.  Sets internal 'duration' property to T
            %
            %   SELF = SETSCANSTEPS(SELF) uses internal duration property
            %   to set scan steps and times
            if nargin > 1
                self.duration = T;
            end
            
            self.stepTime.set(min(0.5*self.duration/200,self.stepTime.upperLimit));
            self.stepSize.set(2*self.amplitude.get*self.stepTime.get/(0.5*self.duration));
            
        end
        
        function [t,v] = estimateScan(self)
            %ESTIMATESCAN estimates the scan times and voltages in order to
            %map time to scan value
            %
            %   [T,V] = ESTIMATESCAN(SELF) estimates the scan times T and
            %   voltages V for object SELF
            
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
            %GET Retrieves parameter values from associated registers
            %
            %   SELF = GET(SELF) Retrieves values for parameters associated
            %   with object SELF
            self.enable.get;
            self.offset.get;
            self.amplitude.get;
            self.stepTime.get;
            self.stepSize.get;
            self.duration = 2*2*self.amplitude.value/self.stepSize.value*self.stepTime.value;
        end
        
        function ss = print(self,width)
            %PRINT Prints a string representing the object
            %
            %   S = PRINT(SELF,WIDTH) returns a string S representing the
            %   object SELF with label width WIDTH.  If S is not requested,
            %   prints it to the command line
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
        
        function disp(self)
            %DISP Displays the object properties
            disp('LaserServoScanControl object with properties:');
            disp(self.print(25));
        end
    end
    
end