classdef LaserServoPID < handle
    %LASERSERVOPID Defines a class for handling the PID modules in the
    %laser servo
    
    properties(SetAccess = immutable)
        Kp              %Proportional gain value
        Ki              %Integral gain value
        Kd              %Derivative gain value
        divisor         %Overall divisor for gain values to convert to fractions
        polarity        %Polarity of PID module
        enable          %Enable/disable PID module
        scanEnable      %Enable scan of output
        control         %Control/set-point of the module
        lowerLimit      %Lower output limit for the module
        upperLimit      %Upper output limit for the module
    end
    
    properties(SetAccess = protected)
        parent          %Parent object
    end
    
    methods
        function self = LaserServoPID(parent,regs)
            %LASERSERVOPID Creates an instance of the class
            %
            %   SELF = LASERSERVOPID(PARENT,REGS) Creates instance SELF
            %   with parent object PARENT and associated with registers
            %   REGS
            
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
            self.Kp = DeviceParameter([0,7],regs(2))...
                .setLimits('lower',0,'upper',2^8-1);
            self.Ki = DeviceParameter([8,15],regs(2))...
                .setLimits('lower',0,'upper',2^8-1);
            self.Kd = DeviceParameter([16,23],regs(2))...
                .setLimits('lower',0,'upper',2^8-1);
            self.divisor = DeviceParameter([24,31],regs(2))...
                .setLimits('lower',0,'upper',2^8-1);
            self.lowerLimit = DeviceParameter([0,15],regs(3),'int16')...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) x/self.parent.CONV_DAC,'from',@(x) x*self.parent.CONV_DAC);
            self.upperLimit = DeviceParameter([16,31],regs(3),'int16')...
                .setLimits('lower',-1,'upper',1)...
                .setFunctions('to',@(x) x/self.parent.CONV_DAC,'from',@(x) x*self.parent.CONV_DAC);
        end
        
        function self = setDefaults(self)
            %SETDEFAULTS Sets the default values for the module
            %
            %   SELF = SETDEFAULTS(SELF) sets the default values of object
            %   SELF
            
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
            %GET Retrieves parameter values from associated registers
            %
            %   SELF = GET(SELF) Retrieves values for parameters associated
            %   with object SELF
            
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
            %CALCULATEREALGAINS Calculates the "real",
            %continuous-controller equivalent gains
            %
            %   [Kp,Ki,Kd] = CALCULATEREALGAINS(SELF) Calculates the real
            %   gains Kp, Ki, and Kd using the set DIVISOR value and the
            %   parent object's sampling interval
            
            Kp = self.Kp.value*2^(-self.divisor.value);
            Ki = self.Ki.value*2^(-self.divisor.value)/self.parent.dt();
            Kd = self.Kd.value*2^(-self.divisor.value)*self.parent.dt();
        end
        
        function ss = print(self,width)
            %PRINT Prints a string representing the object
            %
            %   S = PRINT(SELF,WIDTH) returns a string S representing the
            %   object SELF with label width WIDTH.  If S is not requested,
            %   prints it to the command line
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
        
        function disp(self)
            %DISP Displays the object properties
            disp('LaserServoPID object with properties:');
            disp(self.print(25));
        end
        
        function s = struct(self)
            %STRUCT Creates a struct from the object
            if numel(self) == 1
                s.Kp = self.Kp.struct;
                s.Ki = self.Ki.struct;
                s.Kd = self.Kd.struct;
                s.divisor = self.divisor.struct;
                s.polarity = self.polarity.struct;
                s.enable = self.enable.struct;
                s.scanEnable = self.scanEnable.struct;
                s.control = self.control.struct;
                s.lowerLimit = self.lowerLimit.struct;
                s.upperLimit = self.upperLimit.struct;
            else
                for nn = 1:numel(self)
                    s(nn) = self(nn).struct;
                end
            end
        end
        
        function self = loadstruct(self,s)
            %LOADSTRUCT Loads a struct into the object
            if numel(self) == 1
                self.Kp.set(s.Kp.value);
                self.Ki.set(s.Ki.value);
                self.Kd.set(s.Kd.value);
                self.divisor.set(s.divisor.value);
                self.polarity.set(s.polarity.value);
                self.enable.set(s.enable.value);
                self.scanEnable.set(s.scanEnable.value);
                self.control.set(s.control.value);
                self.lowerLimit.set(s.lowerLimit.value);
                self.upperLimit.set(s.upperLimit.value);
            else
                for nn = 1:numel(self)
                    self(nn).loadstruct(s(nn));
                end
            end
        end
        
    end
    
end