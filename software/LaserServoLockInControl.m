classdef LaserServoLockInControl < handle
    %LASERSERVOLOCKINCONTROL Defines a class for handling the laser servo
    %lock-in detector
    
    properties(SetAccess = immutable)
        driveFreq       %Driving frequency
        demodFreq       %Demodulation frequency
        demodPhase      %Demodulation phase
        cicRate         %Log2(CIC decimation rate)
    end
    
    properties(SetAccess = protected)
        parent          %Parent object for the lock-in module
    end
    
    properties
        lockAtMultiple  %Lock demod frequency at specified multiple
    end
    
    properties(Constant)
        DDS_WIDTH = 27; %Width of DDS phase increment
    end
    
    methods
        function self = LaserServoLockInControl(parent,regs)
            %LASERSERVOLOCKINCONTROL Creates an instance of the object
            %
            %   SELF = LASERSERVOLOCKINCONTROL(PARENT,REGS) creates an
            %   instance SELF with parent object PARENT and registers REGS
            
            self.parent = parent;
            
            self.driveFreq = DeviceParameter([0,26],regs(1))...
                .setLimits('lower',0,'upper',self.parent.CLK/(2^(self.DDS_WIDTH - 1)))...
                .setFunctions('to',@(x) x/self.parent.CLK*2^(self.DDS_WIDTH - 1),'from',@(x)  x*self.parent.CLK/2^(self.DDS_WIDTH - 1));
            
            self.demodFreq = DeviceParameter([0,26],regs(2))...
                .setLimits('lower',0,'upper',self.parent.CLK/(2^(self.DDS_WIDTH - 1)))...
                .setFunctions('to',@(x) x/self.parent.CLK*2^(self.DDS_WIDTH - 1),'from',@(x)  x*self.parent.CLK/2^(self.DDS_WIDTH - 1));
            
            self.demodPhase = DeviceParameter([0,26],regs(3))...
                .setLimits('lower',0,'upper',2*pi)...
                .setFunctions('to',@(x) x/(2*pi)*2^(self.DDS_WIDTH - 1),'from',@(x)  x*(2*pi)/2^(self.DDS_WIDTH - 1));
            
            self.cicRate = DeviceParameter([0,12],regs(4))...
                .setLimits('lower',7,'upper',13);
            
            self.lockAtMultiple = 1;
        end
        
        function self = setDefaults(self)
            %SETDEFAULTS Sets the default values for the lock-in module
            %
            %   SELF = SETDEFAULTS(SELF) sets the default values for object
            %   SELF
            
            self.driveFreq.set(3e6);
            self.demodFreq.set(3e6);
            self.demodPhase.set(0);
            self.cicRate.set(7);
            
            self.lockAtMultiple = 1;
        end
        
        function self = get(self)
            %GET Retrieves parameter values from associated registers
            %
            %   SELF = GET(SELF) Retrieves values for parameters associated
            %   with object SELF
            self.driveFreq.get;
            self.demodFreq.get;
            self.demodPhase.get;
            self.cicRate.get;
        end

        function ss = print(self,width)
            %PRINT Prints a string representing the object
            %
            %   S = PRINT(SELF,WIDTH) returns a string S representing the
            %   object SELF with label width WIDTH.  If S is not requested,
            %   prints it to the command line
            s{1} = self.driveFreq.print('Drive frequency [Hz]',width,'%.3e');
            s{2} = self.demodFreq.print('Demod. frequency [Hz]',width,'%.3e');
            s{3} = self.demodPhase.print('Demod. phase [rad]',width,'%.3f');
            s{4} = self.cicRate.print('Log2(CIC decimation',width,'%d');
            s{5} = sprintf(['% ',num2str(width),'s: %d\n'],'Fixed demodulation multiple',self.lockAtMultiple);
            
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
            disp('LaserServoLockInControl object with properties:');
            disp(self.print(25));
        end
        
    end
    
end