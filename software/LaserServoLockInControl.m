classdef LaserServoLockInControl < LaserServoSubModule
    %LASERSERVOLOCKINCONTROL Defines a class for handling the laser servo
    %lock-in detector
    
    properties(SetAccess = immutable)
        driveFreq       %Driving frequency
        demodPhase      %Demodulation phase
        cicRate         %Log2(CIC decimation rate)
        shift           %Log2(division of filtered signals)
        driveAmp        %Driving amplitude as multiplier
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
                .setLimits('lower',0,'upper',50e6)...
                .setFunctions('to',@(x) x/self.parent.CLK*2^(self.DDS_WIDTH),'from',@(x) x*self.parent.CLK/2^(self.DDS_WIDTH));
            
            self.demodPhase = DeviceParameter([0,26],regs(2))...
                .setLimits('lower',-360,'upper',360)...
                .setFunctions('to',@(x) mod(x,360)/360*2^(self.DDS_WIDTH),'from',@(x) x*360/2^(self.DDS_WIDTH));
            
            self.cicRate = DeviceParameter([8,11],regs(3))...
                .setLimits('lower',2,'upper',13);

            self.shift = DeviceParameter([12,15],regs(3))...
                .setLimits('lower',0,'upper',16);
            
            self.driveAmp = DeviceParameter([0,7],regs(3))...
                .setLimits('lower',0,'upper',1)...
                .setFunctions('to',@(x) x*255,'from',@(x) x/255);
        end
        
        function self = setDefaults(self)
            %SETDEFAULTS Sets the default values for the lock-in module
            %
            %   SELF = SETDEFAULTS(SELF) sets the default values for object
            %   SELF
            
            self.driveFreq.set(3e6);
            self.demodPhase.set(0);
            self.cicRate.set(7);
            self.shift.set(12);
            self.driveAmp.set(1);
        end

        function ss = print(self,width)
            %PRINT Prints a string representing the object
            %
            %   S = PRINT(SELF,WIDTH) returns a string S representing the
            %   object SELF with label width WIDTH.  If S is not requested,
            %   prints it to the command line
            s{1} = self.driveFreq.print('Drive frequency [Hz]',width,'%.3e');
            s{2} = self.demodPhase.print('Demod. phase [deg]',width,'%.3f');
            s{3} = self.cicRate.print('Log2(CIC decimation)',width,'%d');
            s{4} = self.shift.print('Log2(Div. filt. signals)',width,'%d');
            s{5} = self.driveAmp.print('Drive amplitude',width,'%.3f');
            
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