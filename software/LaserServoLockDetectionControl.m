classdef LaserServoLockDetectionControl < handle
    %LASERSERVOLOCKDETECTIONCONTROL Defines a class for handling the laser servo
    %lock detection
    
    properties(SetAccess = immutable)
        threshold       %Lock threshold value
        cicRate         %Log2(CIC decimation rate)
        shift           %Log2(division of filtered signals)
        cicRate2
        shift2
    end
    
    properties(SetAccess = protected)
        parent          %Parent object for the lock-in module
    end
    
    methods
        function self = LaserServoLockDetectionControl(parent,reg)
            %LASERSERVOLOCKDETECTIONCONTROL Creates an instance of the object
            %
            %   SELF = LASERSERVOLOCKDETECTIONCONTROL(PARENT,REGS) creates an
            %   instance SELF with parent object PARENT and registers REGS
            
            self.parent = parent;

            self.threshold = DeviceParameter([0,7],reg)...
                .setLimits('lower',0,'upper',255);
            
            self.cicRate = DeviceParameter([8,11],reg)...
                .setLimits('lower',2,'upper',13);

            self.shift = DeviceParameter([12,15],reg)...
                .setLimits('lower',0,'upper',16);

            self.cicRate2 = DeviceParameter([16,19],reg)...
                .setLimits('lower',2,'upper',13);

            self.shift2 = DeviceParameter([20,23],reg)...
                .setLimits('lower',0,'upper',16);
        end
        
        function self = setDefaults(self)
            %SETDEFAULTS Sets the default values for the lock-in module
            %
            %   SELF = SETDEFAULTS(SELF) sets the default values for object
            %   SELF
            
            self.threshold.set(100);
            self.cicRate.set(7);
            self.shift.set(0);
            self.cicRate2.set(10);
            self.shift2.set(0);
        end
        
        function self = get(self)
            %GET Retrieves parameter values from associated registers
            %
            %   SELF = GET(SELF) Retrieves values for parameters associated
            %   with object SELF
            self.threshold.get;
            self.cicRate.get;
            self.shift.get;
            self.cicRate2.get;
            self.shift2.get;
        end

        function ss = print(self,width)
            %PRINT Prints a string representing the object
            %
            %   S = PRINT(SELF,WIDTH) returns a string S representing the
            %   object SELF with label width WIDTH.  If S is not requested,
            %   prints it to the command line
            s{1} = self.threshold.print('Detection threshold',width,'%.0f');
            s{2} = self.cicRate.print('Log2(CIC decimation)',width,'%d');
            s{3} = self.shift.print('Log2(Div. filt. signals)',width,'%d');
            s{4} = self.cicRate2.print('Log2(CIC decimation) 2',width,'%d');
            s{5} = self.shift2.print('Log2(Div. filt. signals 2)',width,'%d');
            
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
            disp('LaserServoLockDetectionControl object with properties:');
            disp(self.print(25));
        end
        
        function s = struct(self)
            %STRUCT Creates a struct from the object
            s.threshold = self.threshold.struct;
            s.cicRate = self.cicRate.struct;
            s.shift = self.shift.struct;
            s.cicRate2 = self.cicRate2.struct;
            s.shift2 = self.shift2.struct;
        end
        
        function self = loadstruct(self,s)
            %LOADSTRUCT Loads a struct into the object
            self.threshold.set(s.threshold.value);
            self.cicRate.set(s.cicRate.value);
            self.shift.set(s.shift.value);
            self.cicRate2.set(s.cicRate2.value);
            self.shift2.set(s.shift2.value);
        end
        
    end
    
end