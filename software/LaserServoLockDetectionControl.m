classdef LaserServoLockDetectionControl < LaserServoSubModule
    %LASERSERVOLOCKDETECTIONCONTROL Defines a class for handling the laser servo
    %lock detection
    
    properties(SetAccess = immutable)
        threshold       %Lock threshold value
        cicRates        %Log2(CIC decimation rate) for stages 1 and 2
        shifts          %Log2(division of filtered signals) for stages 1 and 2
        power           %Current power measurement
        detected        %Is lock detected?
    end

    properties(Constant)
        NUM_FILT_STAGES = 2;
    end
    
    methods
        function self = LaserServoLockDetectionControl(parent,regs)
            %LASERSERVOLOCKDETECTIONCONTROL Creates an instance of the object
            %
            %   SELF = LASERSERVOLOCKDETECTIONCONTROL(PARENT,REGS) creates an
            %   instance SELF with parent object PARENT and registers REGS
            
            self.parent = parent;

            self.cicRates = DeviceParameter([0,3],regs(1))...
                .setLimits('lower',2,'upper',13);

            self.shifts = DeviceParameter([4,7],regs(1),'int8')...
                .setLimits('lower',-8,'upper',7);

            self.cicRates(2) = DeviceParameter([8,11],regs(1))...
                .setLimits('lower',2,'upper',13);

            self.shifts(2) = DeviceParameter([12,15],regs(1),'int8')...
                .setLimits('lower',-8,'upper',7);

            self.threshold = DeviceParameter([16,31],regs(1))...
                .setLimits('lower',0,'upper',2^16 - 1);

            self.power = DeviceParameter([0,15],regs(2));
            self.detected = DeviceParameter([31,31],regs(2));
        end
        
        function self = setDefaults(self)
            %SETDEFAULTS Sets the default values for the lock-in module
            %
            %   SELF = SETDEFAULTS(SELF) sets the default values for object
            %   SELF
            
            self.threshold.set(100);
            self.cicRates(1).set(10);
            self.shifts(1).set(0);
            self.cicRates(2).set(5);
            self.shifts(2).set(0);
        end

        function ss = print(self,width)
            %PRINT Prints a string representing the object
            %
            %   S = PRINT(SELF,WIDTH) returns a string S representing the
            %   object SELF with label width WIDTH.  If S is not requested,
            %   prints it to the command line
            s{1} = self.threshold.print('Detection threshold',width,'%.0f');
            s{2} = self.cicRates(1).print('Log2(CIC decimation)',width,'%d');
            s{3} = self.shifts(1).print('Log2(Div. filt. signals)',width,'%d');
            s{4} = self.cicRates(2).print('Log2(CIC decimation) 2',width,'%d');
            s{5} = self.shifts(2).print('Log2(Div. filt. signals 2)',width,'%d');
            s{6} = self.power.print('Lock detection power',width,'%d');
            s{7} = self.detected.print('Lock detected?',width,'%d');

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
        
    end
    
end