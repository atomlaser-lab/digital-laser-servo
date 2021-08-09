classdef LaserServo < handle
    %LASERSERVO Defines a class for handling control of the laser servo
    %design
    properties

    end
    
    properties(SetAccess = immutable)
        conn                %Instance of DPFeedbackClient used for communication with socket server
        
        log2Avgs            %Log2 of the number of averages on initial filter
        pid                 %PID settings
        scan                %Scan settings
        skipWidth           %FIFO skip width to maintain constant data size
        
    end
    
    properties(SetAccess = protected)
        trigReg             %Register for software trigger signals
        topReg              %Register for shared top-level parameters
        filtReg             %Register for initial filtering
        pidRegs             %Registers for PID controllers
        scanRegs            %Registers for scan control
        
        fifoReg             %Register for FIFO
        
    end
    
    properties(Constant)
        CLK = 125e6;                    %Clock frequency of the board
        MAX_REAL_TIME_DATA = 4096;      %Maximum number of points to transfer in real-time
        HOST_ADDRESS = '';              %Default socket server address
    end
    
    methods
        function self = LaserServo(varargin)
            %LASERSERVO Creates an instance of a LASERSERVO object.  Sets
            %up the registers and parameters as instances of the correct
            %classes with the necessary
            %addressses/registers/limits/functions
            %
            %   LS = LASERSERVO(HOST) creates an instance with socket
            %   server host address HOST
            if numel(varargin)==1
                self.conn = DPFeedbackClient(varargin{1});
            else
                self.conn = DPFeedbackClient(self.HOST_ADDRESS);
            end
            
            
            % R/W registers
            self.trigReg = DeviceRegister('0',self.conn);
            self.topReg = DeviceRegister('4',self.conn);
            self.filtReg = DeviceRegister('8',self.conn);
            for nn = 1:4
                self.pidRegs(1,nn) = DeviceRegister(8+nn*4,self.conn);
                self.pidRegs(2,nn) = DeviceRegister(24+nn*4,self.conn);
            end
            
            self.scanRegs = DeviceRegister('2C',self.conn);
            self.scanRegs(2) = DeviceRegister('30',self.conn);
            self.fifoReg = DeviceRegister('34',self.conn);

            
            %Initial filtering
            self.log2Avgs = DeviceParameter([31,28],self.filtReg)...
                .setLimits('lower',0,'upper',2^16);
            %PID settings
            self.pid(1) = LaserServoPID(self,self.pidRegs(1,:));
            self.pid(2) = LaserServoPID(self,self.pidRegs(2,:));
            %Scan settings
            self.scan = LaserServoScanControl(self,self.scanRegs);
            %FIFO settings
            self.skipWidth = DeviceParameter([15,0],self.fifoReg)...
                .setLimits('lower',0,'upper',2^16-1);
            
        end
        
        function self = setDefaults(self,varargin)
            %SETDEFAULTS Sets parameter values to their defaults
            %
            %   FB = FB.SETDEFAULTS() sets default values for FB
            self.log2Avgs.set(4);
            self.pid.setDefaults;
            self.scan.setDefaults;
            self.setSkipWidth;
        end
        
        function self = setSkipWidth(self)
            dt = 2^(self.log2Avgs.value)/self.CLK;
            Npoints = self.scan.duration/dt;
            if Npoints > self.MAX_REAL_TIME_DATA
                self.skipWidth.set(self.scan.duration/(self.MAX_REAL_TIME_DATA*dt));
            end
        end
        
        function self = check(self)
            %CHECK Checks parameter values and makes sure that they are
            %within acceptable ranges.  Throws errors if they are not


        end
        
        function self = upload(self)
            %UPLOAD Uploads register values to the device
            %
            %   FB = FB.UPLOAD() uploads register values associated with
            %   object FB
            self.check;
            self.topReg.write;
            self.filtReg.write;
            self.pidRegs.write;
            self.scanRegs.write;
            self.fifoReg.write;
            
        end
        
        function self = fetch(self)
            %FETCH Retrieves parameter values from the device
            %
            %   FB = FB.FETCH() retrieves values and stores them in object
            %   FB
            %Read registers
            self.topReg.read;
            self.filtReg.read;
            self.pidRegs.read;
            self.scanRegs.read;
            self.fifoReg.read;

            %Read parameters
            self.log2Avgs.get;
            self.pid.get;
            self.scan.get;
            self.skipWidth.get;
            
        end
        
        function self = start(self)
            %START Sends a software-based start trigger to the device
            %
            %   FB = FB.START() sends a start trigger associated with
            %   object FB
            
        end
        
        function self = reset(self)
            %RESET Resets the device
            %
            %   FB = FB.RESET() resets the device associated with object FB
            self.trigReg.set(1,[0,0]).write;
            self.trigReg.set(0,[0,0]).write;
        end
        
        function disp(self)
            %DISP Displays information about the object
            strwidth = 36;
            fprintf(1,'LaserServo object with properties:\n');
            fprintf(1,'\t Registers\n');
            self.topReg.makeString('topReg',strwidth);
            self.filtReg.makeString('filtReg',strwidth);
            self.pidRegs.makeString('pidRegs',strwidth);
            self.scanRegs.makeString('scanRegs',strwidth);
            self.fifoReg.makeString('fifoReg',strwidth);
            
        end
        
        function s = struct(self)
            %STRUCT Returns a structure representing the data
            %
            %   S = STRUCT(SELF) Returns structure S from current
            %   object SELF
            
            
            
        end

        function s = saveobj(self)
            %SAVEOBJ Returns a structure used for saving data
            %
            %   S = SAVEOBJ(SELF) Returns structure S used for saving
            %   data representing object SELF
            s = self.struct;
        end
        
    end
    
    methods(Static)
        function self = loadobj(s)
            %LOADOBJ Creates a DPFEEDBACK object using input structure
            %
            %   SELF = LOADOBJ(S) uses structure S to create new DPFEEDBACK
            %   object SELF
            
            
        end
    end
    
end