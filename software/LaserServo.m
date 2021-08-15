classdef LaserServo < handle
    %LASERSERVO Defines a class for handling control of the laser servo
    %design
    properties
        jumpers
        t
        data
    end
    
    properties(SetAccess = immutable)
        conn                %Instance of DPFeedbackClient used for communication with socket server
        conn2
        
        log2Avgs            %Log2 of the number of averages on initial filter
        pid                 %PID settings
        scan                %Scan settings
        fifoRoute           %Routing settings for FIFO
        sampleTime          %FIFO sample time
        
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
        ADC_WIDTH = 14;
        DAC_WIDTH = 14;
        CONV_LV = 1.1851/2^(LaserServo.DAC_WIDTH - 1);
        CONV_HV = 29.3570/2^(LaserServo.DAC_WIDTH - 1);
        
        FIFO_ROUTE_TABLE = {'adc1','adc2','scan','pid1','pid2','act1','act2'};
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
            if numel(varargin) == 1
                self.conn = ConnectionClient(varargin{1});
                self.conn2 = ConnectionClient(varargin{1},6667);
            else
                self.conn = ConnectionClient(self.HOST_ADDRESS);
                self.conn2 = ConnectionClient(self.HOST_ADDRESS,6667);
            end
            
            self.jumpers = 'lv';
            
            
            % R/W registers
            self.trigReg = DeviceRegister('0',self.conn);
            self.topReg = DeviceRegister('4',self.conn);
            self.filtReg = DeviceRegister('8',self.conn);
            self.pidRegs = DeviceRegister.empty;
            for nn = 0:3
                self.pidRegs(1,nn + 1) = DeviceRegister(hex2dec('0c') + nn*4,self.conn);
                self.pidRegs(2,nn + 1) = DeviceRegister(hex2dec('1c') + nn*4,self.conn);
            end
            
            self.scanRegs = DeviceRegister('2C',self.conn);
            self.scanRegs(2) = DeviceRegister('30',self.conn);
            self.scanRegs(3) = DeviceRegister('34',self.conn);
            self.fifoReg = DeviceRegister('38',self.conn);

            
            %Initial filtering
            self.log2Avgs = DeviceParameter([0,3],self.filtReg)...
                .setLimits('lower',0,'upper',2^16);
            %PID settings
            self.pid = LaserServoPID(self,self.pidRegs(1,:));
            self.pid(2) = LaserServoPID(self,self.pidRegs(2,:));
            %Scan settings
            self.scan = LaserServoScanControl(self,self.scanRegs);
            %FIFO settings
            self.fifoRoute = DeviceParameterString([0,3],self.fifoReg)...
                .setFunctions('to',@(x) convert_fifo_route(x,'int'),'from',@(x) convert_fifo_route(x,'string'));
            self.fifoRoute(2) = DeviceParameterString([4,7],self.fifoReg)...
                .setFunctions('to',@(x) convert_fifo_route(x,'int'),'from',@(x) convert_fifo_route(x,'string'));
            self.sampleTime = DeviceParameter([8,23],self.fifoReg)...
                .setLimits('lower',0,'upper',2^24-1)...
                .setFunctions('to',@(x) x*LaserServo.CLK,'from',@(x) x/LaserServo.CLK);
            
        end
        
        function self = setDefaults(self,varargin)
            %SETDEFAULTS Sets parameter values to their defaults
            %
            %   FB = FB.SETDEFAULTS() sets default values for FB
            self.log2Avgs.set(4);
            self.pid.setDefaults;
            self.scan.setDefaults;
            self.fifoRoute(1).set('adc1');
            self.fifoRoute(2).set('scan');
            self.setSampleTime;
        end
        
        function self = setSampleTime(self)
            self.sampleTime.set(0.5*self.scan.duration/self.MAX_REAL_TIME_DATA);
        end
        
        function self = check(self)
            %CHECK Checks parameter values and makes sure that they are
            %within acceptable ranges.  Throws errors if they are not


        end
        
        function r = dt(self)
            r = 2^(self.log2Avgs.value)/self.CLK;
        end
        
        function self = autoset(self)
            self.scan.setScanSteps;
            self.setSampleTime;
        end
        
        function self = upload(self)
            %UPLOAD Uploads register values to the device
            %
            %   FB = FB.UPLOAD() uploads register values associated with
            %   object FB
            self.check;
%             self.topReg.write;
%             self.filtReg.write;
%             self.pidRegs.write;
%             self.scanRegs.write;
%             self.fifoReg.write;

            d = [self.topReg.getWriteData;
                 self.filtReg.getWriteData;
                 self.pidRegs.getWriteData;
                 self.scanRegs.getWriteData;
                 self.fifoReg.getWriteData];
            d = d';
            d = d(:);
            self.conn.write(d,'mode','write');
        end
        
        function self = fetch(self)
            %FETCH Retrieves parameter values from the device
            %
            %   FB = FB.FETCH() retrieves values and stores them in object
            %   FB
            %Read registers
%             self.topReg.read;
%             self.filtReg.read;
%             self.pidRegs.read;
%             self.scanRegs.read;
%             self.fifoReg.read;

            d = [self.topReg.getReadData;
                 self.filtReg.getReadData;
                 self.pidRegs(1,:).getReadData;
                 self.pidRegs(2,:).getReadData;
                 self.scanRegs.getReadData;
                 self.fifoReg.getReadData];
            self.conn.write(d,'mode','read');
            value = self.conn.recvMessage;
            self.topReg.value = value(1);
            self.filtReg.value = value(2);
            for nn = 1:size(self.pidRegs,2)
                self.pidRegs(1,nn).value = value(2 + nn);
                self.pidRegs(2,nn).value = value(6 + nn);
            end
            
            self.scanRegs(1).value = value(11);
            self.scanRegs(2).value = value(12);
            self.scanRegs(3).value = value(13);
            self.fifoReg.value = value(14);

            %Read parameters
            self.log2Avgs.get;
            self.pid.get;
            self.scan.get;
            for nn = 1:numel(self.fifoRoute)
                self.fifoRoute(nn).get;
            end
            self.sampleTime.get;
            
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
            self.trigReg.set(0,[0,0]);
        end
        
        function self = getScanData(self,numSamples,resetFlag)
            if nargin < 2
                numSamples = self.MAX_REAL_TIME_DATA;
                resetFlag = 0;
            elseif nargin < 3
                resetFlag = 0;
            end
            self.conn2.write(0,'mode','get scan data','numSamples',numSamples,'reset',resetFlag);
            raw = typecast(self.conn2.recvMessage,'uint8');
            if strcmpi(self.jumpers,'hv')
                c = self.CONV_HV;
            elseif strcmpi(self.jumpers,'lv')
                c = self.CONV_LV;
            end
            self.data = self.convertData(raw,c);
            self.t = self.sampleTime.value*(0:(size(self.data,1)-1));
        end
        
        function r = convert2volts(self,x)
            if strcmpi(self.jumpers,'hv')
                c = self.CONV_HV;
            elseif strcmpi(self.jumpers,'lv')
                c = self.CONV_LV;
            end
            r = x*c;
        end
        
        function r = convert2int(self,x)
            if strcmpi(self.jumpers,'hv')
                c = self.CONV_HV;
            elseif strcmpi(self.jumpers,'lv')
                c = self.CONV_LV;
            end
            r = x/c;
        end
        
        function disp(self)
            %DISP Displays information about the object
            strwidth = 25;
            fprintf(1,'LaserServo object with properties:\n');
            fprintf(1,'\t Registers\n');
            self.topReg.makeString('topReg',strwidth);
            self.filtReg.makeString('filtReg',strwidth);
            self.pidRegs.makeString('pidRegs',strwidth);
            self.scanRegs.makeString('scanRegs',strwidth);
            self.fifoReg.makeString('fifoReg',strwidth);
            fprintf(1,'\t ----------------------------------\n');
            fprintf(1,'\t Filtering Parameters\n');
            self.log2Avgs.print('log2Avgs',strwidth,'%d');
            fprintf(1,'\t ----------------------------------\n');
            fprintf(1,'\t PID1 Parameters\n');
            self.pid(1).print(strwidth);
            fprintf(1,'\t ----------------------------------\n');
            fprintf(1,'\t PID2 Parameters\n');
            self.pid(2).print(strwidth);
            fprintf(1,'\t ----------------------------------\n');
            fprintf(1,'\t Scan Parameters\n');
            self.scan.print(strwidth);
            fprintf(1,'\t ----------------------------------\n');
            fprintf(1,'\t FIFO Parameters\n');
            self.fifoRoute(1).print('FIFO Routing 1',strwidth,'%s');
            self.fifoRoute(2).print('FIFO Routing 2',strwidth,'%s');
            self.sampleTime.print('Sample time',strwidth,'%.3e','s');
            
        end
        
        function s = struct(self)
            %STRUCT Returns a structure representing the data
            %
            %   S = STRUCT(SELF) Returns structure S from current
            %   object SELF
            
            s.log2Avgs = self.log2Avgs;
            
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
        
        function v = convertData(raw,c)
            Nraw = numel(raw);
            d = zeros(Nraw/4,2,'int16');
            
            mm = 1;
            for nn = 1:4:Nraw
                d(mm,1) = typecast(uint8(raw(nn + (0:1))),'int16');
                d(mm,2) = typecast(uint8(raw(nn + (2:3))),'int16');
                mm = mm + 1;
            end
            
            v = double(d)*c;
        end
    end
    
end

function r = convert_fifo_route(x,method)

if strcmpi(method,'int')
    for nn = 1:numel(LaserServo.FIFO_ROUTE_TABLE)
        if strcmpi(x,LaserServo.FIFO_ROUTE_TABLE{nn})
            r = nn - 1;
            return;
        end
    end
elseif strcmpi(method,'string')
    r = LaserServo.FIFO_ROUTE_TABLE{x+1};
else
    error('Unknown conversion direction ''%s''',method);
end

end