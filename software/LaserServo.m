classdef LaserServo < handle
    %LASERSERVO Defines a class for handling control of the laser servo
    %design
    properties
        jumpers             %Jumper settings, either 'lv' or 'hv'
        t                   %Time vector for retrieved data
        data                %Retrieved data, an Nx2 array
    end
    
    properties(SetAccess = immutable)
        conn                %Instance of CONNECTIONCLIENT used for communication with socket server
        %
        % All of these are DEVICEPARAMETER objects
        %
        inputSelect         %2-bit selector for what signal to use for locking
        outputSelect        %2-bit selector for what outputs to generate
        log2Avgs            %Log2 of the number of averages on initial filter
        pid                 %PID settings, a 2 element array
        scan                %Scan settings
        lockin              %Lock-in settings
        fifoRoute           %Routing settings for FIFO, a 2 element array
        sampleTime          %FIFO sample time
        
    end
    
    properties(SetAccess = protected)
        trigReg             %Register for software trigger signals
        topReg              %Register for shared top-level parameters
        filtReg             %Register for initial filtering
        pidRegs             %Registers for PID controllers
        scanRegs            %Registers for scan control
        
        fifoReg             %Register for FIFO
        
        lockinRegs          %Registers for lock-in detection
        
    end
    
    properties(Constant)
        CLK = 125e6;                    %Clock frequency of the board
        MAX_REAL_TIME_DATA = 4096;      %Maximum number of points to transfer in real-time
        DEFAULT_HOST = '';              %Default socket server address
        DEFAULT_PORT = 6666;            %Default port of socket server
        ADC_WIDTH = 14;                 %Bit width of ADC values
        DAC_WIDTH = 14;                 %Bit width of DAC values
        %
        % Conversion values going from integer values to volts
        %
        CONV_ADC_LV = 1.1851/2^(LaserServo.ADC_WIDTH - 1);
        CONV_ADC_HV = 29.3570/2^(LaserServo.ADC_WIDTH - 1);
        CONV_DAC = 1.079/2^(LaserServo.ADC_WIDTH - 1);
        
        FIFO_ROUTE_TABLE = {'adc1','adc2','scan','pid1','pid2','out1','out2','demod1','demod2'};
    end
    
    methods
        function self = LaserServo(host,port)
            %LASERSERVO Creates an instance of a LASERSERVO object.  Sets
            %up the registers and parameters as instances of the correct
            %classes with the necessary
            %addressses/registers/limits/functions
            %
            %   SELF = LASERSERVO() creates an instance with default host
            %   and port
            %
            %   SELF = LASERSERVO(HOST) creates an instance with socket
            %   server host address HOST
            %
            %   SELF = LASERSERVO(HOST,PORT) creates an instance with
            %   socket server host address HOST and port PORT
            
            if nargin == 0
                self.conn = ConnectionClient(self.DEFAULT_HOST,self.DEFAULT_PORT);
            elseif nargin == 1
                self.conn = ConnectionClient(host,self.DEFAULT_PORT);
            else
                self.conn = ConnectionClient(host,port);
            end
            %
            % Set jumper values
            %
            self.jumpers = 'lv';
            %
            % R/W registers
            %
            self.trigReg = DeviceRegister('0',self.conn);
            self.topReg = DeviceRegister('4',self.conn);
            self.filtReg = DeviceRegister('8',self.conn);
            %
            % There are 3 PID registers PER PID
            %
            self.pidRegs = DeviceRegister.empty;
            for nn = 0:2
                self.pidRegs(1,nn + 1) = DeviceRegister(hex2dec('0c') + nn*4,self.conn);
                self.pidRegs(2,nn + 1) = DeviceRegister(hex2dec('1c') + nn*4,self.conn);
            end
            %
            % There are 3 scan registers
            %
            self.scanRegs = DeviceRegister('2C',self.conn);
            self.scanRegs(2) = DeviceRegister('30',self.conn);
            self.scanRegs(3) = DeviceRegister('34',self.conn);
            %
            % There is one FIFO register
            %
            self.fifoReg = DeviceRegister('38',self.conn);
            %
            % There are four lock-in registers from 0x40 to 0x4C
            %
            self.lockinRegs = DeviceRegister.empty;
            for nn = 1:4
                self.lockinRegs(nn) = DeviceRegister(hex2dec('40') + nn*4,self.conn);
            end
            %
            % Input selector and top-level settings
            %
            self.inputSelect = DeviceParameter([0,1],self.topReg)...
                .setLimits('lower',0,'upper',3);
            self.outputSelect = DeviceParameter([2,3],self.topReg)...
                .setLimits('lower',0,'upper',3);
            % 
            % Initial filtering
            %
            self.log2Avgs = DeviceParameter([0,3],self.filtReg)...
                .setLimits('lower',0,'upper',2^16);
            %
            % PID settings
            %
            self.pid = LaserServoPID(self,self.pidRegs(1,:));
            self.pid(2) = LaserServoPID(self,self.pidRegs(2,:));
            %
            % Scan settings
            %
            self.scan = LaserServoScanControl(self,self.scanRegs);
            %
            % FIFO settings
            %
            self.fifoRoute = DeviceParameter([0,3],self.fifoReg)...
                .setFunctions('to',@(x) convert_fifo_route(x,'int'),'from',@(x) convert_fifo_route(x,'string'));
            self.fifoRoute(2) = DeviceParameter([4,7],self.fifoReg)...
                .setFunctions('to',@(x) convert_fifo_route(x,'int'),'from',@(x) convert_fifo_route(x,'string'));
            self.sampleTime = DeviceParameter([8,31],self.fifoReg)...
                .setLimits('lower',0,'upper',2^24-1)...
                .setFunctions('to',@(x) x*LaserServo.CLK,'from',@(x) x/LaserServo.CLK);
            %
            % Lock-in settings
            %
            self.lockin = LaserServoLockInControl(self,self.lockinRegs);
        end
        
        function self = setDefaults(self,varargin)
            %SETDEFAULTS Sets parameter values to their defaults
            %
            %   SELF = SETDEFAULTS(SELF) sets default values for SELF
            self.inputSelect.set(1);
            self.log2Avgs.set(4);
            self.pid.setDefaults;
            self.scan.setDefaults;
            self.lockin.setDefaults;
            self.fifoRoute(1).set('adc1');
            self.fifoRoute(2).set('scan');
            self.setSampleTime;
        end
        
        function self = setSampleTime(self)
            %SETSAMPLETIME Automatically sets the FIFO sampling time
            %
            %   SELF = SETSAMPLETIME(SELF) sets the FIFO sampling time for
            %   LASERSERVO object SELF.  It uses the scan parameters to
            %   calculate the time between samples such that samples are
            %   only acquired during the positive scan ramp
            
            duration = 2*2*self.scan.amplitude.get/self.scan.stepSize.get*self.scan.stepTime.get;
            dtt = 0.5*duration/self.MAX_REAL_TIME_DATA;
            dtt = floor(dtt*self.CLK)/self.CLK;
            self.sampleTime.set(dtt);
        end
        
        function self = check(self)
            %CHECK Checks parameter values and makes sure that they are
            %within acceptable ranges.  Throws errors if they are not


        end
        
        function r = dt(self)
            %DT Returns the current sampling time based on the filter
            %settings
            %
            %   R = DT(SELF) returns sampling time R for LASERSERVO object
            %   SELF
            r = 2^(self.log2Avgs.value)/self.CLK;
        end
        
        function self = autoset(self)
            %AUTOSET Automatically sets scan step parameters and FIFO
            %sample times
            self.scan.setScanSteps;
            self.setSampleTime;
        end
        
        function self = upload(self)
            %UPLOAD Uploads register values to the device
            %
            %   SELF = UPLOAD(SELF) uploads register values associated with
            %   object SELF
            
            %
            % Check parameters first
            %
            self.check;
            %
            % Get all write data
            %
            d = [self.topReg.getWriteData;
                 self.filtReg.getWriteData;
                 self.pidRegs.getWriteData;
                 self.scanRegs.getWriteData;
                 self.fifoReg.getWriteData;
                 self.lockinRegs.getWriteData];
            d = d';
            d = d(:);
            %
            % Write every register using the same connection
            %
            self.conn.write(d,'mode','write');
        end
        
        function self = fetch(self)
            %FETCH Retrieves parameter values from the device
            %
            %   SELF = FETCH(SELF) retrieves values and stores them in
            %   object SELF
            
            %
            % Get addresses to read from for each register and get data
            % from device
            %
            d = [self.topReg.getReadData;
                 self.filtReg.getReadData;
                 self.pidRegs(1,:).getReadData;
                 self.pidRegs(2,:).getReadData;
                 self.scanRegs.getReadData;
                 self.fifoReg.getReadData;
                 self.lockinRegs.getReadData];
            self.conn.write(d,'mode','read');
            value = self.conn.recvMessage;
            %
            % Parse the received data in the same order as the addresses
            % were written
            %
            self.topReg.value = value(1);
            self.filtReg.value = value(2);
            for nn = 1:size(self.pidRegs,2)
                self.pidRegs(1,nn).value = value(2 + nn);
                self.pidRegs(2,nn).value = value(5 + nn);
            end
            self.scanRegs(1).value = value(9);
            self.scanRegs(2).value = value(10);
            self.scanRegs(3).value = value(11);
            self.fifoReg.value = value(12);
            self.lockinRegs(1).value = value(13);
            self.lockinRegs(2).value = value(14);
            self.lockinRegs(3).value = value(15);
            %
            % Read parameters from registers
            %
            self.inputSelect.get;
            self.log2Avgs.get;
            self.pid.get;
            self.scan.get;
            self.lockin.get;
            for nn = 1:numel(self.fifoRoute)
                self.fifoRoute(nn).get;
            end
            self.sampleTime.get;
            
        end
        
        function self = reset(self)
            %RESET Resets the FIFOs on the device
            %
            %   SELF = RESET(SELF) resets the FIFOs on the device SELF
            self.trigReg.set(1,[0,0]).write;
            self.trigReg.set(0,[0,0]);
        end
        
        function self = getScanData(self,numSamples,resetFlag)
            %GETSCANDATA Retrieves the scan data from the device
            %
            %   SELF = GETSCANDATA(SELF) retrieves
            %   LASERSERVO.MAX_REAL_TIME_DATA samples from the device
            %
            %   SELF = GETSCANDATA(SELF,N) retrieves N samples from the
            %   device
            %
            %   SELF = GETSCANDATA(__,RESETFLAG) sets the reset flag to
            %   RESETFLAG, which if true will reset the FIFO before
            %   collecting data
            
            if nargin < 2
                numSamples = self.MAX_REAL_TIME_DATA;
                resetFlag = 0;
            elseif nargin < 3
                resetFlag = 0;
            end
            self.conn.write(0,'mode','get scan data','numSamples',numSamples,'reset',resetFlag);
            raw = typecast(self.conn.recvMessage,'uint8');
            %
            % Convert data to correct units
            %
            v = self.convertData(raw);
            if strcmpi(self.jumpers,'lv')
                c = self.CONV_ADC_LV;
            elseif strcmpi(self.jumpers,'hv')
                c = self.CONV_ADC_HV;
            end
            %
            % Loop through channels
            %
            for nn = 1:size(v,2)
                if any(strcmpi(self.fifoRoute(nn).value,{'adc1','adc2'}))
                    self.data(:,nn) = c*v(:,nn);
                else
                    self.data(:,nn) = self.CONV_DAC*v(:,nn);
                end
            end
            %
            % Create time vectors
            %
            self.t = self.sampleTime.value*(0:(size(self.data,1)-1))';
        end
        
        function r = convert2volts(self,x)
            %CONVERT2VOLTS Converts an ADC value to volts
            %
            %   R = CONVERT2VOLTS(SELF,X) converts value X to volts R
            if strcmpi(self.jumpers,'hv')
                c = self.CONV_ADC_HV;
            elseif strcmpi(self.jumpers,'lv')
                c = self.CONV_ADC_LV;
            end
            r = x*c;
        end
        
        function r = convert2int(self,x)
            %CONVERT2INT Converts an ADC voltage to integer values
            %
            %   R = CONVERT2INT(SELF,X) converts voltage X to integer R
            if strcmpi(self.jumpers,'hv')
                c = self.CONV_ADC_HV;
            elseif strcmpi(self.jumpers,'lv')
                c = self.CONV_ADC_LV;
            end
            r = x/c;
        end
        
        function disp(self)
            %DISP Displays information about the object
            strwidth = 25;
            fprintf(1,'LaserServo object with properties:\n');
            fprintf(1,'\t Registers\n');
            self.topReg.print('topReg',strwidth);
            self.filtReg.print('filtReg',strwidth);
            self.pidRegs.print('pidRegs',strwidth);
            self.scanRegs.print('scanRegs',strwidth);
            self.fifoReg.print('fifoReg',strwidth);
            self.lockinRegs.print('lockinRegs',strwidth);
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
            fprintf(1,'\t ----------------------------------\n');
            fprintf(1,'\t Lock-in Parameters\n');
            self.lockin.print(strwidth);
            
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
            %CONVERTDATA Converts raw data into proper int16/double format
            %
            %   V = CONVERTDATA(RAW) Unpacks raw data from uint8 values to
            %   a pair of double values for each measurement
            %
            %   V = CONVERTDATA(RAW,C) uses conversion factor C in the
            %   conversion
            
            if nargin < 2
                c = 1;
            end
            
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
%CONVERT_FIFO_ROUTE Converts the FIFO routing string to an integer value
%and vice versa
%
%   R = CONVERT_FIFO_ROUTE(X,'int') converts string X into an integer R
%
%   R = CONVERT_FIFO_ROUTE(X,'string') converts integer X into string R

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