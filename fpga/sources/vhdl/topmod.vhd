library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;
use work.AXI_Bus_Package.all;


entity topmod is
    port (
        --
        -- Clocks and reset
        --
        sysClk          :   in  std_logic;
        adcClk          :   in  std_logic;
        aresetn         :   in  std_logic;
        --
        -- AXI-super-lite signals
        --      
        addr_i          :   in  unsigned(AXI_ADDR_WIDTH-1 downto 0);            --Address out
        writeData_i     :   in  std_logic_vector(AXI_DATA_WIDTH-1 downto 0);    --Data to write
        dataValid_i     :   in  std_logic_vector(1 downto 0);                   --Data valid out signal
        readData_o      :   out std_logic_vector(AXI_DATA_WIDTH-1 downto 0);    --Data to read
        resp_o          :   out std_logic_vector(1 downto 0);                   --Response in
        --
        -- External I/O
        --
        ext_i           :   in  std_logic_vector(7 downto 0);
        ext_o           :   out std_logic_vector(7 downto 0);
        --
        -- ADC data
        --
        adcData_i       :   in  std_logic_vector(31 downto 0);
        --
        -- DAC data
        --
        m_axis_tdata    :   out std_logic_vector(31 downto 0);
        m_axis_tvalid   :   out std_logic
    );
end topmod;


architecture Behavioural of topmod is

ATTRIBUTE X_INTERFACE_INFO : STRING;
ATTRIBUTE X_INTERFACE_INFO of m_axis_tdata: SIGNAL is "xilinx.com:interface:axis:1.0 m_axis TDATA";
ATTRIBUTE X_INTERFACE_INFO of m_axis_tvalid: SIGNAL is "xilinx.com:interface:axis:1.0 m_axis TVALID";
ATTRIBUTE X_INTERFACE_PARAMETER : STRING;
ATTRIBUTE X_INTERFACE_PARAMETER of m_axis_tdata: SIGNAL is "CLK_DOMAIN system_processing_system7_0_0_FCLK_CLK0,FREQ_HZ 125000000";
ATTRIBUTE X_INTERFACE_PARAMETER of m_axis_tvalid: SIGNAL is "CLK_DOMAIN system_processing_system7_0_0_FCLK_CLK0,FREQ_HZ 125000000";

component QuickAvg is
    port(
        clk         :   in  std_logic;          --Input clock
        aresetn     :   in  std_logic;          --Asynchronous reset
        
        reg0        :   in  t_param_reg;        --Parameters: (log2Avgs (4)) 
        
        adc_i       :   in  t_adc_array;        --Input ADC data
        valid_i     :   in  std_logic;          --Input valid signal
        adc_o       :   out t_adc_array;        --Output, averaged ADC data
        valid_o     :   out std_logic           --Indicates valid averaged data
    );
end component;

component LockInDetector is
    port(
        --
        -- Clocking and reset
        --
        clk         :   in  std_logic;
        aresetn     :   in  std_logic;
        --
        -- Control
        --
        regs_i      :   in  t_param_reg_array(3 downto 0);
        --
        -- Signal out
        --
        dac_o       :   out t_dac;    
        --
        -- Data in
        --
        data_i      :   in  t_adc;
        valid_i     :   in  std_logic;
        --
        -- Data out
        --
        data_o      :   out t_adc_array;
        valid_o     :   out std_logic_vector(1 downto 0)
    );
end component;

component PID_Controller is
	port(	
		clk			:	in	std_logic;									--Clock signal
		aresetn		:	in	std_logic;									--Asynchronous, active-low reset
		--
		-- Input signals
		--
		control_i	:	in 	t_adc;										--Control signal
		measure_i	:	in	t_adc;										--Measurement signal
		measValid_i	:	in	std_logic;									--Signal that new measurement is valid.
		scan_i		:	in	t_dac;										--Input scan value
		scanValid_i	:	in	std_logic;									--Signal that a new scan value is valid
		--
		-- Parameter inputs:
		-- 0: (0 => enable, polarity => 1)
		-- 1: 8 bit values (divisor, Kd, Ki, Kp)
		-- 2: (31 downto 16 => upper limit, 15 downto 0 => lower limit)
		--
		regs_i		:	in	t_param_reg_array(2 downto 0);
		--
		-- Outputs
		--
		pid_o		:	out t_dac;										--Actuator output from PID (debugging)
		act_o		:	out t_dac;										--Output actuator signal
		valid_o		:	out std_logic									--Indicates act_o is valid
	);								
end component;

component TriangularScan is
    port(
        clk         :   in  std_logic;          --Input clock
        aresetn     :   in  std_logic;          --Asynchronous reset
        enable_i    :   in  std_logic;          --Enable signal
        enable_o    :   out std_logic;          --Output enable signal
        --
        -- Parameter inputs:
        -- 0: (31 downto 16 => scan amplitude, 15 downto 0 => offset)
        -- 1: (31 downto 0) => step size
        -- 2: (31 downto 0) => step time
        --
        regs_i      :   in  t_param_reg_array(2 downto 0);
        
        scan_o      :   out t_dac;              --Output scan data
        polarity_o  :   out std_logic;          --Indicates the scan direction ('0' = negative, '1' = positive)
        valid_o     :   out std_logic           --Indicates valid averaged data
    );
end component;

component FIFOHandler is
    port(
        wr_clk      :   in  std_logic;
        rd_clk      :   in  std_logic;
        aresetn     :   in  std_logic;
        
        sampleTime_i:   in  std_logic_vector(23 downto 0);
        enable_i    :   in  std_logic;   
        data_i      :   in  std_logic_vector(FIFO_WIDTH-1 downto 0);
        
        fifoReset   :   in  std_logic;
        bus_m       :   in  t_fifo_bus_master;
        bus_s       :   out t_fifo_bus_slave
    );
end component;
--
-- Output signals
--
signal dac_o                        :   t_dac_array;
--
-- AXI communication signals
--
signal comState                     :   t_status                        :=  idle;
signal bus_m                        :   t_axi_bus_master                :=  INIT_AXI_BUS_MASTER;
signal bus_s                        :   t_axi_bus_slave                 :=  INIT_AXI_BUS_SLAVE;
signal reset                        :   std_logic;
--
-- Shared registers and signals
--
signal triggers                     :   t_param_reg                     :=  (others => '0');
signal topReg                       :   t_param_reg;
signal inputSignalSelect            :   std_logic_vector(1 downto 0);
signal outputSignalSelect            :   std_logic_vector(1 downto 0);
--
-- Initial filter signals
--
signal initFiltReg                  :   t_param_reg             :=  (others => '0');
signal adcFilt_i                    :   t_adc_array;
signal adcFilt_o                    :   t_adc_array;
signal filtValid_i, filtValid_o     :   std_logic;
--
-- Lock in signals
--
signal lockinRegs                   :   t_param_reg_array(3 downto 0);
signal lockin_dac_o                 :   t_dac;
signal lockin_data_i                :   t_adc;
signal lockin_data_o                :   t_adc_array;
signal lockin_valid_o               :   std_logic_vector(1 downto 0);
--
-- PID 1 settings and signals
--
signal pidRegs1, pidRegs1_i         :   t_param_reg_array(2 downto 0);
signal pidEnable1, pidScanEnable1   :   std_logic;
signal control1_i, measure1_i       :   t_adc;
signal scan1_i                      :   t_dac;
signal measValid1_i, scanValid1_i   :   std_logic;
signal error1_o                     :   t_adc;
signal pid1_o, act1_o               :   t_dac;
signal pidValid1_o                  :   std_logic;
--
-- PID 2 settings and signals
--
signal pidRegs2, pidRegs2_i         :   t_param_reg_array(2 downto 0);
signal pidEnable2, pidScanEnable2   :   std_logic;
signal control2_i, measure2_i       :   t_adc;
signal scan2_i                      :   t_dac;
signal measValid2_i, scanValid2_i   :   std_logic;
signal error2_o                     :   t_adc;
signal pid2_o, act2_o               :   t_dac;
signal pidValid2_o                  :   std_logic;
--
-- Scan settings and signals
--
signal scanRegs                     :   t_param_reg_array(2 downto 0);
signal scan_o                       :   t_dac;
signal scanValid_o                  :   std_logic;
signal scanEnable_i                 :   std_logic;
signal scanEnable_o                 :   std_logic;
signal scanEnableSet                :   std_logic;
signal scanPolarity_o               :   std_logic;
--
-- Memory signals and settings
--
type t_fifo_route is (adc1, adc2, scan, pid1, pid2, act1, act2, demod1, demod2, no_output);
type t_fifo_valid_state is (idle,wait_for_fifo1,wait_for_fifo2);
signal fifoValidState               :   t_fifo_valid_state;
signal fifoReg_o, fifoReg           :   t_param_reg;
signal fifoRoute1, fifoRoute2       :   t_fifo_route;
signal fifo1, fifo2                 :   signed(15 downto 0);
signal fifoValid1, fifoValid2       :   std_logic;
signal fifo_i, fifo_o               :   std_logic_vector(31 downto 0);
signal fifoValid_i                  :   std_logic;
signal fifoReset, fifoEnable        :   std_logic;
signal fifo_m                       :   t_fifo_bus_master;
signal fifo_s                       :   t_fifo_bus_slave;

function convert_fifo_route(s : std_logic_vector(3 downto 0)) return t_fifo_route is
--    variable s      :   std_logic_vector(3 downto 0);
    variable result :   t_fifo_route;
begin
--    s := val_i(3 downto 0);
    if s = X"0" then
        result := adc1;
    elsif s = X"1" then
        result := adc2;
    elsif s = X"2" then
        result := scan;
    elsif s = X"3" then
        result := pid1;
    elsif s = X"4" then
        result := pid2;
    elsif s = X"5" then
        result := act1;
    elsif s = X"6" then
        result := act2;
    elsif s = X"7" then
        result := demod1;
    elsif s = X"8" then
        result := demod2;
    else
        result := no_output;
    end if;
    return result;
end convert_fifo_route;

begin
--
-- Assign outputs
--
ext_o <= (others => '0');
dac_o(0) <= act1_o when outputSignalSelect(0) = '0' else lockin_dac_o;
dac_o(1) <= act2_o when outputSignalSelect(1) = '0' else lockin_dac_o;

m_axis_tdata <= std_logic_vector(dac_o(1)) & std_logic_vector(dac_o(0));
m_axis_tvalid <= '1';
--
-- Parse parameters needed for global control
--
inputSignalSelect <= topReg(1 downto 0);
outputSignalSelect <= topReg(3 downto 2);
scanEnableSet <= topReg(4);
--
-- PID registers
--
pidEnable1 <= pidRegs1(0)(0);
pidScanEnable1 <= pidRegs1(0)(2);
control1_i <= signed(pidRegs1(0)(31 downto 16));
pidEnable2 <= pidRegs2(0)(0);
pidScanEnable2 <= pidRegs2(0)(2);
control2_i <= signed(pidRegs2(0)(31 downto 16));

pidRegs1_i(0)(0) <= pidEnable1 and not(scanEnable_o);
pidRegs1_i(0)(31 downto 1) <= pidRegs1(0)(31 downto 1);
pidRegs1_i(pidRegs1_i'length - 1 downto 1) <= pidRegs1(pidRegs1'length - 1 downto 1);

pidRegs2_i(0)(0) <= pidEnable2 and not(scanEnable_o);
pidRegs2_i(0)(31 downto 1) <= pidRegs2(0)(31 downto 1);
pidRegs2_i(pidRegs2_i'length - 1 downto 1) <= pidRegs2(pidRegs2'length - 1 downto 1);

--
-- Begin with a quick-average module for initial filtering
--
adcFilt_i(0) <= signed(adcData_i(15 downto 0));
adcFilt_i(1) <= signed(adcData_i(31 downto 16));
filtValid_i <= '1';
InitFilt: QuickAvg
port map(
    clk         =>  adcClk,
    aresetn     =>  aresetn,
    reg0        =>  initFiltReg,
    adc_i       =>  adcFilt_i,
    valid_i     =>  filtValid_i,
    adc_o       =>  adcFilt_o,
    valid_o     =>  filtValid_o
);
--
-- Also instantiate the lock-in detection module
--
LockIn: LockInDetector
port map(
    clk         =>  adcClk,
    aresetn     =>  aresetn,
    regs_i      =>  lockinRegs,
    dac_o       =>  lockin_dac_o,
    data_i      =>  lockin_data_i,
    valid_i     =>  '1',
    data_o      =>  lockin_data_o,
    valid_o     =>  lockin_valid_o
);
--
-- Define the scan module
--
GenScan: TriangularScan
port map(
    clk             =>  adcClk,
    aresetn         =>  aresetn,
    enable_i        =>  scanEnable_i,
    enable_o        =>  scanEnable_o,
    regs_i          =>  scanRegs,
    scan_o          =>  scan_o,
    polarity_o      =>  scanPolarity_o,
    valid_o         =>  scanValid_o
);
scanEnable_i <= scanEnableSet and not(pidEnable1 or pidEnable2);
--
-- Define PID 1 signals
--
measure1_i <=   adcFilt_o(0) when inputSignalSelect = "00" else 
                adcFilt_o(1) when inputSignalSelect = "01" else
                lockin_data_o(0) when inputSignalSelect = "10" else
                lockin_data_o(1) when inputSignalSelect = "11";
                
measValid1_i <= lockin_valid_o(0) when inputSignalSelect = "10" else
                lockin_valid_o(1) when inputSignalSelect = "11" else
                filtValid_o;
                
scanValid1_i <= scanValid_o and pidScanEnable1;
scan1_i <= scan_o when pidScanEnable1 = '1' else (others => '0');
--
-- Define PID 1 module
--
PID_Controller1: PID_Controller
port map(
    clk         =>  adcClk,
    aresetn     =>  aresetn,
    control_i   =>  control1_i,
    measure_i   =>  measure1_i,
    measValid_i =>  measValid1_i,
    scan_i      =>  scan1_i,
    scanValid_i =>  scanValid1_i,
    regs_i      =>  pidRegs1_i,
    pid_o       =>  pid1_o,
    act_o       =>  act1_o,
    valid_o     =>  pidValid1_o
);
--
-- Define PID 2 signals
--
measure2_i <=   adcFilt_o(0) when inputSignalSelect = "00" else 
                adcFilt_o(1) when inputSignalSelect = "01" else
                lockin_data_o(0) when inputSignalSelect = "10" else
                lockin_data_o(1) when inputSignalSelect = "11";
                
measValid2_i <= lockin_valid_o(0) when inputSignalSelect = "10" else
                lockin_valid_o(1) when inputSignalSelect = "11" else
                filtValid_o;
                
scanValid2_i <= scanValid_o and pidScanEnable2;
scan2_i <= scan_o when pidScanEnable2 = '1' else (others => '0');
--
-- Define PID 2 module
--
PID_Controller2: PID_Controller
port map(
    clk         =>  adcClk,
    aresetn     =>  aresetn,
    control_i   =>  control2_i,
    measure_i   =>  measure2_i,
    measValid_i =>  measValid2_i,
    scan_i      =>  scan2_i,
    scanValid_i =>  scanValid2_i,
    regs_i      =>  pidRegs2_i,
    pid_o       =>  pid2_o,
    act_o       =>  act2_o,
    valid_o     =>  pidValid2_o
);
--
-- Routing of signals to memory
--
fifoRoute1 <= convert_fifo_route(fifoReg(3 downto 0));
fifo1 <= adcFilt_o(0)       when fifoRoute1 = adc1 else
         adcFilt_o(1)       when fifoRoute1 = adc2 else
         scan_o             when fifoRoute1 = scan else
         pid1_o             when fifoRoute1 = pid1 else
         pid2_o             when fifoRoute1 = pid2 else
         act1_o             when fifoRoute1 = act1 else
         act2_o             when fifoRoute1 = act2 else
         lockin_data_o(0)   when fifoRoute1 = demod1 else
         lockin_data_o(1)   when fifoRoute1 = demod1 else
         (others => '0');        

fifoRoute2 <= convert_fifo_route(fifoReg(7 downto 4));
fifo2 <= adcFilt_o(0)       when fifoRoute2 = adc1 else
         adcFilt_o(1)       when fifoRoute2 = adc2 else
         scan_o             when fifoRoute2 = scan else
         pid1_o             when fifoRoute2 = pid1 else
         pid2_o             when fifoRoute2 = pid2 else
         act1_o             when fifoRoute2 = act1 else
         act2_o             when fifoRoute2 = act2 else
         lockin_data_o(0)   when fifoRoute2 = demod1 else
         lockin_data_o(1)   when fifoRoute2 = demod1 else
         (others => '0');


fifo_i <= std_logic_vector(fifo2) & std_logic_vector(fifo1);
--
-- Parse FIFO parameters
--
fifoReg_o(0) <= not(fifo_s.empty);
fifoReg_o(1) <= not(scanPolarity_o);
fifoReg_o(31 downto 2) <= (others => '0');
fifoReset <= triggers(0);

FIFO: FIFOHandler
port map(
    wr_clk          =>  adcClk,
    rd_clk          =>  sysClk,
    aresetn         =>  aresetn,
    sampleTime_i    =>  fifoReg(31 downto 8),
    enable_i        =>  scanPolarity_o,
    data_i          =>  fifo_i,
    fifoReset       =>  fifoReset,
    bus_m           =>  fifo_m,
    bus_s           =>  fifo_s
);

--
-- AXI communication routing - connects bus objects to std_logic signals
--
bus_m.addr <= addr_i;
bus_m.valid <= dataValid_i;
bus_m.data <= writeData_i;
readData_o <= bus_s.data;
resp_o <= bus_s.resp;
--
-- Define the AXI parameter parsing process
--
Parse: process(sysClk,aresetn) is
begin
    if aresetn = '0' then
        comState <= idle;
        reset <= '0';
        bus_s <= INIT_AXI_BUS_SLAVE;
        triggers <= (others => '0');
        topReg <= (others => '0');
        initFiltReg <= X"0000_0002";
        pidRegs1 <= (others => (others => '0'));
        pidRegs2 <= (others => (others => '0'));
        scanRegs <= (others => (others => '0'));
        fifo_m <= INIT_FIFO_BUS_MASTER;
        fifoReg <= (others => '0');
        lockinRegs <= (others => (others => '0'));
    elsif rising_edge(sysClk) then
        FSM: case(comState) is
            when idle =>
                triggers <= (others => '0');
                reset <= '0';
                bus_s.resp <= "00";
                if bus_m.valid(0) = '1' then
                    comState <= processing;
                end if;

            when processing =>
                AddrCase: case(bus_m.addr(31 downto 24)) is
                    --
                    -- Parameter parsing
                    --
                    when X"00" =>
                        ParamCase: case(bus_m.addr(23 downto 0)) is
                            --
                            -- This issues a reset signal to the memories and writes data to
                            -- the trigger registers
                            --
                            when X"000000" => 
                                rw(bus_m,bus_s,comState,triggers);
                                reset <= '1';
                            --
                            -- Top-level register
                            --
                            when X"000004" => rw(bus_m,bus_s,comState,topReg);
                            --
                            -- Initial filtering register
                            --
                            when X"000008" => rw(bus_m,bus_s,comState,initFiltReg);
                            --
                            -- PID 1 registers
                            --
                            when X"000010" => rw(bus_m,bus_s,comState,pidRegs1(0));
                            when X"000014" => rw(bus_m,bus_s,comState,pidRegs1(1));
                            when X"000018" => rw(bus_m,bus_s,comState,pidRegs1(2));
                            --
                            -- PID 2 registers
                            --
                            when X"000020" => rw(bus_m,bus_s,comState,pidRegs2(0));
                            when X"000024" => rw(bus_m,bus_s,comState,pidRegs2(1));
                            when X"000028" => rw(bus_m,bus_s,comState,pidRegs2(2));
                            --
                            -- Scan registers
                            --
                            when X"000030" => rw(bus_m,bus_s,comState,scanRegs(0));
                            when X"000034" => rw(bus_m,bus_s,comState,scanRegs(1));
                            when X"000038" => rw(bus_m,bus_s,comState,scanRegs(2));
                            --
                            -- FIFO read/write register
                            --
                            when X"000040" => rw(bus_m,bus_s,comState,fifoReg);
                            --
                            -- Lock in detector settings
                            --
                            when X"000050" => rw(bus_m,bus_s,comState,lockinRegs(0));
                            when X"000054" => rw(bus_m,bus_s,comState,lockinRegs(1));
                            when X"000058" => rw(bus_m,bus_s,comState,lockinRegs(2));
                            when X"00005C" => rw(bus_m,bus_s,comState,lockinRegs(3));
                            --
                            -- If not specified, throw an error
                            --
                            when others => 
                                comState <= finishing;
                                bus_s.resp <= "11";
                        end case;
                    --
                    -- Read-only parameters
                    --
                    when X"01" =>
                        ParamCaseReadOnly: case(bus_m.addr(23 downto 0)) is
                            when X"000000" => rw(bus_m,bus_s,comState,triggers);
                            when X"000004" => readOnly(bus_m,bus_s,comState,fifoReg_o);
                            when X"000008" => fifoRead(bus_m,bus_s,comState,fifo_m,fifo_s);
                            when others => 
                                comState <= finishing;
                                bus_s.resp <= "11";
                        end case;
                    
                    when others => 
                        comState <= finishing;
                        bus_s.resp <= "11";
                end case;
            when finishing =>
--                triggers <= (others => '0');
--                reset <= '0';
                comState <= idle;

            when others => comState <= idle;
        end case;
    end if;
end process;

    
end architecture Behavioural;