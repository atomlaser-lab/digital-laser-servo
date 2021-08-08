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
        adcData_i       :   in  std_logic_vector(31 downto 0)
    );
end topmod;


architecture Behavioural of topmod is

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
		scan_i		:	in	t_dac;
		scanValid_i	:	in	std_logic;
		--
		-- Parameter inputs:
		-- 0: (0 => enable, 1 => polarity)
		-- 1: (31 downto 16 => Ki, 15 downto 0 => Kp)
		-- 2: (31 downto 16 => divisor, 15 downto 0 => Kd)
		-- 3: (31 downto 16 => upper limit, 15 downto 0 => lower limit)
		regs_i		:	in	t_param_reg_array(3 downto 0);				--Parameter inputs
		--
		-- Outputs
		--
		error_o		:	out	t_adc;										--Output error signal (debugging)
		pid_o		:	out t_dac;										--Actuator output from PID (debugging)
		act_o		:	out t_dac;										--Output actuator signal
		valid_o		:	out std_logic									--Indicates act_o is valid
	);								
end component;

component TriangularScan is
    port(
        clk         :   in  std_logic;          --Input clock
        aresetn     :   in  std_logic;          --Asynchronous reset
        enable      :   in  std_logic;

        --
        -- Parameter inputs:
        -- 0: (31 downto 16 => scan amplitude, 15 downto 0 => offset)
        -- 1: (31 downto 16 => step time, 15 downto 0 => step size)
        --
        regs_i      :   in  t_param_reg_array(1 downto 0);
        
        scan_o      :   out t_dac;              --Output scan data
        scanDir_o   :   out std_logic;          --Indicates the scan direction ('0' = negative, '1' = positive)
        valid_o     :   out std_logic           --Indicates valid averaged data
    );
end component;

component SaveADCData is
    generic(
        MEM_SIZE    :   natural                 --Options are 14, 13, and 12
    );
    port(
        readClk     :   in  std_logic;          --Clock for reading data
        writeClk    :   in  std_logic;          --Clock for writing data
        aresetn     :   in  std_logic;          --Asynchronous reset
        
        data_i      :   in  std_logic_vector;   --Input data, maximum length of 32 bits
        valid_i     :   in  std_logic;          --High for one clock cycle when data_i is valid
        
        bus_m       :   in  t_mem_bus_master;   --Master memory bus
        bus_s       :   out t_mem_bus_slave     --Slave memory bus
    );
end component;

--
-- AXI communication signals
--
signal comState             :   t_status                        :=  idle;
signal bus_m                :   t_axi_bus_master                :=  INIT_AXI_BUS_MASTER;
signal bus_s                :   t_axi_bus_slave                 :=  INIT_AXI_BUS_SLAVE;
--
-- Shared registers and signals
--
signal triggers             :   t_param_reg                     :=  (others => '0');
signal topReg               :   t_param_reg;
--
-- Initial filter signals
--
signal initFilterReg                :   t_param_reg             :=  (others => '0');
signal adcFilt_i                    :   t_adc_array;
signal adcFilt_o                    :   t_adc_array;
signal filtValid_i, filtValid_o     :   std_logic;
--
-- PID 1 settings and signals
--
signal pidRegs1                     :   t_param_reg_array(3 downto 0);
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
signal pidRegs2                     :   t_param_reg_array(3 downto 0);
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
signal scanRegs                     :   t_param_reg_array(1 downto 0);
signal scan_o                       :   t_dac;
signal scanValid_o                  :   std_logic;
signal scanEnable_i                 :   std_logic;
signal scanEnableSet                :   std_logic;
signal scanDir_o                    :   std_logic;
--
-- Memory signals and settings
--
type t_fifo_route is (adc1, adc2, scan, pid1, pid2, act1, act2, no_output);
signal fifoReg                      :   t_param_reg;
signal fifoRoute                    :   t_fifo_route;
signal fifo1, fifo2                 :   signed(15 downto 0);
signal fifo_i, fifo_o               :   std_logic_vector(31 downto 0);
signal fifoReset, fifoEnable        :   std_logic;

procedure convert_fifo_route(
    signal val_i    :   in  std_logic_vector(7 downto 0);
    signal route_o  :   out t_fifo_route) is
begin
    if val_i = X"00" then
        route_o <= adc1;
    elsif val_i = X"01" then
        route_o <= adc2;
    elsif val_i = X"02" then
        route_o <= scan;
    elsif val_i = X"03" then
        route_o <= pid1;
    elsif val_i = X"04" then
        route_o <= pid2;
    elsif val_i = X"06" then
        route_o <= act1;
    elsif val_i = X"07" then
        route_o <= act2;
    else
        route_o <= no_output;
    end if;
end convert_fifo_route;

begin
--
-- Start by parsing parameters
--
pidRegs1(0) <= std_logic_vector(resize(unsigned(topReg(7 downto 0)),PARAM_WIDTH));
pidRegs2(0) <= std_logic_vector(resize(unsigned(topReg(15 downto 8)),PARAM_WIDTH));
scanEnableSet <= topReg(16);

pidEnable1 <= pidRegs1(0)(0);
pidScanEnable1 <= pidRegs1(0)(2);
pidEnable2 <= pidRegs2(0)(0);
pidScanEnable2 <= pidRegs2(0)(2);


--
-- Begin with a quick-average module for initial filtering
--
adcFilt_i(0) <= signed(adcData_i(15 downto 0));
adcFilt_i(1) <= signed(adcData_i(31 downto 0));
filtValid_i <= '1';
InitFilt: QuickAvg
port map(
    clk         =>  adcClk,
    aresetn     =>  aresetn,
    reg0        =>  initFilterReg,
    adc_i       =>  adcFilt_i,
    valid_i     =>  filtValid_i;
    adc_o       =>  adcFilt_o,
    valid_o     =>  filtValid_o
);
--
-- Define the scan module
--
GenScan: TriangularScan
port map(
    clk         =>  adcClk,
    aresetn     =>  aresetn,
    enable      =>  scanEnable_i,
    regs_i      =>  scanRegs,
    scan_o      =>  scan_o,
    scanDir_o   =>  scanDir_o,
    valid_o     =>  scanValid_o
);
scanEnable_i <= scanEnableSet and not(pidEnable1 or pidEnable2);
--
-- Define PID 1 signals
--
measure1_i <= adcFilt_o(0);
measValid1_i <= filtValid_o;
scanValid1_i <= scanValid_o and pidScanEnable1;
scan1_i <= scan_o when pidScanEnable1 = '1' else (others => '0');
--
-- Define PID 1 module
--
PID1: PID_Controller
port map(
    clk         =>  adcClk,
    aresetn     =>  aresetn,
    control_i   =>  control1_i,
    measure_i   =>  measure1_i,
    measValid_i =>  measValid1_i,
    scan_i      =>  scan1_i,
    scanValid_i =>  scanValid1_i,
    regs_i      =>  pidRegs1,
    error_o     =>  error1_o,
    pid_o       =>  pid1_o,
    act_o       =>  act1_o,
    valid_o     =>  pidValid1_o
);
--
-- Define PID 2 signals
--
measure2_i <= adcFilt_o(0);
measValid2_i <= filtValid_o;
scanValid2_i <= scanValid_o and pidScanEnable2;
scan2_i <= scan_o when pidScanEnable2 = '1' else (others => '0');
--
-- Define PID 2 module
--
PID2: PID_Controller
port map(
    clk         =>  adcClk,
    aresetn     =>  aresetn,
    control_i   =>  control2_i,
    measure_i   =>  measure2_i,
    measValid_i =>  measValid2_i,
    scan_i      =>  scan2_i,
    scanValid_i =>  scanValid2_i,
    regs_i      =>  pidRegs2,
    error_o     =>  error2_o,
    pid_o       =>  pid2_o,
    act_o       =>  act2_o,
    valid_o     =>  pidValid2_o
);
--
-- Routing of signals to memory
--
fifoRoute <= convert_fifo_route(fifoReg(7 downto 0));
fifo1 <= adcFilt_o(0) when fifoRoute = adc1 else
         adcFilt_o(1) when fifoRoute = adc2 else
         scan_o       when fifoRoute = scan else
         pid1_o       when fifoRoute = pid1 else
         pid2_o       when fifoRoute = pid2 else
         act1_o       when fifoRoute = act1 else
         act2_o       when fifoRoute = act2 else
         (others => '0');
fifo2 <= adcFilt_o(0) when fifoRoute = adc1 else
         adcFilt_o(1) when fifoRoute = adc2 else
         scan_o       when fifoRoute = scan else
         pid1_o       when fifoRoute = pid1 else
         pid2_o       when fifoRoute = pid2 else
         act1_o       when fifoRoute = act1 else
         act2_o       when fifoRoute = act2 else
         (others => '0');

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
        initFiltReg <= (others => '0');
        pidRegs1(pidRegs1'length - 1 downto 1) <= (others => (others => '0'));
        pidRegs2(pidRegs2'length - 1 downto 1) <= (others => (others => '0'));
        scanRegs <= (others => (others => '0'));
        fifoReg <= (others => '0');
        
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
                            when X"00000C" => rw(bus_m,bus_s,comState,pidRegs1(0));
                            when X"000010" => rw(bus_m,bus_s,comState,pidRegs1(1));
                            when X"000014" => rw(bus_m,bus_s,comState,pidRegs1(2));
                            when X"000018" => rw(bus_m,bus_s,comState,pidRegs1(3));
                            --
                            -- PID 2 registers
                            --
                            when X"00001C" => rw(bus_m,bus_s,comState,pidRegs2(0));
                            when X"000020" => rw(bus_m,bus_s,comState,pidRegs2(1));
                            when X"000024" => rw(bus_m,bus_s,comState,pidRegs2(2));
                            when X"000028" => rw(bus_m,bus_s,comState,pidRegs2(3));
                            --
                            -- Scan registers
                            --
                            when X"00002C" => rw(bus_m,bus_s,comState,scanRegs(0));
                            when X"000030" => rw(bus_m,bus_s,comState,scanRegs(1));
                            --
                            -- FIFO register
                            --
                            when X"000034" => rw(bus_m,bus_s,comState,fifoReg);
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
                            when X"000000" => readOnly(bus_m,bus_s,comState,mem_bus_s(0).last);
                            when X"000004" => readOnly(bus_m,bus_s,comState,mem_bus_s(1).last);
                            when X"000008" => readOnly(bus_m,bus_s,comState,mem_bus_s(2).last);
                            when X"00000C" => readOnly(bus_m,bus_s,comState,mem_bus_s(3).last);
                            when X"000010" => readOnly(bus_m,bus_s,comState,mem_bus_s(4).last);
                            when others => 
                                comState <= finishing;
                                bus_s.resp <= "11";
                        end case;
                    --
                    -- Read data
                    -- X"02" => Raw data for signal acquisition
                    -- X"03" => Integrated data for signal acquisition
                    -- X"04" => Raw data for auxiliary acquisition
                    -- X"05" => Integrated data for signal acquisition
                    -- 
                    when X"02" | X"03" | X"04" | X"05" | X"06" =>
                        if bus_m.valid(1) = '0' then
                            bus_s.resp <= "11";
                            comState <= finishing;
                            mem_bus_m(memIdx).trig <= '0';
                            mem_bus_m(memIdx).status <= idle;
                        elsif mem_bus_s(memIdx).valid = '1' then
                            bus_s.data <= mem_bus_s(memIdx).data;
                            comState <= finishing;
                            bus_s.resp <= "01";
                            mem_bus_m(memIdx).status <= idle;
                            mem_bus_m(memIdx).trig <= '0';
                        elsif mem_bus_m(memIdx).status = idle then
                            mem_bus_m(memIdx).addr <= bus_m.addr(MEM_ADDR_WIDTH+1 downto 2);
                            mem_bus_m(memIdx).status <= waiting;
                            mem_bus_m(memIdx).trig <= '1';
                         else
                            mem_bus_m(memIdx).trig <= '0';
                        end if;
                    
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