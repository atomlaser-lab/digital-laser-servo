library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;

entity PID_Controller_tb is
--  Port ( );
end PID_Controller_tb;

architecture Behavioral of PID_Controller_tb is

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
        polarity_o  :   out std_logic;          --Indicates the scan direction ('0' = negative, '1' = positive)
        valid_o     :   out std_logic           --Indicates valid averaged data
    );
end component;

signal clk_period   :   time    :=  10 ns;
signal clk          :   std_logic;
signal aresetn      :   std_logic;

signal control_i, measure_i :   t_adc;
signal measValid_i  :   std_logic;


signal regs_i   :   t_param_reg_array(3 downto 0);

signal error_o      :   t_adc;
signal pid_o,act_o  :   t_dac;
signal valid_o      :   std_logic;

signal count, delay :   unsigned(15 downto 0);

--
-- Scan
--
signal scanRegs     :   t_param_reg_array(1 downto 0);
signal scan_o       :   t_dac;
signal polarity_o   :   std_logic;
signal scanValid    :   std_logic;
signal scanEnable   :   std_logic;

begin

uut: PID_Controller
port map(
    clk     =>  clk,
    aresetn =>  aresetn,
    control_i   =>  control_i,
    measure_i   =>  measure_i,
    measValid_i =>  measValid_i,
    scan_i      =>  scan_o,
    scanValid_i =>  scanValid,
    regs_i      =>  regs_i,
    error_o     =>  error_o,
    pid_o       =>  pid_o,
    act_o       =>  act_o,
    valid_o     =>  valid_o
);

ScanUUT: TriangularScan
port map(
    clk     =>  clk,
    aresetn =>  aresetn,
    enable  =>  scanEnable,
    regs_i  =>  scanRegs,
    scan_o  =>  scan_o,
    polarity_o  =>  polarity_o,
    valid_o     =>  scanValid
);

clk_proc: process is
begin
    clk <= '0';
    wait for clk_period/2;
    clk <= '1';
    wait for clk_period/2;
end process;

MeasGen: process(clk,aresetn) is
begin
    if aresetn = '0' then
        measure_i <= (others => '0');
        measValid_i <= '0';
        delay <= to_unsigned(10,delay'length);
        count <= (others => '0');
    elsif rising_edge(clk) then
        if count < (delay - 1) then
            count <= count + 1;
            measValid_i <= '0';
        else
            measure_i <= act_o;
            measValid_i <= '1';
            count <= (others => '0');
        end if;
    end if;
end process;


scanEnable <= not(regs_i(0)(0));
main_proc: process is
begin
    aresetn <= '0';
    control_i <= to_signed(500,control_i'length);
    regs_i(0) <= (0 => '0',1 => '0',others => '0');
    regs_i(1) <= X"0001" & X"0001";
    regs_i(2) <= X"0002" & X"0001";
    regs_i(3) <= std_logic_vector(to_signed(8000,16)) & std_logic_vector(to_signed(-8000,16));

    scanRegs(0) <= std_logic_vector(to_unsigned(500,16)) & std_logic_vector(to_unsigned(500,16));
    scanRegs(1) <= std_logic_vector(to_unsigned(5,16)) &  std_logic_vector(to_unsigned(100,16));
    
    wait for 100 ns;
    wait until clk'event and clk = '1';
    aresetn <= '1';
    wait for 4 us;
    regs_i(0)(0) <= '1';
    wait;
end process; 


end Behavioral;
