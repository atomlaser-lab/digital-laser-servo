library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;

entity TriangularScan_tb is
--  Port ( );
end TriangularScan_tb;

architecture Behavioral of TriangularScan_tb is

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

signal clk_period   :   time    :=  10 ns;
signal clk          :   std_logic;
signal aresetn      :   std_logic;
signal enable_i, enable_o       :   std_logic;
signal regs_i       :   t_param_reg_array(2 downto 0);
signal scan_o       :   t_dac;
signal polarity_o   :   std_logic;
signal valid_o      :   std_logic;

begin

uut: TriangularScan
port map(
    clk         =>  clk,
    aresetn     =>  aresetn,
    enable_i    =>  enable_i,
    enable_o    =>  enable_o,
    regs_i      =>  regs_i,
    scan_o      =>  scan_o,
    polarity_o  =>  polarity_o,
    valid_o     =>  valid_o
);

clk_proc: process is
begin
    clk <= '0';
    wait for clk_period/2;
    clk <= '1';
    wait for clk_period/2;
end process;

main_proc: process is
begin
    aresetn <= '0';
    regs_i(0) <= std_logic_vector(to_unsigned(500,16)) & std_logic_vector(to_unsigned(250,16));
    regs_i(1) <= std_logic_vector(to_unsigned(10,32));
    regs_i(2) <= std_logic_vector(to_unsigned(5,32));
    enable_i <= '0';
    wait for 100 ns;
    wait until clk'event and clk = '1';
    aresetn <= '1';
    enable_i <= '1';
    wait for 9.8 us;
    wait until rising_edge(clk);
    enable_i <= '0';
    wait;
end process; 


end Behavioral;
