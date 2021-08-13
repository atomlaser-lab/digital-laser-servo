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
signal enable       :   std_logic;
signal regs_i       :   t_param_reg_array(1 downto 0);
signal scan_o       :   t_dac;
signal polarity_o   :   std_logic;
signal valid_o      :   std_logic;

signal amp, offset  :   std_logic_vector(15 downto 0);
signal stepTime,stepSize    :   std_logic_vector(15 downto 0);

begin

uut: TriangularScan
port map(
    clk     =>  clk,
    aresetn =>  aresetn,
    enable  =>  enable,
    regs_i  =>  regs_i,
    scan_o  =>  scan_o,
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

amp <= std_logic_vector(to_unsigned(500,amp'length));
offset <= std_logic_vector(to_unsigned(0,amp'length));
stepSize <= std_logic_vector(to_unsigned(10,amp'length));
stepTime <= std_logic_vector(to_unsigned(5,amp'length));

main_proc: process is
begin
    aresetn <= '0';
    regs_i(0) <= std_logic_vector(to_unsigned(500,amp'length)) & std_logic_vector(to_unsigned(500,amp'length));
    regs_i(1) <= std_logic_vector(to_unsigned(5,amp'length)) &  std_logic_vector(to_unsigned(100,amp'length));
    enable <= '0';
    wait for 100 ns;
    wait until clk'event and clk = '1';
    aresetn <= '1';
    enable <= '1';
    wait;
end process; 


end Behavioral;
