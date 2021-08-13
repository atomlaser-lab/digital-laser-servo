library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;

entity QuickAvg_tb is
--  Port ( );
end QuickAvg_tb;

architecture Behavioral of QuickAvg_tb is

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

signal clk_period   :   time    :=  10 ns;
signal clk          :   std_logic;
signal aresetn      :   std_logic;

signal reg0     :   t_param_reg;
signal adc_i,adc_o    :   t_adc_array;
signal valid_i, valid_o :   std_logic;

begin

uut: QuickAvg
port map(
    clk     =>  clk,
    aresetn =>  aresetn,
    reg0    =>  reg0,
    adc_i   =>  adc_i,
    valid_i =>  valid_i,
    adc_o   =>  adc_o,
    valid_o =>  valid_o
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
    reg0(3 downto 0) <= X"4";
    reg0(31 downto 4) <= (others => '0');
    adc_i(0) <= to_signed(100,ADC_WIDTH);
    adc_i(1) <= to_signed(-100,ADC_WIDTH);
    valid_i <= '1';
    wait for 100 ns;
    wait until clk'event and clk = '1';
    aresetn <= '1';
    wait;
end process; 


end Behavioral;
