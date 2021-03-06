library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;
--
-- This module creates a symmetrical triangular scan
-- output
--
entity TriangularScan is
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
end TriangularScan;

architecture Behavioural of TriangularScan is

type t_dac_array_local is array(natural range <>) of t_dac;

signal polarity                 :   std_logic;
signal offset                   :   t_dac;
signal amplitude                :   t_dac;
signal lowerLimit, upperLimit   :   t_dac;
signal stepSize                 :   t_dac;
signal count, stepTime          :   unsigned(31 downto 0);
signal scan                     :   t_dac_array_local(1 downto 0);
signal valid, enable            :   std_logic;

begin
--
-- Parse parameters
--
offset <= signed(regs_i(0)(15 downto 0));
amplitude <= signed(regs_i(0)(31 downto 16));
stepSize <= resize(signed(regs_i(1)(15 downto 0)),stepSize'length);
stepTime <= resize(unsigned(regs_i(2)),stepTime'length);
lowerLimit <= offset - amplitude;
upperLimit <= offset + amplitude;
--
-- Define process for reduced clock signal
--
ReducedClock: process(clk,aresetn) is
begin
    if aresetn = '0' then
        count <= (others => '0');
        valid <= '0';
    elsif rising_edge(clk) then
        if count < stepTime then
            count <= count + 1;
            valid <= '0';
        else
            count <= (0 => '1', others => '0');
            valid <= '1';
        end if;
    end if;
end process;
--
-- Define process, and assign scan_o to scan
--
scan_o <= scan(0) when enable = '1' else offset;
polarity_o <= polarity;
enable_o <= enable;
ScanProc: process(clk,aresetn) is
begin
    if aresetn = '0' then
        scan <= (others => offset);
        polarity <= '1';
        valid_o <= '0';
    elsif rising_edge(clk) then
        if valid = '1' then
            --
            -- When there is a valid reduced clock signal, output a new value
            --
            valid_o <= '1';
            scan(1) <= scan(0);
            if scan(0) < lowerLimit then
                --
                -- When we go below the lower limit, switch the polarity to positive
                --
                polarity <= '1';
                scan(0) <= lowerLimit;
            elsif scan(0) > upperLimit then
                --
                -- When we go above the upper limit, switch the polarity to negative
                --
                polarity <= '0';
                scan(0) <= upperLimit;
            elsif polarity = '1' then
                --
                -- When the polarity is positive, add the step size
                --
                scan(0) <= scan(0) + stepSize;
            elsif polarity = '0' then
                --
                -- When the polarity is negative, subtract the step size
                --
                scan(0) <= scan(0) - stepSize;
            end if;
        else
            valid_o <= '0';
        end if;
    end if;
end process;


P1: process(clk,aresetn) is
begin
    if aresetn = '0' then
        enable <= '0';
    elsif rising_edge(clk) then
        if enable_i = '1' then
            enable <= '1';
        else
            if (scan(0) >= offset and scan(1) < offset) or (scan(0) < offset and scan(1) >= offset) then
                enable <= '0';
            end if;
        end if;
    end if;
end process;


end architecture Behavioural;