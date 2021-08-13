library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;
use work.AXI_Bus_Package.all;

entity FIFOHandler is
    generic(
        ENABLE_SKIP :   boolean :=  false
    );
    port(
        wr_clk      :   in  std_logic;
        rd_clk      :   in  std_logic;
        aresetn     :   in  std_logic;
        
        data_i      :   in  std_logic_vector(FIFO_WIDTH-1 downto 0);
        valid_i     :   in  std_logic;

        writeSkip   :   in  unsigned(15 downto 0);
        
        fifoReset   :   in  std_logic;
        bus_m       :   in  t_fifo_bus_master;
        bus_s       :   out t_fifo_bus_slave
    );
end FIFOHandler;

architecture Behavioral of FIFOHandler is

COMPONENT FIFO_Continuous
  PORT (
    wr_clk : IN STD_LOGIC;
    rd_clk : IN STD_LOGIC;
    rst : IN STD_LOGIC;
    din : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
    wr_en : IN STD_LOGIC;
    rd_en : IN STD_LOGIC;
    dout : OUT STD_LOGIC_VECTOR(31 DOWNTO 0);
    full : OUT STD_LOGIC;
    empty : OUT STD_LOGIC
  );
END COMPONENT;

signal rst      :   std_logic;
signal count    :   unsigned(writeSkip'length - 1 downto 0);
signal valid    :   std_logic;
signal data     :   std_logic_vector(data_i'length - 1 downto 0);

begin
--
-- Generate reset signal
--
rst <= not(aresetn) or fifoReset;
--
-- Creates a valid output signal when rd_en is high
--
ValidDelay: process(rd_clk,aresetn) is
begin
    if aresetn = '0' then
        bus_s.valid <= '0';
    elsif rising_edge(rd_clk) then
        if bus_m.rd_en = '1' then
            bus_s.valid <= '1';
        else
            bus_s.valid <= '0';
        end if;
    end if;    
end process;
--
-- Count out number of inputs to reduce the data that is saved
--
SkipGen: if ENABLE_SKIP generate
    SkipProc: process(wr_clk,aresetn) is
    begin
        if aresetn = '0' then
            count <= (others => '0');
            valid <= '0';
            data <= (others => '0');
        elsif rising_edge(wr_clk) then
            if writeSkip = 0 then
                --
                -- If no skip value is set, pass data and valid signal
                -- straight through
                --
                data <= data_i;
                valid <= valid_i;
            else
                if count = 0 and valid_i = '1' then
                    --
                    -- When there is a skip value and a valid value arrives
                    -- write that value and start a counter
                    --
                    data <= data_i;
                    valid <= '1';
                    count <= (0 => '1', others => '0');
                elsif count < writeSkip then
                    --
                    -- Count upwards to the skip value
                    --
                    count <= count + 1;
                    valid <= '0';
                else
                    --
                    -- Reset the counter
                    --
                    count <= (others => '0');
                    valid <= '0';
                end if;
            end if;
        end if;
    end process;
end generate SkipGen;
--
-- Instantiate FIFO part
--
FIFO: FIFO_Continuous
port map(
    wr_clk      =>  wr_clk,
    rd_clk      =>  rd_clk,
    rst         =>  rst,
    din         =>  data,
    wr_en       =>  valid,
    rd_en       =>  bus_m.rd_en,
    dout        =>  bus_s.data,
    full        =>  bus_s.full,
    empty       =>  bus_s.empty
);


end Behavioral;
