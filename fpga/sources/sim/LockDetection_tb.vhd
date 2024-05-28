library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;

entity LockDetection_tb is
--  Port ( );
end LockDetection_tb;

architecture Behavioral of LockDetection_tb is

component LockDetection is
    port(
        --
        -- Clocking and reset
        --
        clk         :   in  std_logic;
        aresetn     :   in  std_logic;
        --
        -- Control
        --
        mod_freq_i      :   in  std_logic_vector;
        reg_i           :   in  t_param_reg;
        --
        -- Data in
        --
        data_i      :   in  t_adc;
        valid_i     :   in  std_logic;
        --
        -- Lock detection
        --
        power_2f_o          :   out unsigned(15 downto 0);
        lock_detect_o       :   out std_logic;
        lock_detect_valid_o :   out std_logic
    );
end component;

COMPONENT DDS_Stream_Phase
  PORT (
    aclk : IN STD_LOGIC;
    aresetn : IN STD_LOGIC;
    s_axis_phase_tvalid : IN STD_LOGIC;
    s_axis_phase_tdata : IN STD_LOGIC_VECTOR(63 DOWNTO 0);
    m_axis_data_tvalid : OUT STD_LOGIC;
    m_axis_data_tdata : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
  );
END COMPONENT;

constant DDS_OUT_WIDTH      :   natural :=  12;
signal dds_mix_o                        :   std_logic_vector(31 downto 0);
signal dds_cos, dds_sin                 :   std_logic_vector(DDS_OUT_WIDTH - 1 downto 0);


signal clk_period   :   time    :=  10 ns;
signal clk          :   std_logic;
signal aresetn      :   std_logic;

signal data_i   :   t_adc;
signal valid_i  :   std_logic;
signal mod_freq :   std_logic_vector(31 downto 0);
signal reg_i    :   t_param_reg;

signal power_2f :   unsigned(15 downto 0);
signal lock_detect_o, lock_detect_valid_o   :   std_logic;

signal dds_phase_i                      :   std_logic_vector(63 downto 0);

begin

uut: LockDetection
port map(
    clk     =>  clk,
    aresetn =>  aresetn,
    mod_freq_i  =>  mod_freq,
    reg_i   =>  reg_i,
    data_i  =>  data_i,
    valid_i =>  valid_i,
    power_2f_o  =>  power_2f,
    lock_detect_o   =>  lock_detect_o,
    lock_detect_valid_o =>  lock_detect_valid_o
);

clk_proc: process is
begin
    clk <= '0';
    wait for clk_period/2;
    clk <= '1';
    wait for clk_period/2;
end process;

dds_phase_i <= X"00000000" & std_logic_vector(shift_right(unsigned(mod_freq),1));
LockDetectDDS: DDS_Stream_Phase
port map(
    aclk                =>  clk,
    aresetn             =>  aresetn,
    s_axis_phase_tvalid =>  '1',
    s_axis_phase_tdata  =>  dds_phase_i,
    m_axis_data_tvalid  =>  open,
    m_axis_data_tdata   =>  dds_mix_o
);

dds_cos <= dds_mix_o(DDS_OUT_WIDTH - 1 downto 0);
dds_sin <= dds_mix_o(DDS_OUT_WIDTH + 16 - 1 downto 16); 

data_i <= resize(signed(dds_cos),data_i'length);

main_proc: process is
begin
    aresetn <= '0';
    mod_freq <= X"003126e9";
    reg_i <= X"00000964";
    valid_i <= '1';
    wait for 100 ns;
    wait until clk'event and clk = '1';
    aresetn <= '1';
    wait until clk'event and clk = '1';
    reg_i <= X"00000864";
    wait;
end process; 


end Behavioral;
