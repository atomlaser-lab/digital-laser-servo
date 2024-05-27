library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;

entity LockDetection is
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
        lock_detect_o       :   std_logic;
        lock_detect_valid_o :   out std_logic
    );
end LockDetection;

architecture Behavioral of LockDetection is

COMPONENT DDS_Fixed_Phase
PORT (
    aclk : IN STD_LOGIC;
    aresetn : IN STD_LOGIC;
    s_axis_phase_tvalid : IN STD_LOGIC;
    s_axis_phase_tdata : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
    m_axis_data_tvalid : OUT STD_LOGIC;
    m_axis_data_tdata : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
);
END COMPONENT;
  
COMPONENT Mixer_Mult
  PORT (
    CLK : IN STD_LOGIC;
    A : IN STD_LOGIC_VECTOR(13 DOWNTO 0);
    B : IN STD_LOGIC_VECTOR(11 DOWNTO 0);
    P : OUT STD_LOGIC_VECTOR(25 DOWNTO 0)
  );
END COMPONENT;

COMPONENT Power_Mult
  PORT (
    CLK : IN STD_LOGIC;
    A : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    B : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
    P : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
  );
END COMPONENT;

COMPONENT LockInFilter
  PORT (
    aclk : IN STD_LOGIC;
    aresetn : IN STD_LOGIC;
    s_axis_config_tdata : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    s_axis_config_tvalid : IN STD_LOGIC;
    s_axis_config_tready : OUT STD_LOGIC;
    s_axis_data_tdata : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
    s_axis_data_tvalid : IN STD_LOGIC;
    s_axis_data_tready : OUT STD_LOGIC;
    m_axis_data_tdata : OUT STD_LOGIC_VECTOR(71 DOWNTO 0);
    m_axis_data_tvalid : OUT STD_LOGIC
  );
END COMPONENT;

constant DDS_PHASE_WIDTH    :   natural :=  32;
constant DDS_OUT_WIDTH      :   natural :=  12;

signal dds_mix_o                        :   std_logic_vector(31 downto 0);
signal dds_cos, dds_sin                 :   std_logic_vector(DDS_OUT_WIDTH - 1 downto 0);
signal data_slv_i                       :   std_logic_vector(13 downto 0);
signal mult_cos_i, mult_sin_o           :   std_logic_vector(data_slv_i'length + DDS_OUT_WIDTH - 1 downto 0);  

signal cicLog2Rate                      :   unsigned(3 downto 0);
signal cicShift                         :   natural;
signal setShift                         :   unsigned(3 downto 0);
signal filter_config_old                :   std_logic_vector(15 downto 0);
signal filter_valid                     :   std_logic;
signal filt_cos_i, filt_sin_i           :   std_logic_vector(31 downto 0);
signal filt_cos2_i, filt_sin2_i         :   std_logic_vector(31 downto 0);
signal filt_cos_o, filt_sin_o           :   std_logic_vector(71 downto 0);
signal filt_cos2_o, filt_sin2_o         :   std_logic_vector(71 downto 0);
signal filt_cos_valid, filt_sin_valid   :   std_logic;
signal filt_cos2_valid, filt_sin2_valid :   std_logic;

signal power_mult_cos_i, power_mult_sin_i   :   std_logic_vector(31 downto 0);
signal power_mult_cos_o, power_mult_sin_o   :   std_logic_vector(31 downto 0);

signal power_2f, power_threshold            :   unsigned(15 downto 0);


begin
--
-- Parse register
--
power_threshold <= shift_left(unsigned(reg_i(7 downto 0)),8);
cicLog2Rate <= unsigned(reg_i(11 downto 8));
setShift <= unsigned(reg_i(15 downto 12));
cicShift <= to_integer(cicLog2Rate) + to_integer(cicLog2Rate) + to_integer(cicLog2Rate);
filter_config <= std_logic_vector(shift_left(to_unsigned(1,filter_config'length),to_integer(cicLog2Rate)));
--
-- Generate 2f signal, sin and cos
--
LockDetectionDDS: DDS_Fixed_Phase
port map(
    aclk                =>  clk,
    aresetn             =>  aresetn,
    s_axis_phase_tvalid =>  '1',
    s_axis_phase_tdata  =>  std_logic_vector(shift_left(unsigned(mod_freq_i),1)),
    m_axis_data_tvalid  =>  open,
    m_axis_data_tdata   =>  dds_mix_o
);
dds_cos <= dds_mix_o(DDS_OUT_WIDTH - 1 downto 0);
dds_sin <= dds_mix_o(DDS_OUT_WIDTH + 16 - 1 downto 16); 
--
-- Perform mixing (multiplication)
--
LockDetectCosMult: Mixer_Mult
port map(
    clk     =>  clk,
    A       =>  data_slv_i,
    B       =>  dds_cos,
    P       =>  mult_cos_o
);

LockDetectSinMult: Mixer_Mult
port map(
    clk     =>  clk,
    A       =>  data_slv_i,
    B       =>  dds_sin,
    P       =>  mult_sin_o
);
--
-- Filter once
--
filt_cos_i <= std_logic_vector(resize(signed(mult_cos_o),filt_cos_i'length));
filt_sin_i <= std_logic_vector(resize(signed(mult_sin_o),filt_cos_i'length));

ChangeProc: process(clk,aresetn) is
    begin
        if aresetn = '0' then
            filter_config_old <= filter_config;
            filter_valid <= '0';
        elsif rising_edge(clk) then
            filter_config_old <= filter_config;
            if filter_config /= filter_config_old then
                filter_valid <= '1';
            else
                filter_valid <= '0';
            end if;
        end if;
    end process; 

CosFilter : LockInFilter
PORT MAP (
    aclk                    => clk,
    aresetn                 => aresetn,
    s_axis_config_tdata     => filter_config,
    s_axis_config_tvalid    => filter_valid,
    s_axis_config_tready    => open,
    s_axis_data_tdata       => filt_cos_i,
    s_axis_data_tvalid      => '1',
    s_axis_data_tready      => open,
    m_axis_data_tdata       => filt_cos_o,
    m_axis_data_tvalid      => filt_cos_valid
);
    
SinFilter : LockInFilter
PORT MAP (
    aclk                    => clk,
    aresetn                 => aresetn,
    s_axis_config_tdata     => filter_config_i,
    s_axis_config_tvalid    => filter_valid_i,
    s_axis_config_tready    => open,
    s_axis_data_tdata       => filt_sin_i,
    s_axis_data_tvalid      => '1',
    s_axis_data_tready      => open,
    m_axis_data_tdata       => filt_sin_o,
    m_axis_data_tvalid      => filt_sin_valid
); 
--
-- Filter a second time
--
filt_cos2_i <= resize(shift_right(signed(filt_cos_o(64 downto 0)),cicShift),filt_cos2_i'length);
filt_sin2_i <= resize(shift_right(signed(filt_sin_o(64 downto 0)),cicShift),filt_sin2_i'length);

CosFilter2 : LockInFilter
PORT MAP (
    aclk                    => clk,
    aresetn                 => aresetn,
    s_axis_config_tdata     => filter_config_i,
    s_axis_config_tvalid    => filt_cos_valid,
    s_axis_config_tready    => open,
    s_axis_data_tdata       => filt_cos2_i,
    s_axis_data_tvalid      => '1',
    s_axis_data_tready      => open,
    m_axis_data_tdata       => filt_cos2_o,
    m_axis_data_tvalid      => filt_cos2_valid
);
    
SinFilter2 : LockInFilter
PORT MAP (
    aclk                    => clk,
    aresetn                 => aresetn,
    s_axis_config_tdata     => filter_config_i,
    s_axis_config_tvalid    => filt_sin_valid,
    s_axis_config_tready    => open,
    s_axis_data_tdata       => filt_sin2_i,
    s_axis_data_tvalid      => '1',
    s_axis_data_tready      => open,
    m_axis_data_tdata       => filt_sin2_o,
    m_axis_data_tvalid      => filt_sin2_valid
); 

filt_cos2_o <= resize(shift_right(signed(filt_cos2_o(64 downto 0)),cicShift),filt_cos2_o'length);
filt_sin2_o <= resize(shift_right(signed(filt_sin2_o(64 downto 0)),cicShift),filt_sin2_o'length);

--
-- Determine signal power at 2f
--
power_mult_cos_i <= std_logic_vector(resize(filt_cos2_o,power_mult_cos_i'length));
power_mult_sin_i <= std_logic_vector(resize(filt_sin2_o,power_mult_cos_i'length));

PowerMultCos: Power_Mult
port map(
    clk     =>  clk,
    A       =>  power_mult_cos_i,
    B       =>  power_mult_cos_i,
    P       =>  power_mult_cos_o
);

PowerMultSin: Power_Mult
port map(
    clk     =>  clk,
    A       =>  power_mult_sin_i,
    B       =>  power_mult_sin_i,
    P       =>  power_mult_sin_o
);

PowerMultDelayProc: process(clk,aresetn) is
begin
    if aresetn = '0' then
        lock_detect_valid_o <= '0';
        power_2f <= (others => '0');
    elsif rising_edge(clk) then
        if filt_cos2_valid = '1' and delayCount = 0 then
            delayCount <= to_unsigned(1,delayCount'length);
            lock_detect_valid_o <= '0';
        elsif delayCount < MULT_LATENCY then
            delayCount <= delayCount + 1;
        elsif delayCount = MULT_LATENCY then
            delayCount <= (others => '0');
            lock_detect_valid_o <= '1';
            power_2f <= resize(unsigned(power_mult_cos_o) + unsigned(power_mult_sin_o),power_2f'length);
        else
            lock_detect_valid_o <= '0';
        end if;
    end if;
end process;

power_2f_o <= power_2f;
lock_detect_o <= '1' when power_2f >= power_threshold else '0';

end Behavioral;