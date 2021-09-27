library IEEE;
use ieee.std_logic_1164.all; 
use ieee.numeric_std.ALL;
use ieee.std_logic_unsigned.all; 
use work.CustomDataTypes.all;

--
--Uses measurement and control values to implement a PID controller
--by calculating a correction to the actuator (DAC) value at each
--time step.
--
entity PID_Controller is
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
end PID_Controller;

architecture Behavioral of PID_Controller is

--
-- Define constant widths
--
constant EXP_WIDTH	:	integer	:=	16;							--Expanded width of signals
constant MULT_WIDTH	:	integer	:=	PID_WIDTH + EXP_WIDTH;		--Width of multiplied signals
constant MULT_DELAY	:	integer	:=	5;							--Latency of the multiplication blocks
--
-- Multiplies a 40-bit signed value with a 16-bit unsigned value.
-- The widths of these signals must match the width of their appropriate parameters.
-- a'length = EXP_WIDTH, b'length = K_WIDTH, p'length = MULT_WIDTH
--
COMPONENT K_Multiplier
PORT (
    clk : in std_logic;
    a : IN STD_LOGIC_VECTOR(EXP_WIDTH-1 DOWNTO 0);
    b : IN STD_LOGIC_VECTOR(PID_WIDTH-1 DOWNTO 0);
    p : OUT STD_LOGIC_VECTOR(MULT_WIDTH-1 DOWNTO 0)
);
END COMPONENT;
--
-- Control signals
--
signal enable	:	std_logic;
signal polarity	:	std_logic;
--
-- Internal measurement, control, and scan signals
--
signal measurement, control				:	signed(EXP_WIDTH - 1 downto 0);
signal scan								:	signed(MULT_WIDTH - 1 downto 0);
--
-- Error signals -- need the current value and the last 2
--
signal err0, err1, err2 				:	signed(EXP_WIDTH - 1 downto 0);
--
-- Individual multiplier inputs and ouputs
--
signal Kp, Ki, Kd						:	std_logic_vector(PID_WIDTH - 1 downto 0);
signal divisor							:	integer range 0 to 2**(PID_WIDTH - 1);
signal prop_i, integral_i, derivative_i	:	signed(EXP_WIDTH - 1 downto 0);
signal prop_o, integral_o, derivative_o	:	std_logic_vector(MULT_WIDTH - 1 downto 0);
--
-- Final values
--
signal pidSum, pidNew, pidDivide		:	signed(MULT_WIDTH-1 downto 0);
signal lowerLimit, upperLimit			:	signed(MULT_WIDTH-1 downto 0);
--
-- State and flow control
--
signal multCount	:	unsigned(3 downto 0);
type t_state_local is (acquire_input, multiplying, add_scan, check_limits, outputting);
signal state		:	t_state_local;


begin	
--
-- Parse parameters
--
enable <= regs_i(0)(0);
polarity <= regs_i(0)(1);
--
-- Parse gains
--
Kp <= regs_i(1)(15 downto 0);
Ki <= regs_i(1)(31 downto 16);
Kd <= regs_i(2)(15 downto 0);
divisor <= to_integer(unsigned(regs_i(2)(31 downto 16)));
--
-- Parse limits, and shift left so that we can compare the full-precision values with the limits
--
lowerLimit <= shift_left(resize(signed(regs_i(3)(15 downto 0)),lowerLimit'length),divisor);
upperLimit <= shift_left(resize(signed(regs_i(3)(31 downto 16)),upperLimit'length),divisor);
--
-- Resize inputs to EXP_WIDTH. Scan is resized and shifted left because it will be
-- shifted down by DIVISOR later
--
measurement <= resize(measure_i,measurement'length);
control <= resize(control_i,control'length);
scan <= shift_left(resize(scan_i,scan'length),divisor);		
--
-- Calculate error signal
--
err0 <= control - measurement when polarity = '0' else measurement - control;
error_o <= resize(err0,error_o'length);

--
-- Calculate actuator stages
--
MultProp: K_Multiplier
port map (
	clk => clk,
	a => std_logic_vector(prop_i),
	b => Kp,
	p => prop_o);
	
MultInt: K_Multiplier
port map (
	clk => clk,
	a => std_logic_vector(integral_i),
	b => Ki,
	p => integral_o);	

MultDeriv: K_Multiplier
port map (
	clk => clk,
	a => std_logic_vector(derivative_i),
	b => Kd,
	p => derivative_o);

pidSum <= signed(prop_o) + signed(integral_o) + signed(derivative_o);

--
-- This is the main PID process and provides parsing of the loop registers as well
-- as handling the timing.
--
PID_Process: process(clk,aresetn) is
begin
	if aresetn = '0' then
		multCount <= (others => '0');
		valid_o <= '0';

		prop_i <= (others => '0');
		integral_i <= (others => '0');
		derivative_i <= (others => '0');
		pidNew <= (others => '0');
		pidDivide <= (others => '0');
		pid_o <= (others => '0');
		act_o <= (others => '0');
		err1 <= (others => '0');
		err2 <= (others => '0');

		state <= acquire_input;
	elsif rising_edge(clk) then
		PID_FSM: case state is
			--
			-- Wait-for-measurement state
			--
			when acquire_input =>
				multCount <= (others => '0');
				valid_o <= '0';
				if enable = '1' and measValid_i = '1' then
					--
					-- Calculate the various PID terms from current and previous error signals
					--
					prop_i <= err0 - err1;
					integral_i <= shift_right(err0 + err1,1);
					derivative_i <= err0 - shift_left(err1,1) + err2;
					err2 <= err1;
					err1 <= err0;

					state <= multiplying;	
					
				elsif enable = '0' then
					--
					-- If the PID controller is not enabled, then set all signals to zero
					--
					prop_i <= (others => '0');
					integral_i <= (others => '0');
					derivative_i <= (others => '0');
					pidNew <= (others => '0');
					pidDivide <= (others => '0');
					pid_o <= (others => '0');
					err1 <= (others => '0');
					err2 <= (others => '0');
					--
					-- If a new scan value arrives, proceed to next stage
					--
					if scanValid_i = '1' then
						state <= add_scan;
					end if;
				end if;
				
			--
			-- Insert delay for multipliers
			--
			when multiplying =>
				if multCount < MULT_DELAY then
					multCount <= multCount + 1;
				else
					--
					-- Add the actuator correction pidSum to the old value of the actuator pidDivide.
					-- Note that this addition takes place BEFORE the right-shift by divisor bits.
					-- Also add the scan value (shifted left to be the same size)
					--
					pidNew <= pidNew + pidSum;
					multCount <= (others => '0');
					state <= add_scan;
				end if;

			--
			-- Add scan value
			--
			when add_scan =>
				pidDivide <= pidNew + scan;
				state <= check_limits;
			
			--
			-- Check limits
			--
			when check_limits =>
				state <= outputting;
				if pidDivide < lowerLimit then
					pidDivide <= lowerLimit;
				elsif pidDivide > upperLimit then
					pidDivide <= upperLimit;
				end if;
			
			--
			-- Generate trigger indicating a valid output
			--
			when outputting =>
				pid_o <= resize(shift_right(pidNew,divisor),pid_o'length);
				act_o <= resize(shift_right(pidDivide,divisor),act_o'length);
				valid_o <= '1';
				state <= acquire_input;
					
			when others => null;
		end case;	--end PID_FSM
	end if;
end process;


end Behavioral;

