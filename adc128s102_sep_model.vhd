-- Behavioral simulation model for the Texas Instruments ADC128S102-SEP.
--
-- Functional scope:
--   * Active-low CS framing.
--   * 16 SCLK rising edges per conversion (integer multiples allowed per frame).
--   * DIN control byte captured MSB-first on the first 8 rising edges of each
--     16-clock conversion segment.
--   * ADD2:ADD0 are DIN bits 5:3 and select the channel for the NEXT conversion.
--   * DOUT is high-impedance while CS is high.
--   * DOUT presents four leading zero bits followed by a 12-bit straight-binary
--     conversion result, MSB first.  Output data advances on SCLK falling edges.
--   * The first conversion after power-up is intentionally from a deterministic
--     generic-selected channel to represent the datasheet's unspecified/random
--     initial channel.  Testbenches should perform a dummy conversion.
--
-- This is a digital functional model.  It does not model acquisition settling,
-- INL/DNL, aperture effects, input impedance, reference error, propagation
-- delays, metastability, or radiation effects.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity adc128s102_sep_model is
    generic (
        -- Real silicon powers up with an unspecified/random selected channel.
        -- A deterministic value keeps regressions repeatable while preserving
        -- the requirement for a dummy conversion.
        POWERUP_CHANNEL : integer range 0 to 7 := 7
    );
    port (
        cs_n : in  std_logic;
        sclk : in  std_logic;
        din  : in  std_logic;
        dout : out std_logic;

        -- Idealized 12-bit conversion codes for the eight analog inputs.
        in0_code : in std_logic_vector(11 downto 0);
        in1_code : in std_logic_vector(11 downto 0);
        in2_code : in std_logic_vector(11 downto 0);
        in3_code : in std_logic_vector(11 downto 0);
        in4_code : in std_logic_vector(11 downto 0);
        in5_code : in std_logic_vector(11 downto 0);
        in6_code : in std_logic_vector(11 downto 0);
        in7_code : in std_logic_vector(11 downto 0);

        -- Simulation observability/debug outputs.
        selected_channel_o : out std_logic_vector(2 downto 0);
        last_control_o     : out std_logic_vector(7 downto 0);
        conversion_done_o  : out std_logic;
        frame_error_o      : out std_logic;
        first_conversion_o : out std_logic
    );
end entity adc128s102_sep_model;

architecture behavioral of adc128s102_sep_model is
    signal selected_channel : integer range 0 to 7 := POWERUP_CHANNEL;
    signal rises_in_segment : integer range 0 to 16 := 0;
    signal rises_in_frame   : integer := 0;
    signal control_shift    : std_logic_vector(7 downto 0) := (others => '0');
    signal last_control     : std_logic_vector(7 downto 0) := (others => '0');
    signal output_word      : std_logic_vector(15 downto 0) := (others => '0');
    signal dout_bit         : std_logic := '0';
    signal conversion_done  : std_logic := '0';
    signal frame_error      : std_logic := '0';
    signal first_conversion : std_logic := '1';

begin
    -- DOUT is explicitly tri-stated whenever CS is inactive.
    dout <= 'Z' when cs_n = '1' else dout_bit;

    selected_channel_o <= std_logic_vector(to_unsigned(selected_channel, 3));
    last_control_o     <= last_control;
    conversion_done_o  <= conversion_done;
    frame_error_o      <= frame_error;
    first_conversion_o <= first_conversion;

    serial_model : process(cs_n, sclk)
        variable next_channel : integer range 0 to 7;
        variable next_word    : std_logic_vector(15 downto 0);
    begin
        -- Default pulse behavior.  A pulse remains asserted until the next
        -- relevant CS/SCLK event, which is sufficient for simulation monitors.
        conversion_done <= '0';

        if falling_edge(cs_n) then
            rises_in_frame   <= 0;
            rises_in_segment <= 0;
            control_shift    <= (others => '0');
            frame_error      <= '0';

            -- The currently selected channel is the result returned during this
            -- conversion.  The control byte captured during this segment selects
            -- the channel used by the subsequent conversion.
            case selected_channel is
                when 0 => next_word := "0000" & in0_code;
                when 1 => next_word := "0000" & in1_code;
                when 2 => next_word := "0000" & in2_code;
                when 3 => next_word := "0000" & in3_code;
                when 4 => next_word := "0000" & in4_code;
                when 5 => next_word := "0000" & in5_code;
                when 6 => next_word := "0000" & in6_code;
                when 7 => next_word := "0000" & in7_code;
            end case;

            output_word <= next_word;
            -- Present bit 15 before the first rising SCLK edge.  The first four
            -- bits are zeros, followed by D11..D0.
            dout_bit <= next_word(15);

        elsif rising_edge(sclk) then
            if cs_n = '0' then
                rises_in_frame <= rises_in_frame + 1;

                -- The first eight rising edges of every 16-clock conversion
                -- capture the control byte MSB-first.
                if rises_in_segment < 8 then
                    control_shift <= control_shift(6 downto 0) & din;
                end if;

                if rises_in_segment < 16 then
                    rises_in_segment <= rises_in_segment + 1;
                end if;
            end if;

        elsif falling_edge(sclk) then
            if cs_n = '0' then
                if rises_in_segment = 16 then
                    -- A complete conversion segment has finished.  Commit its
                    -- control byte now so the selected channel applies to the
                    -- NEXT conversion segment.
                    next_channel := to_integer(unsigned(control_shift(5 downto 3)));
                    selected_channel <= next_channel;
                    last_control <= control_shift;
                    conversion_done <= '1';
                    first_conversion <= '0';

                    -- In continuous-conversion mode CS may remain low.  Prepare
                    -- the next 16-bit output word immediately for the next
                    -- conversion segment.
                    case next_channel is
                        when 0 => next_word := "0000" & in0_code;
                        when 1 => next_word := "0000" & in1_code;
                        when 2 => next_word := "0000" & in2_code;
                        when 3 => next_word := "0000" & in3_code;
                        when 4 => next_word := "0000" & in4_code;
                        when 5 => next_word := "0000" & in5_code;
                        when 6 => next_word := "0000" & in6_code;
                        when 7 => next_word := "0000" & in7_code;
                    end case;

                    output_word <= next_word;
                    dout_bit <= next_word(15);
                    rises_in_segment <= 0;
                    control_shift <= (others => '0');

                elsif rises_in_segment > 0 then
                    -- After rising edge N has sampled the current DOUT bit, the
                    -- following falling edge advances DOUT to the next bit.
                    -- N=1 -> bit14, ... N=15 -> bit0.
                    dout_bit <= output_word(15 - rises_in_segment);
                end if;
            end if;

        elsif rising_edge(cs_n) then
            -- Every CS-low frame must contain an integer multiple of 16 rising
            -- SCLK edges.  Complete 16-clock segments are committed above;
            -- therefore an incomplete trailing segment never changes channel.
            if rises_in_frame mod 16 /= 0 then
                frame_error <= '1';
            end if;
        end if;
    end process;

end architecture behavioral;
