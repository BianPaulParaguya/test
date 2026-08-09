-- Functional SPI model for Texas Instruments DAC121S101-SEP
--
-- Models the programming behavior described in datasheet section 6.5:
--   * SYNC is active low.
--   * DIN is sampled on each falling edge of SCLK while SYNC is low.
--   * A complete frame is 16 bits, MSB first.
--   * DB15:DB14 are don't-care bits.
--   * DB13:DB12 are PD1:PD0.
--   * DB11:DB0 are the 12-bit DAC code.
--   * The DAC register and power-down mode update on the 16th falling edge.
--   * Raising SYNC before 16 falling edges aborts the frame and leaves the
--     previously programmed DAC state unchanged.
--
-- This is a simulation model.  dac_code_o represents the digital code stored
-- in the DAC register; it intentionally does not attempt to model analog
-- settling, output impedance, reference error, INL/DNL, or noise.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dac121s101_model is
    port (
        sync_n      : in  std_logic;
        sclk        : in  std_logic;
        din         : in  std_logic;

        dac_code_o  : out std_logic_vector(11 downto 0);
        pd_mode_o   : out std_logic_vector(1 downto 0);
        update_o    : out std_logic;
        aborted_o   : out std_logic
    );
end entity dac121s101_model;

architecture behavioral of dac121s101_model is
    signal shift_reg  : std_logic_vector(15 downto 0) := (others => '0');
    signal bit_count  : integer range 0 to 16 := 0;

    signal dac_code   : std_logic_vector(11 downto 0) := (others => '0');
    signal pd_mode    : std_logic_vector(1 downto 0) := "00";
    signal update_int : std_logic := '0';
    signal abort_int  : std_logic := '0';
begin
    dac_code_o <= dac_code;
    pd_mode_o  <= pd_mode;
    update_o   <= update_int;
    aborted_o  <= abort_int;

    process(sync_n, sclk)
        variable shifted_word : std_logic_vector(15 downto 0);
    begin
        -- A falling edge of SYNC starts a new serial write frame.
        if falling_edge(sync_n) then
            shift_reg  <= (others => '0');
            bit_count  <= 0;
            update_int <= '0';
            abort_int  <= '0';

        -- If SYNC rises before 16 clocks, the datasheet defines the frame as
        -- invalid.  Do not modify dac_code or pd_mode.
        elsif rising_edge(sync_n) then
            if bit_count > 0 and bit_count < 16 then
                abort_int <= '1';
            end if;

            shift_reg  <= (others => '0');
            bit_count  <= 0;
            update_int <= '0';

        -- DAC121S101 samples DIN on falling SCLK edges.
        elsif falling_edge(sclk) then
            if sync_n = '0' then
                if bit_count < 16 then
                    shifted_word := shift_reg(14 downto 0) & din;
                    shift_reg <= shifted_word;

                    if bit_count = 15 then
                        -- Sixteenth falling edge: execute the command.
                        pd_mode    <= shifted_word(13 downto 12);
                        dac_code   <= shifted_word(11 downto 0);
                        bit_count  <= 16;
                        update_int <= '1';
                    else
                        bit_count <= bit_count + 1;
                    end if;
                end if;
            end if;
        end if;
    end process;

end architecture behavioral;
