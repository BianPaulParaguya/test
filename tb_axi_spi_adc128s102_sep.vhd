-- Integrated self-checking testbench for:
--   AXI4-Lite -> axi_spi_simple -> spi_peripheral -> ADC128S102-SEP model
--
-- The ADC uses 16 clocks per conversion.  The SPI controller is byte-oriented,
-- so each ADC conversion is generated with two queued 8-bit TX writes while
-- SSN/CS remains continuously low.  The updated spi_peripheral contains a
-- 32-bit rolling RX accumulator, allowing the complete ADC response to be read
-- after the frame finishes rather than requiring a mid-frame AXI read.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

entity tb_axi_spi_adc128s102_sep is
end entity tb_axi_spi_adc128s102_sep;

architecture sim of tb_axi_spi_adc128s102_sep is
    constant CLK_PERIOD : time := 10 ns; -- 100 MHz AXI/system clock

    constant TX_ADDR     : std_logic_vector(31 downto 0) := x"00000000";
    constant RX_ADDR     : std_logic_vector(31 downto 0) := x"00000000";
    constant STATUS_ADDR : std_logic_vector(31 downto 0) := x"00000004";
    constant CTRL_ADDR   : std_logic_vector(31 downto 0) := x"00000008";

    -- ADC timing from the programming description:
    --   DIN captured on SCLK rising edges
    --   DOUT advances on SCLK falling edges
    -- Use CPOL=0, CPHA=0, MSB first, loopback disabled, baud divider=4.
    -- At a 100 MHz system clock this produces approximately 10 MHz SCLK.
    constant CTRL_ADC_MODE0 : std_logic_vector(31 downto 0) := x"00000004";

    signal clk    : std_logic := '0';
    signal resetn : std_logic := '0';

    signal mosi : std_logic;
    signal ssn  : std_logic;
    signal sclk : std_logic;
    signal miso : std_logic;
    signal gpo  : std_logic_vector(31 downto 0);

    signal saxi_awaddr  : std_logic_vector(31 downto 0) := (others => '0');
    signal saxi_awprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal saxi_awvalid : std_logic := '0';
    signal saxi_awready : std_logic;
    signal saxi_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal saxi_wstrb   : std_logic_vector(3 downto 0) := (others => '0');
    signal saxi_wvalid  : std_logic := '0';
    signal saxi_wready  : std_logic;
    signal saxi_bresp   : std_logic_vector(1 downto 0);
    signal saxi_bvalid  : std_logic;
    signal saxi_bready  : std_logic := '0';
    signal saxi_araddr  : std_logic_vector(31 downto 0) := (others => '0');
    signal saxi_arprot  : std_logic_vector(2 downto 0) := (others => '0');
    signal saxi_arvalid : std_logic := '0';
    signal saxi_arready : std_logic;
    signal saxi_rdata   : std_logic_vector(31 downto 0);
    signal saxi_rresp   : std_logic_vector(1 downto 0);
    signal saxi_rvalid  : std_logic;
    signal saxi_rready  : std_logic := '0';

    -- Idealized ADC channel codes.  These stand in for analog sensor voltages.
    signal ch0_code : std_logic_vector(11 downto 0) := x"000";
    signal ch1_code : std_logic_vector(11 downto 0) := x"001";
    signal ch2_code : std_logic_vector(11 downto 0) := x"555";
    signal ch3_code : std_logic_vector(11 downto 0) := x"AAA";
    signal ch4_code : std_logic_vector(11 downto 0) := x"7FF";
    signal ch5_code : std_logic_vector(11 downto 0) := x"800";
    signal ch6_code : std_logic_vector(11 downto 0) := x"FFE";
    signal ch7_code : std_logic_vector(11 downto 0) := x"FFF";

    -- ADC-model observability.
    signal adc_selected_channel : std_logic_vector(2 downto 0);
    signal adc_last_control     : std_logic_vector(7 downto 0);
    signal adc_conversion_done  : std_logic;
    signal adc_frame_error      : std_logic;
    signal adc_first_conversion : std_logic;

    signal rising_edges_in_frame : integer := 0;
    signal allow_short_frame      : std_logic := '0';

begin
    clk <= not clk after CLK_PERIOD/2;

    DUT : entity work.axi_spi_simple
        generic map (
            GPIO_WIDTH      => 32,
            USE_GPIO        => false,
            SAXI_DATA_WIDTH => 32,
            SAXI_ADDR_WIDTH => 2
        )
        port map (
            mosi => mosi,
            ssn  => ssn,
            sclk => sclk,
            miso => miso,
            gpo  => gpo,

            saxi_aclk    => clk,
            saxi_aresetn => resetn,
            saxi_awaddr  => saxi_awaddr,
            saxi_awprot  => saxi_awprot,
            saxi_awvalid => saxi_awvalid,
            saxi_awready => saxi_awready,
            saxi_wdata   => saxi_wdata,
            saxi_wstrb   => saxi_wstrb,
            saxi_wvalid  => saxi_wvalid,
            saxi_wready  => saxi_wready,
            saxi_bresp   => saxi_bresp,
            saxi_bvalid  => saxi_bvalid,
            saxi_bready  => saxi_bready,
            saxi_araddr  => saxi_araddr,
            saxi_arprot  => saxi_arprot,
            saxi_arvalid => saxi_arvalid,
            saxi_arready => saxi_arready,
            saxi_rdata   => saxi_rdata,
            saxi_rresp   => saxi_rresp,
            saxi_rvalid  => saxi_rvalid,
            saxi_rready  => saxi_rready
        );

    ADC_MODEL : entity work.adc128s102_sep_model
        generic map (
            POWERUP_CHANNEL => 7
        )
        port map (
            cs_n => ssn,
            sclk => sclk,
            din  => mosi,
            dout => miso,

            in0_code => ch0_code,
            in1_code => ch1_code,
            in2_code => ch2_code,
            in3_code => ch3_code,
            in4_code => ch4_code,
            in5_code => ch5_code,
            in6_code => ch6_code,
            in7_code => ch7_code,

            selected_channel_o => adc_selected_channel,
            last_control_o     => adc_last_control,
            conversion_done_o  => adc_conversion_done,
            frame_error_o      => adc_frame_error,
            first_conversion_o => adc_first_conversion
        );

    --------------------------------------------------------------------------
    -- Physical-frame monitor.  The ADC requires an integer multiple of 16
    -- rising SCLK edges while CS is low.  One deliberately invalid 8-clock
    -- frame is allowed in the negative test.
    --------------------------------------------------------------------------
    frame_monitor : process(ssn, sclk)
    begin
        if falling_edge(ssn) then
            assert sclk = '0'
                report "ADC frame began while SCLK was not at the configured idle-low level"
                severity failure;
            rising_edges_in_frame <= 0;

        elsif rising_edge(sclk) then
            if ssn = '0' then
                rising_edges_in_frame <= rising_edges_in_frame + 1;
            end if;

        elsif rising_edge(ssn) then
            assert sclk = '0'
                report "ADC frame ended while SCLK was not at the configured idle-low level"
                severity failure;

            if rising_edges_in_frame /= 0 and resetn = '1' then
                if allow_short_frame = '0' then
                    assert (rising_edges_in_frame mod 16) = 0
                        report "ADC frame ended with " & integer'image(rising_edges_in_frame) &
                               " rising SCLK edges; expected an integer multiple of 16"
                        severity failure;
                else
                    assert rising_edges_in_frame = 8
                        report "Expected deliberately invalid 8-clock ADC frame"
                        severity failure;
                end if;
            end if;
        end if;
    end process;

    stimulus : process
        procedure axi_write(
            constant address : in std_logic_vector(31 downto 0);
            constant data    : in std_logic_vector(31 downto 0);
            constant strobe  : in std_logic_vector(3 downto 0) := "1111"
        ) is
            variable aw_done : boolean := false;
            variable w_done  : boolean := false;
        begin
            saxi_awaddr  <= address;
            saxi_awvalid <= '1';
            saxi_wdata   <= data;
            saxi_wstrb   <= strobe;
            saxi_wvalid  <= '1';
            saxi_bready  <= '1';

            while not (aw_done and w_done) loop
                wait until rising_edge(clk);
                if (not aw_done) and saxi_awready = '1' then
                    aw_done := true;
                    saxi_awvalid <= '0';
                end if;
                if (not w_done) and saxi_wready = '1' then
                    w_done := true;
                    saxi_wvalid <= '0';
                end if;
            end loop;

            loop
                wait until rising_edge(clk);
                exit when saxi_bvalid = '1';
            end loop;

            assert saxi_bresp = "00"
                report "AXI write returned non-OKAY"
                severity failure;

            saxi_bready <= '0';
            saxi_awaddr <= (others => '0');
            saxi_wdata  <= (others => '0');
            saxi_wstrb  <= (others => '0');
        end procedure;

        procedure axi_read(
            constant address : in  std_logic_vector(31 downto 0);
            variable data    : out std_logic_vector(31 downto 0)
        ) is
        begin
            saxi_araddr  <= address;
            saxi_arvalid <= '1';
            saxi_rready  <= '1';

            loop
                wait until rising_edge(clk);
                exit when saxi_arready = '1';
            end loop;
            saxi_arvalid <= '0';

            loop
                wait until rising_edge(clk);
                if saxi_rvalid = '1' then
                    data := saxi_rdata;
                    exit;
                end if;
            end loop;

            assert saxi_rresp = "00"
                report "AXI read returned non-OKAY"
                severity failure;

            saxi_rready <= '0';
            saxi_araddr <= (others => '0');
        end procedure;

        procedure wait_for_tx_empty is
            variable status_word : std_logic_vector(31 downto 0);
            variable found       : boolean := false;
        begin
            for poll in 0 to 100 loop
                axi_read(STATUS_ADDR, status_word);
                if status_word(0) = '1' then
                    found := true;
                    exit;
                end if;
            end loop;
            assert found
                report "Timed out waiting for SPTEF/TX-empty"
                severity failure;
        end procedure;

        procedure wait_for_ss_low is
            variable count : integer := 0;
        begin
            while ssn /= '0' loop
                wait until rising_edge(clk);
                count := count + 1;
                assert count < 200
                    report "Timed out waiting for ADC CS/SSN to assert"
                    severity failure;
            end loop;
        end procedure;

        procedure wait_for_ss_high is
            variable count : integer := 0;
        begin
            while ssn /= '1' loop
                wait until rising_edge(clk);
                count := count + 1;
                assert count < 2000
                    report "Timed out waiting for ADC CS/SSN to deassert"
                    severity failure;
            end loop;
        end procedure;

        procedure configure_spi_for_adc is
            variable rd : std_logic_vector(31 downto 0);
        begin
            axi_write(CTRL_ADDR, CTRL_ADC_MODE0);
            axi_read(CTRL_ADDR, rd);

            assert rd(19 downto 0) = CTRL_ADC_MODE0(19 downto 0)
                report "ADC SPI control-register readback mismatch"
                severity failure;
            assert rd(17) = '0' and rd(16) = '0' and rd(19) = '0'
                report "ADC test requires CPOL=0, CPHA=0, MSB-first"
                severity failure;
            report "PASS: ADC SPI configured CPOL=0, CPHA=0, MSB-first, divider=4" severity note;
        end procedure;

        procedure adc_transfer16(
            constant next_channel : in integer range 0 to 7;
            constant expected_code : in std_logic_vector(11 downto 0);
            constant name          : in string;
            constant dc_hi         : in std_logic_vector(1 downto 0) := "00";
            constant dc_lo         : in std_logic_vector(2 downto 0) := "000"
        ) is
            variable ctrl_byte   : std_logic_vector(7 downto 0);
            variable tx_word     : std_logic_vector(31 downto 0);
            variable status_word : std_logic_vector(31 downto 0);
            variable rx_data     : std_logic_vector(31 downto 0);
            variable rx_word     : std_logic_vector(15 downto 0);
        begin
            ctrl_byte := dc_hi & std_logic_vector(to_unsigned(next_channel, 3)) & dc_lo;
            tx_word := (others => '0');
            tx_word(7 downto 0) := ctrl_byte;

            assert ssn = '1'
                report name & ": transfer started while SPI was already active"
                severity failure;

            -- Byte 1 carries the ADC control register.
            axi_write(TX_ADDR, tx_word, "0001");
            wait_for_ss_low;

            -- As soon as byte 1 has moved into the shifter, queue byte 2 so CS
            -- remains low for one continuous 16-clock ADC conversion.
            wait_for_tx_empty;
            assert ssn = '0'
                report name & ": CS rose before the second byte was queued"
                severity failure;

            axi_write(TX_ADDR, x"00000000", "0001");
            wait_for_ss_high;
            wait until rising_edge(clk);

            assert rising_edges_in_frame = 16
                report name & ": expected exactly 16 rising SCLK edges"
                severity failure;
            assert adc_frame_error = '0'
                report name & ": ADC model rejected a supposedly valid frame"
                severity failure;
            assert miso = 'Z'
                report name & ": ADC DOUT must be high-impedance while CS is high"
                severity failure;

            -- STATUS polling must not consume the receive result.
            axi_read(STATUS_ADDR, status_word);
            assert status_word(0) = '1' and status_word(2) = '0'
                report name & ": expected TX empty and SPI idle after frame"
                severity failure;
            assert status_word(1) = '1'
                report name & ": SPRF did not indicate received ADC data"
                severity failure;

            -- One read after the full frame retrieves both bytes from the new
            -- rolling receive accumulator.
            axi_read(RX_ADDR, rx_data);
            rx_word := rx_data(15 downto 0);

            assert rx_word(15 downto 12) = "0000"
                report name & ": ADC response did not contain four leading zeros"
                severity failure;
            assert rx_word(11 downto 0) = expected_code
                report name & ": conversion mismatch. Expected 0x" &
                       to_hstring(expected_code) & " received 0x" &
                       to_hstring(rx_word(11 downto 0))
                severity failure;

            assert adc_selected_channel = std_logic_vector(to_unsigned(next_channel, 3))
                report name & ": next-channel pipeline selection mismatch"
                severity failure;
            assert adc_last_control = ctrl_byte
                report name & ": ADC control-byte capture mismatch"
                severity failure;

            axi_read(STATUS_ADDR, status_word);
            assert status_word(1) = '0'
                report name & ": RX read did not clear SPRF"
                severity failure;

            report "PASS: " & name &
                   "  returned=0x" & to_hstring(rx_word(11 downto 0)) &
                   "  next_channel=IN" & integer'image(next_channel)
                severity note;
        end procedure;

        procedure adc_transfer32_continuous(
            constant current_code        : in std_logic_vector(11 downto 0);
            constant second_code         : in std_logic_vector(11 downto 0);
            constant first_next_channel  : in integer range 0 to 7;
            constant second_next_channel : in integer range 0 to 7
        ) is
            variable ctrl1       : std_logic_vector(7 downto 0);
            variable ctrl2       : std_logic_vector(7 downto 0);
            variable tx          : std_logic_vector(31 downto 0);
            variable rx          : std_logic_vector(31 downto 0);
            variable status_word : std_logic_vector(31 downto 0);
            variable expected32  : std_logic_vector(31 downto 0);
        begin
            ctrl1 := "00" & std_logic_vector(to_unsigned(first_next_channel, 3)) & "000";
            ctrl2 := "00" & std_logic_vector(to_unsigned(second_next_channel, 3)) & "000";
            expected32 := ("0000" & current_code) & ("0000" & second_code);

            tx := (others => '0');
            tx(7 downto 0) := ctrl1;
            axi_write(TX_ADDR, tx, "0001");
            wait_for_ss_low;

            wait_for_tx_empty;
            axi_write(TX_ADDR, x"00000000", "0001");

            -- Queue the control byte for conversion #2 while conversion #1's
            -- second byte is still transmitting.
            wait_for_tx_empty;
            assert ssn = '0'
                report "continuous mode: CS deasserted between conversion segments"
                severity failure;
            tx := (others => '0');
            tx(7 downto 0) := ctrl2;
            axi_write(TX_ADDR, tx, "0001");

            wait_for_tx_empty;
            assert ssn = '0'
                report "continuous mode: CS deasserted before fourth byte"
                severity failure;
            axi_write(TX_ADDR, x"00000000", "0001");

            wait_for_ss_high;
            wait until rising_edge(clk);

            assert rising_edges_in_frame = 32
                report "continuous mode did not produce exactly 32 rising SCLK edges"
                severity failure;
            assert adc_frame_error = '0'
                report "ADC model rejected valid 32-clock continuous frame"
                severity failure;
            assert adc_selected_channel = std_logic_vector(to_unsigned(second_next_channel, 3))
                report "continuous mode final channel selection mismatch"
                severity failure;

            axi_read(STATUS_ADDR, status_word);
            assert status_word(1) = '1' and status_word(2) = '0'
                report "continuous mode RX/status flags incorrect"
                severity failure;

            axi_read(RX_ADDR, rx);
            assert rx = expected32
                report "32-bit rolling RX accumulator mismatch. Expected 0x" &
                       to_hstring(expected32) & " received 0x" & to_hstring(rx)
                severity failure;

            report "PASS: 32-clock continuous conversion frame; RX accumulator = 0x" &
                   to_hstring(rx)
                severity note;
        end procedure;

        variable rd              : std_logic_vector(31 downto 0);
        variable selected_before : std_logic_vector(2 downto 0);
        variable control_before  : std_logic_vector(7 downto 0);
        variable partial_tx      : std_logic_vector(31 downto 0);

    begin
        ----------------------------------------------------------------------
        -- Reset and configuration
        ----------------------------------------------------------------------
        resetn <= '0';
        wait for 5 * CLK_PERIOD;
        wait until rising_edge(clk);
        resetn <= '1';
        wait until rising_edge(clk);

        assert ssn = '1' and sclk = '0'
            report "SPI outputs did not return to idle after reset"
            severity failure;
        assert miso = 'Z'
            report "ADC DOUT must be high-impedance while CS is high"
            severity failure;

        axi_read(STATUS_ADDR, rd);
        assert rd(0) = '1' and rd(1) = '0' and rd(2) = '0'
            report "Unexpected SPI status after reset"
            severity failure;
        report "PASS: reset defaults/status and ADC DOUT tri-state" severity note;

        configure_spi_for_adc;

        ----------------------------------------------------------------------
        -- Power-up behavior / required dummy conversion.
        -- The model uses deterministic POWERUP_CHANNEL=7 only so the regression
        -- is repeatable.  Real hardware's first channel is unspecified, so the
        -- first result must be discarded by software.
        ----------------------------------------------------------------------
        assert adc_first_conversion = '1'
            report "ADC model did not start in first-conversion state"
            severity failure;
        adc_transfer16(0, ch7_code, "dummy power-up conversion (discard result)");
        assert adc_first_conversion = '0'
            report "ADC model did not leave first-conversion state"
            severity failure;
        report "PASS: dummy conversion establishes IN0 for the next conversion" severity note;

        ----------------------------------------------------------------------
        -- Pipeline/channel sweep.  Command N selects the channel returned on
        -- conversion N+1.
        ----------------------------------------------------------------------
        adc_transfer16(1, ch0_code, "pipeline IN0 -> select IN1");
        adc_transfer16(2, ch1_code, "pipeline IN1 -> select IN2");
        adc_transfer16(3, ch2_code, "pipeline IN2 -> select IN3");
        adc_transfer16(4, ch3_code, "pipeline IN3 -> select IN4");
        adc_transfer16(5, ch4_code, "pipeline IN4 -> select IN5");
        adc_transfer16(6, ch5_code, "pipeline IN5 -> select IN6");
        adc_transfer16(7, ch6_code, "pipeline IN6 -> select IN7");
        adc_transfer16(0, ch7_code, "pipeline IN7 -> select IN0");
        report "PASS: all eight ADC channel selections and one-conversion pipeline" severity note;

        ----------------------------------------------------------------------
        -- Continuous conversion mode: two complete 16-clock conversions while
        -- CS remains low for one 32-clock frame.  Starting selected channel is
        -- IN0; conversion #1 selects IN4, so conversion #2 returns IN4.  The
        -- second control byte selects IN5 for the following conversion.
        ----------------------------------------------------------------------
        adc_transfer32_continuous(ch0_code, ch4_code, 4, 5);

        -- Prove pipeline state continues correctly after the continuous frame.
        adc_transfer16(2, ch5_code, "post-continuous pipeline IN5 -> select IN2");

        ----------------------------------------------------------------------
        -- Don't-care control bits must not affect ADD2:ADD0 decoding.
        ----------------------------------------------------------------------
        adc_transfer16(3, ch2_code, "control don't-care bits = 11/101", "11", "101");
        adc_transfer16(4, ch3_code, "verify don't-care command selected IN3");
        report "PASS: control-register don't-care bits do not affect channel selection" severity note;

        ----------------------------------------------------------------------
        -- Dynamic sensor/input scenario.  Change the ideal IN4 code between
        -- conversions and verify the subsequently returned result tracks it.
        ----------------------------------------------------------------------
        ch4_code <= x"321";
        wait until rising_edge(clk);
        adc_transfer16(4, x"321", "dynamic IN4 sensor-code update");
        report "PASS: dynamic ADC input-code update" severity note;

        ----------------------------------------------------------------------
        -- Invalid framing: one byte produces only 8 rising clocks.  The ADC
        -- model must flag the frame and must NOT commit its partial channel
        -- command.  The following valid conversion should therefore still
        -- return IN4.
        ----------------------------------------------------------------------
        selected_before := adc_selected_channel;
        control_before := adc_last_control;
        partial_tx := (others => '0');
        partial_tx(7 downto 0) := "00" & std_logic_vector(to_unsigned(7, 3)) & "000";

        allow_short_frame <= '1';
        axi_write(TX_ADDR, partial_tx, "0001");
        wait_for_ss_low;
        wait_for_ss_high;
        wait until rising_edge(clk);
        allow_short_frame <= '0';

        assert rising_edges_in_frame = 8
            report "Invalid-frame test did not generate exactly 8 rising clocks"
            severity failure;
        assert adc_frame_error = '1'
            report "ADC model did not flag invalid non-16-clock frame"
            severity failure;
        assert adc_selected_channel = selected_before
            report "Incomplete ADC frame incorrectly changed selected channel"
            severity failure;
        assert adc_last_control = control_before
            report "Incomplete ADC frame incorrectly committed control register"
            severity failure;

        -- Consume the one-byte RX indication before the recovery transaction.
        axi_read(RX_ADDR, rd);
        report "PASS: invalid 8-clock ADC frame rejected without changing channel state" severity note;

        -- Starting channel must still be IN4; then select IN6.
        adc_transfer16(6, x"321", "post-invalid-frame recovery IN4 -> select IN6");
        adc_transfer16(0, ch6_code, "pipeline after recovery IN6 -> select IN0");

        ----------------------------------------------------------------------
        -- Final status / idle checks.
        ----------------------------------------------------------------------
        axi_read(STATUS_ADDR, rd);
        assert rd(0) = '1' and rd(1) = '0' and rd(2) = '0'
            report "Final SPI status should be TX-empty, RX-consumed, and idle"
            severity failure;
        assert ssn = '1' and sclk = '0' and miso = 'Z'
            report "Final SPI/ADC pins are not in the expected idle state"
            severity failure;

        report "============================================================" severity note;
        report "ALL AXI -> SPI -> ADC128S102-SEP TESTS PASSED" severity note;
        report "============================================================" severity note;

        finish;
        wait;
    end process;

end architecture sim;
