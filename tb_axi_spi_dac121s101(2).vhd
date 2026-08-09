-- Integrated self-checking testbench for:
--   AXI4-Lite -> axi_spi_simple -> spi_peripheral -> DAC121S101 model
--
-- The SPI controller is byte oriented, while the DAC requires one continuous
-- 16-bit SYNC-low frame.  Each DAC command is therefore transmitted as two
-- queued 8-bit writes.  The second byte is written only after SPTEF indicates
-- that the first byte has moved from the TX register into the SPI shifter,
-- while the first byte is still in progress.  This keeps SSN/SYNC low across
-- all 16 falling SCLK edges.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

entity tb_axi_spi_dac121s101 is
end entity tb_axi_spi_dac121s101;

architecture sim of tb_axi_spi_dac121s101 is
    constant CLK_PERIOD : time := 10 ns;

    -- AXI local register addresses.  axi4_lite_interface_v1_0 uses address
    -- bits [3:2] for the four 32-bit registers.
    constant TX_ADDR     : std_logic_vector(31 downto 0) := x"00000000";
    constant STATUS_ADDR : std_logic_vector(31 downto 0) := x"00000004";
    constant CTRL_ADDR   : std_logic_vector(31 downto 0) := x"00000008";
    constant RX_ADDR     : std_logic_vector(31 downto 0) := x"00000000";

    -- Mode 1, MSB first, loopback disabled, baud divider = 4.
    -- bit 17 CPOL = 0
    -- bit 16 CPHA = 1
    -- bit 19 LSBF = 0
    constant CTRL_DAC_MODE1 : std_logic_vector(31 downto 0) := x"00010004";

    signal clk    : std_logic := '0';
    signal resetn : std_logic := '0';

    signal mosi : std_logic;
    signal ssn  : std_logic;
    signal sclk : std_logic;
    signal miso : std_logic := '0';
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

    -- DAC-model observability.
    signal dac_code    : std_logic_vector(11 downto 0);
    signal dac_pd_mode : std_logic_vector(1 downto 0);
    signal dac_update  : std_logic;
    signal dac_aborted : std_logic;

    signal falling_edges_in_frame : integer := 0;
    -- Allows one deliberately short frame in the negative framing test.
    signal allow_short_frame      : std_logic := '0';

begin
    clk <= not clk after CLK_PERIOD/2;

    DUT : entity work.axi_spi_simple
        generic map (
            GPIO_WIDTH     => 32,
            USE_GPIO       => false,
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

    DAC_MODEL : entity work.dac121s101_model
        port map (
            sync_n     => ssn,
            sclk       => sclk,
            din        => mosi,
            dac_code_o => dac_code,
            pd_mode_o  => dac_pd_mode,
            update_o   => dac_update,
            aborted_o  => dac_aborted
        );

    --------------------------------------------------------------------------
    -- Independent frame monitor.  It checks the physical SPI framing in
    -- addition to the behavioral checks inside the DAC model.
    --------------------------------------------------------------------------
    frame_monitor : process(ssn, sclk)
    begin
        if falling_edge(ssn) then
            assert sclk = '0'
                report "SSN asserted while SCLK was not at the Mode-1 idle level (0)"
                severity failure;
            falling_edges_in_frame <= 0;
        elsif falling_edge(sclk) then
            if ssn = '0' then
                falling_edges_in_frame <= falling_edges_in_frame + 1;
            end if;
        elsif rising_edge(ssn) then
            assert sclk = '0'
                report "SSN deasserted while SCLK was not at the Mode-1 idle level (0)"
                severity failure;
            -- Ignore the intentionally aborted frame when reset is asserted.
            -- Any non-reset frame must contain exactly 16 falling edges.
            if falling_edges_in_frame /= 0 and resetn = '1' then
                if allow_short_frame = '0' then
                    assert falling_edges_in_frame = 16
                        report "SPI frame ended with " & integer'image(falling_edges_in_frame) &
                               " falling SCLK edges; DAC121S101 requires 16"
                        severity failure;
                else
                    assert falling_edges_in_frame < 16
                        report "Expected a deliberately short DAC frame, but frame was not short"
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
                report "AXI write returned a non-OKAY response"
                severity failure;

            saxi_bready <= '0';
            saxi_awaddr <= (others => '0');
            saxi_wdata  <= (others => '0');
            saxi_wstrb  <= (others => '0');
        end procedure;

        -- Exercise AXI4-Lite's independent address/data channels by sending
        -- AW first and W later.  This should still produce exactly one write.
        procedure axi_write_aw_first(
            constant address    : in std_logic_vector(31 downto 0);
            constant data       : in std_logic_vector(31 downto 0);
            constant strobe     : in std_logic_vector(3 downto 0) := "1111";
            constant gap_cycles : in natural := 3
        ) is
        begin
            saxi_bready  <= '1';
            saxi_awaddr  <= address;
            saxi_awvalid <= '1';

            loop
                wait until rising_edge(clk);
                exit when saxi_awready = '1';
            end loop;
            saxi_awvalid <= '0';

            for i in 1 to gap_cycles loop
                wait until rising_edge(clk);
            end loop;

            saxi_wdata  <= data;
            saxi_wstrb  <= strobe;
            saxi_wvalid <= '1';
            loop
                wait until rising_edge(clk);
                exit when saxi_wready = '1';
            end loop;
            saxi_wvalid <= '0';

            loop
                wait until rising_edge(clk);
                exit when saxi_bvalid = '1';
            end loop;

            assert saxi_bresp = "00"
                report "AW-first AXI write returned a non-OKAY response"
                severity failure;

            saxi_bready <= '0';
            saxi_awaddr <= (others => '0');
            saxi_wdata  <= (others => '0');
            saxi_wstrb  <= (others => '0');
        end procedure;

        -- Same test in the opposite channel order: W arrives before AW.
        procedure axi_write_w_first(
            constant address    : in std_logic_vector(31 downto 0);
            constant data       : in std_logic_vector(31 downto 0);
            constant strobe     : in std_logic_vector(3 downto 0) := "1111";
            constant gap_cycles : in natural := 3
        ) is
        begin
            saxi_bready <= '1';
            saxi_wdata  <= data;
            saxi_wstrb  <= strobe;
            saxi_wvalid <= '1';

            loop
                wait until rising_edge(clk);
                exit when saxi_wready = '1';
            end loop;
            saxi_wvalid <= '0';

            for i in 1 to gap_cycles loop
                wait until rising_edge(clk);
            end loop;

            saxi_awaddr  <= address;
            saxi_awvalid <= '1';
            loop
                wait until rising_edge(clk);
                exit when saxi_awready = '1';
            end loop;
            saxi_awvalid <= '0';

            loop
                wait until rising_edge(clk);
                exit when saxi_bvalid = '1';
            end loop;

            assert saxi_bresp = "00"
                report "W-first AXI write returned a non-OKAY response"
                severity failure;

            saxi_bready <= '0';
            saxi_awaddr <= (others => '0');
            saxi_wdata  <= (others => '0');
            saxi_wstrb  <= (others => '0');
        end procedure;

        -- Verify BVALID/BRESP are held while the AXI master applies write-response
        -- backpressure.
        procedure axi_write_b_backpressure(
            constant address : in std_logic_vector(31 downto 0);
            constant data    : in std_logic_vector(31 downto 0);
            constant strobe  : in std_logic_vector(3 downto 0) := "1111";
            constant hold_cycles : in natural := 5
        ) is
            variable aw_done : boolean := false;
            variable w_done  : boolean := false;
            variable held_resp : std_logic_vector(1 downto 0);
        begin
            saxi_awaddr  <= address;
            saxi_awvalid <= '1';
            saxi_wdata   <= data;
            saxi_wstrb   <= strobe;
            saxi_wvalid  <= '1';
            saxi_bready  <= '0';

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
            held_resp := saxi_bresp;

            for i in 1 to hold_cycles loop
                wait until rising_edge(clk);
                assert saxi_bvalid = '1'
                    report "BVALID dropped while BREADY was low"
                    severity failure;
                assert saxi_bresp = held_resp
                    report "BRESP changed while BREADY was low"
                    severity failure;
            end loop;

            assert held_resp = "00"
                report "Backpressured AXI write returned a non-OKAY response"
                severity failure;

            saxi_bready <= '1';
            wait until rising_edge(clk);
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
                report "AXI read returned a non-OKAY response"
                severity failure;

            saxi_rready <= '0';
            saxi_araddr <= (others => '0');
        end procedure;

        -- Verify RVALID/RDATA are held while the AXI master applies read-data
        -- backpressure.  This directly regresses the delayed-RREADY behavior.
        procedure axi_read_r_backpressure(
            constant address : in  std_logic_vector(31 downto 0);
            variable data    : out std_logic_vector(31 downto 0);
            constant hold_cycles : in natural := 5
        ) is
            variable held_data : std_logic_vector(31 downto 0);
            variable held_resp : std_logic_vector(1 downto 0);
        begin
            saxi_araddr  <= address;
            saxi_arvalid <= '1';
            saxi_rready  <= '0';

            loop
                wait until rising_edge(clk);
                exit when saxi_arready = '1';
            end loop;
            saxi_arvalid <= '0';

            loop
                wait until rising_edge(clk);
                exit when saxi_rvalid = '1';
            end loop;

            held_data := saxi_rdata;
            held_resp := saxi_rresp;

            for i in 1 to hold_cycles loop
                wait until rising_edge(clk);
                assert saxi_rvalid = '1'
                    report "RVALID dropped while RREADY was low"
                    severity failure;
                assert saxi_rdata = held_data
                    report "RDATA changed while RREADY was low"
                    severity failure;
                assert saxi_rresp = held_resp
                    report "RRESP changed while RREADY was low"
                    severity failure;
            end loop;

            assert held_resp = "00"
                report "Backpressured AXI read returned a non-OKAY response"
                severity failure;

            data := held_data;
            saxi_rready <= '1';
            wait until rising_edge(clk);
            saxi_rready <= '0';
            saxi_araddr <= (others => '0');
        end procedure;

        procedure wait_for_tx_empty is
            variable status_word : std_logic_vector(31 downto 0);
            variable found       : boolean := false;
        begin
            for poll in 0 to 50 loop
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
                assert count < 100
                    report "Timed out waiting for DAC SYNC/SSN to assert"
                    severity failure;
            end loop;
        end procedure;

        procedure wait_for_ss_high is
            variable count : integer := 0;
        begin
            while ssn /= '1' loop
                wait until rising_edge(clk);
                count := count + 1;
                assert count < 500
                    report "Timed out waiting for DAC SYNC/SSN to deassert"
                    severity failure;
            end loop;
        end procedure;

        procedure wait_for_dac_update is
            variable count : integer := 0;
        begin
            while dac_update /= '1' loop
                wait until rising_edge(clk);
                count := count + 1;
                assert count < 500
                    report "Timed out waiting for DAC 16-bit update"
                    severity failure;
            end loop;
        end procedure;

        procedure configure_spi_for_dac is
            variable rd : std_logic_vector(31 downto 0);
        begin
            axi_write(CTRL_ADDR, CTRL_DAC_MODE1);
            axi_read(CTRL_ADDR, rd);

            assert rd(19 downto 0) = CTRL_DAC_MODE1(19 downto 0)
                report "SPI control-register readback mismatch"
                severity failure;

            assert rd(17) = '0' and rd(16) = '1' and rd(19) = '0'
                report "DAC test requires CPOL=0, CPHA=1, MSB-first"
                severity failure;
        end procedure;

        procedure send_dac_command(
            constant pd       : in std_logic_vector(1 downto 0);
            constant code     : in std_logic_vector(11 downto 0);
            constant name     : in string;
            constant dontcare : in std_logic_vector(1 downto 0) := "00"
        ) is
            variable command_word : std_logic_vector(15 downto 0);
            variable tx_hi        : std_logic_vector(31 downto 0);
            variable tx_lo        : std_logic_vector(31 downto 0);
            variable millivolts   : integer;
        begin
            command_word := dontcare & pd & code;
            tx_hi := (others => '0');
            tx_lo := (others => '0');
            tx_hi(7 downto 0) := command_word(15 downto 8);
            tx_lo(7 downto 0) := command_word(7 downto 0);

            assert ssn = '1'
                report name & ": started while previous SPI frame was still active"
                severity failure;

            -- Start byte 1.
            axi_write(TX_ADDR, tx_hi, "0001");
            wait_for_ss_low;

            -- Wait until byte 1 has been copied into the shifter.  The transfer
            -- is still active because the baud divider is intentionally slower
            -- than AXI.  Queue byte 2 now so ST_RESTART continues with SSN low.
            wait_for_tx_empty;
            assert ssn = '0'
                report name & ": SYNC rose before second byte could be queued"
                severity failure;

            axi_write(TX_ADDR, tx_lo, "0001");

            wait_for_dac_update;

            assert ssn = '0'
                report name & ": DAC updated after SYNC had already deasserted"
                severity failure;

            assert dac_code = code
                report name & ": DAC code mismatch. Expected 0x" &
                       to_hstring(code) & " received 0x" & to_hstring(dac_code)
                severity failure;

            assert dac_pd_mode = pd
                report name & ": power-down mode mismatch"
                severity failure;

            assert dac_aborted = '0'
                report name & ": DAC model detected an aborted/incomplete frame"
                severity failure;

            assert falling_edges_in_frame = 16
                report name & ": DAC updated without exactly 16 falling SCLK edges"
                severity failure;

            -- For a nominal 3.3 V supply/reference, report the ideal code voltage.
            -- DAC121S101 uses a 12-bit code range, so VOUT ~= code/4096 * VA.
            millivolts := (to_integer(unsigned(code)) * 3300) / 4096;

            report "PASS: " & name &
                   "  PD=" & integer'image(to_integer(unsigned(pd))) &
                   "  code=0x" & to_hstring(code) &
                   "  ideal@3.3V=" & integer'image(millivolts) & " mV"
                severity note;

            wait_for_ss_high;
            wait until rising_edge(clk);
        end procedure;

        -- Regression for the SPTEF acknowledge/write collision that previously
        -- could lose byte #2.  The second AXI write is launched immediately after
        -- the first write response.  With this RTL's pipeline, its subordinate
        -- write_enable reaches spi_peripheral on the same rising edge on which
        -- byte #1 is acknowledged into the shift register.
        procedure send_dac_command_sptef_collision(
            constant pd   : in std_logic_vector(1 downto 0);
            constant code : in std_logic_vector(11 downto 0);
            constant name : in string
        ) is
            variable command_word : std_logic_vector(15 downto 0);
            variable tx_hi        : std_logic_vector(31 downto 0);
            variable tx_lo        : std_logic_vector(31 downto 0);
            variable status_word  : std_logic_vector(31 downto 0);
        begin
            command_word := "00" & pd & code;
            tx_hi := (others => '0');
            tx_lo := (others => '0');
            tx_hi(7 downto 0) := command_word(15 downto 8);
            tx_lo(7 downto 0) := command_word(7 downto 0);

            assert ssn = '1'
                report name & ": collision regression started while SPI was active"
                severity failure;

            -- No SPTEF polling between these writes: intentionally hit the
            -- acknowledge + replacement-write corner case.
            axi_write(TX_ADDR, tx_hi, "0001");
            axi_write(TX_ADDR, tx_lo, "0001");

            wait_for_ss_low;
            axi_read(STATUS_ADDR, status_word);

            assert status_word(2) = '1'
                report name & ": SPI_BUSY was not asserted during collision regression"
                severity failure;
            assert status_word(0) = '0'
                report name & ": SPTEF incorrectly became empty after ACK + new TX write collision"
                severity failure;

            wait_for_dac_update;

            assert dac_code = code and dac_pd_mode = pd
                report name & ": second byte was lost/corrupted in SPTEF collision regression"
                severity failure;
            assert falling_edges_in_frame = 16
                report name & ": collision regression did not produce one continuous 16-clock frame"
                severity failure;
            assert dac_aborted = '0'
                report name & ": DAC rejected collision-regression frame"
                severity failure;

            wait_for_ss_high;
            report "PASS: " & name & " -- ACK + TX-write collision preserved queued byte and 16-bit frame"
                severity note;
            wait until rising_edge(clk);
        end procedure;

        variable rd              : std_logic_vector(31 downto 0);
        variable code_before_abort : std_logic_vector(11 downto 0);
        variable partial_command : std_logic_vector(15 downto 0);
        variable partial_tx      : std_logic_vector(31 downto 0);
        variable rd_hold         : std_logic_vector(31 downto 0);
        variable lfsr12          : std_logic_vector(11 downto 0) := x"ACE";

    begin
        ----------------------------------------------------------------------
        -- Reset
        ----------------------------------------------------------------------
        resetn <= '0';
        wait for 5 * CLK_PERIOD;
        wait until rising_edge(clk);
        resetn <= '1';
        wait until rising_edge(clk);

        assert ssn = '1'
            report "SSN must be inactive after reset"
            severity failure;

        axi_read(STATUS_ADDR, rd);
        assert rd(0) = '1'
            report "SPTEF should be set after reset"
            severity failure;
        assert rd(1) = '0'
            report "SPRF should be clear after reset"
            severity failure;
        assert rd(2) = '0'
            report "SPI should not be busy after reset"
            severity failure;

        axi_read(CTRL_ADDR, rd);
        assert rd(19 downto 0) = x"00000"
            report "SPI control register should reset to zero"
            severity failure;
        assert sclk = '0'
            report "SCLK should be low while idle after reset"
            severity failure;
        report "PASS: reset defaults/status" severity note;

        configure_spi_for_dac;

        ----------------------------------------------------------------------
        -- AXI4-Lite protocol / register-interface regression
        ----------------------------------------------------------------------
        -- Address and data are independent AXI channels; verify either can arrive
        -- first without duplication or loss.
        axi_write_aw_first(CTRL_ADDR, x"00010006", "1111", 3);
        axi_read(CTRL_ADDR, rd);
        assert rd(19 downto 0) = x"10006"
            report "AW-first write did not program the control register correctly"
            severity failure;

        axi_write_w_first(CTRL_ADDR, CTRL_DAC_MODE1, "1111", 3);
        axi_read(CTRL_ADDR, rd);
        assert rd(19 downto 0) = CTRL_DAC_MODE1(19 downto 0)
            report "W-first write did not restore the control register correctly"
            severity failure;
        report "PASS: AXI AW-before-W and W-before-AW ordering" severity note;

        -- Verify byte strobes on the control register, then restore the DAC setup.
        axi_write(CTRL_ADDR, x"0000002A", "0001");
        axi_read(CTRL_ADDR, rd);
        assert rd(7 downto 0) = x"2A" and rd(19 downto 8) = CTRL_DAC_MODE1(19 downto 8)
            report "Control-register byte-write strobe behavior is incorrect"
            severity failure;
        axi_write(CTRL_ADDR, CTRL_DAC_MODE1, "1111");
        report "PASS: AXI WSTRB byte-write behavior" severity note;

        -- A TX write without WSTRB(0) must not enqueue/transmit a byte.
        axi_write(TX_ADDR, x"0000AB00", "0010");
        for i in 1 to 12 loop
            wait until rising_edge(clk);
            assert ssn = '1'
                report "TX write with WSTRB(0)=0 unexpectedly started SPI"
                severity failure;
        end loop;
        axi_read(STATUS_ADDR, rd);
        assert rd(0) = '1' and rd(2) = '0'
            report "Ignored TX byte-strobe write changed TX-empty/busy status"
            severity failure;
        report "PASS: TX register ignores writes when WSTRB(0)=0" severity note;

        -- Hold both AXI response channels stalled to make sure response data stays
        -- valid and stable until the master accepts it.
        axi_write_b_backpressure(CTRL_ADDR, CTRL_DAC_MODE1, "1111", 6);
        axi_read_r_backpressure(CTRL_ADDR, rd_hold, 6);
        assert rd_hold(19 downto 0) = CTRL_DAC_MODE1(19 downto 0)
            report "Backpressured control read returned incorrect data"
            severity failure;
        report "PASS: AXI BREADY and RREADY backpressure" severity note;

        ----------------------------------------------------------------------
        -- Realistic DAC programming scenarios
        ----------------------------------------------------------------------
        send_dac_command("00", x"000", "normal zero-scale");
        send_dac_command("00", x"800", "normal midscale");
        send_dac_command("00", x"FFF", "normal full-scale");
        send_dac_command("00", x"A5C", "normal arbitrary code");

        -- Exercise all three power-down modes.  The code field is still loaded
        -- into the DAC register when the command executes.
        send_dac_command("01", x"321", "power-down 1k-to-GND");
        send_dac_command("10", x"654", "power-down 100k-to-GND");
        send_dac_command("11", x"987", "power-down high-impedance");
        send_dac_command("00", x"987", "wake from power-down");

        -- Representative application ramp / setpoint changes.
        send_dac_command("00", x"100", "setpoint ramp 1");
        send_dac_command("00", x"400", "setpoint ramp 2");
        send_dac_command("00", x"800", "setpoint ramp 3");
        send_dac_command("00", x"C00", "setpoint ramp 4");
        send_dac_command("00", x"FFF", "setpoint ramp 5");

        -- Repeat the exact same code to prove that a new 16-bit write is still
        -- recognized even when dac_code itself does not change value.
        send_dac_command("00", x"FFF", "repeat full-scale command");

        -- Bit-boundary patterns catch MSB/LSB order, byte order, and off-by-one
        -- shift errors that simple zero/full-scale vectors can miss.
        send_dac_command("00", x"001", "boundary one-LSB code");
        send_dac_command("00", x"FFE", "boundary full-scale-minus-one");
        send_dac_command("00", x"555", "alternating 0101 pattern");
        send_dac_command("00", x"AAA", "alternating 1010 pattern");

        -- DB15:DB14 are documented don't-care bits and must not affect PD/code.
        send_dac_command("00", x"135", "don't-care bits = 11", "11");
        report "PASS: DAC DB15:DB14 don't-care behavior" severity note;

        -- Direct regression for the fixed SPTEF race.  This is intentionally
        -- faster than the normal polling-based queueing path.
        send_dac_command_sptef_collision("00", x"6D3", "SPTEF ACK/write collision");

        -- Deterministic pseudo-random code sweep to increase serializer/bit-pattern
        -- coverage without making failures non-repeatable.
        for test_index in 1 to 16 loop
            lfsr12 := lfsr12(10 downto 0) &
                      (lfsr12(11) xor lfsr12(10) xor lfsr12(9) xor lfsr12(3));
            send_dac_command("00", lfsr12,
                             "pseudo-random DAC code " & integer'image(test_index));
        end loop;
        report "PASS: 16-vector deterministic pseudo-random DAC sweep" severity note;

        ----------------------------------------------------------------------
        -- Negative framing test: deliberately send only one byte and allow SSN
        -- to deassert.  The physical DAC model must reject the resulting 8-clock
        -- frame and preserve its prior register contents.
        ----------------------------------------------------------------------
        code_before_abort := dac_code;
        partial_command := "00" & "00" & x"2A7";
        partial_tx := (others => '0');
        partial_tx(7 downto 0) := partial_command(15 downto 8);

        allow_short_frame <= '1';
        axi_write(TX_ADDR, partial_tx, "0001");
        wait_for_ss_low;
        wait_for_ss_high;
        wait for 1 ns;

        assert falling_edges_in_frame = 8
            report "Expected exactly eight clocks in deliberately truncated single-byte frame"
            severity failure;
        assert dac_aborted = '1'
            report "DAC did not reject the intentionally truncated 8-clock frame"
            severity failure;
        assert dac_code = code_before_abort
            report "Truncated one-byte frame changed DAC code"
            severity failure;

        allow_short_frame <= '0';
        report "PASS: late/missing second byte creates an invalid 8-clock frame and is rejected"
            severity note;
        wait until rising_edge(clk);

        ----------------------------------------------------------------------
        -- Aborted-frame scenario.
        -- Send only the first byte, then reset the SPI bridge while SYNC is low.
        -- The DAC must reject the incomplete frame and preserve its old state.
        ----------------------------------------------------------------------
        code_before_abort := dac_code;
        partial_command := "00" & "00" & x"555";
        partial_tx := (others => '0');
        partial_tx(7 downto 0) := partial_command(15 downto 8);

        axi_write(TX_ADDR, partial_tx, "0001");
        wait_for_ss_low;

        -- Observe several DAC sampling edges, but fewer than 16.
        for edge_index in 1 to 4 loop
            wait until falling_edge(sclk);
        end loop;

        resetn <= '0';
        wait until rising_edge(clk);
        wait for 1 ns;

        assert ssn = '1'
            report "Reset did not terminate the active SPI frame"
            severity failure;

        assert dac_aborted = '1'
            report "DAC model did not flag the incomplete frame as aborted"
            severity failure;

        assert dac_code = code_before_abort
            report "Incomplete frame incorrectly changed DAC output code"
            severity failure;

        report "PASS: incomplete SPI frame was rejected and DAC state was preserved"
            severity note;

        wait for 3 * CLK_PERIOD;
        wait until rising_edge(clk);
        resetn <= '1';
        wait until rising_edge(clk);
        configure_spi_for_dac;

        -- Confirm normal operation recovers after the interrupted transaction.
        send_dac_command("00", x"555", "post-reset recovery");

        ----------------------------------------------------------------------
        -- Status/RX flag semantics.  Each completed SPI byte sets SPRF.  Reading
        -- STATUS must not consume it; reading RX must consume it.
        ----------------------------------------------------------------------
        axi_read(STATUS_ADDR, rd);
        assert rd(1) = '1'
            report "SPRF was not set after completed SPI traffic"
            severity failure;

        axi_read(STATUS_ADDR, rd_hold);
        assert rd_hold(1) = '1'
            report "STATUS read incorrectly cleared SPRF"
            severity failure;

        axi_read(RX_ADDR, rd);
        assert rd(7 downto 0) = x"00"
            report "Expected zero RX data with DAC MISO tied low"
            severity failure;

        axi_read(STATUS_ADDR, rd_hold);
        assert rd_hold(1) = '0'
            report "RX-data read did not clear SPRF"
            severity failure;
        assert rd_hold(0) = '1' and rd_hold(2) = '0'
            report "Final status should be TX-empty and not busy"
            severity failure;
        report "PASS: SPRF/status polling and RX-consume semantics" severity note;

        report "============================================================" severity note;
        report "ALL AXI -> SPI -> DAC121S101 TESTS PASSED" severity note;
        report "============================================================" severity note;

        stop;
        wait;
    end process;

end architecture sim;
