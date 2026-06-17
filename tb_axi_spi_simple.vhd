library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_axi_spi_simple is
end tb_axi_spi_simple;

architecture sim of tb_axi_spi_simple is

    constant CLK_PERIOD : time := 10 ns;

    -- AXI register offsets
    constant REG_TX_RX  : std_logic_vector(31 downto 0) := x"00000000";
    constant REG_STATUS : std_logic_vector(31 downto 0) := x"00000004";
    constant REG_CTRL   : std_logic_vector(31 downto 0) := x"00000008";
    constant REG_GPIO   : std_logic_vector(31 downto 0) := x"0000000C";

    -- SPI control register bits
    constant CTRL_BAUD_DVSR : std_logic_vector(15 downto 0) := x"0002";
    constant CTRL_CPHA      : integer := 16;
    constant CTRL_CPOL      : integer := 17;
    constant CTRL_LOOPBACK  : integer := 18;
    constant CTRL_LSBF      : integer := 19;

    -- Fake external SPI slave response for the MISO test.
    -- This is the byte the testbench drives onto MISO when LOOPBACK = 0.
    constant MISO_RESPONSE : std_logic_vector(7 downto 0) := x"3C";

    signal saxi_aclk    : std_logic := '0';
    signal saxi_aresetn : std_logic := '0';

    signal saxi_awaddr  : std_logic_vector(31 downto 0) := (others => '0');
    signal saxi_awprot  : std_logic_vector(2 downto 0)  := (others => '0');
    signal saxi_awvalid : std_logic := '0';
    signal saxi_awready : std_logic;

    signal saxi_wdata   : std_logic_vector(31 downto 0) := (others => '0');
    signal saxi_wstrb   : std_logic_vector(3 downto 0)  := (others => '0');
    signal saxi_wvalid  : std_logic := '0';
    signal saxi_wready  : std_logic;

    signal saxi_bresp   : std_logic_vector(1 downto 0);
    signal saxi_bvalid  : std_logic;
    signal saxi_bready  : std_logic := '0';

    signal saxi_araddr  : std_logic_vector(31 downto 0) := (others => '0');
    signal saxi_arprot  : std_logic_vector(2 downto 0)  := (others => '0');
    signal saxi_arvalid : std_logic := '0';
    signal saxi_arready : std_logic;

    signal saxi_rdata   : std_logic_vector(31 downto 0);
    signal saxi_rresp   : std_logic_vector(1 downto 0);
    signal saxi_rvalid  : std_logic;
    signal saxi_rready  : std_logic := '0';

    signal mosi : std_logic;
    signal miso : std_logic := '0';
    signal ss   : std_logic;
    signal ssn  : std_logic;
    signal sclk : std_logic;
    signal gpo  : std_logic_vector(31 downto 0);

    --------------------------------------------------------------------------
    -- AXI4-Lite write helper
    --------------------------------------------------------------------------
    procedure axi_write (
        signal clk      : in  std_logic;
        signal awaddr   : out std_logic_vector(31 downto 0);
        signal awvalid  : out std_logic;
        signal awready  : in  std_logic;
        signal wdata    : out std_logic_vector(31 downto 0);
        signal wstrb    : out std_logic_vector(3 downto 0);
        signal wvalid   : out std_logic;
        signal wready   : in  std_logic;
        signal bresp    : in  std_logic_vector(1 downto 0);
        signal bvalid   : in  std_logic;
        signal bready   : out std_logic;
        constant addr   : in  std_logic_vector(31 downto 0);
        constant data   : in  std_logic_vector(31 downto 0);
        constant strb   : in  std_logic_vector(3 downto 0) := "1111"
    ) is
        variable aw_done : boolean := false;
        variable w_done  : boolean := false;
    begin
        awaddr  <= addr;
        wdata   <= data;
        wstrb   <= strb;
        awvalid <= '1';
        wvalid  <= '1';
        bready  <= '1';

        while not (aw_done and w_done) loop
            wait until rising_edge(clk);
            wait for 1 ns;

            if awready = '1' then
                aw_done := true;
            end if;

            if wready = '1' then
                w_done := true;
            end if;

            if aw_done then
                awvalid <= '0';
            end if;

            if w_done then
                wvalid <= '0';
            end if;
        end loop;

        while bvalid /= '1' loop
            wait until rising_edge(clk);
            wait for 1 ns;
        end loop;

        assert bresp = "00"
            report "AXI WRITE ERROR: BRESP was not OKAY"
            severity error;

        wait until rising_edge(clk);
        wait for 1 ns;

        bready <= '0';
        awaddr <= (others => '0');
        wdata  <= (others => '0');
        wstrb  <= (others => '0');
    end procedure;

    --------------------------------------------------------------------------
    -- AXI4-Lite read helper
    --------------------------------------------------------------------------
    procedure axi_read (
        signal clk      : in  std_logic;
        signal araddr   : out std_logic_vector(31 downto 0);
        signal arvalid  : out std_logic;
        signal arready  : in  std_logic;
        signal rdata    : in  std_logic_vector(31 downto 0);
        signal rresp    : in  std_logic_vector(1 downto 0);
        signal rvalid   : in  std_logic;
        signal rready   : out std_logic;
        constant addr   : in  std_logic_vector(31 downto 0);
        variable data   : out std_logic_vector(31 downto 0)
    ) is
    begin
        araddr  <= addr;
        arvalid <= '1';
        rready  <= '1';

        while arready /= '1' loop
            wait until rising_edge(clk);
            wait for 1 ns;
        end loop;

        wait until rising_edge(clk);
        wait for 1 ns;

        arvalid <= '0';

        while rvalid /= '1' loop
            wait until rising_edge(clk);
            wait for 1 ns;
        end loop;

        data := rdata;

        assert rresp = "00"
            report "AXI READ ERROR: RRESP was not OKAY"
            severity error;

        wait until rising_edge(clk);
        wait for 1 ns;

        rready <= '0';
        araddr <= (others => '0');
    end procedure;

    --------------------------------------------------------------------------
    -- Poll status until SPI receive buffer full
    -- Status bit 1 = SPRF
    --------------------------------------------------------------------------
    procedure wait_for_sprf (
        signal clk      : in  std_logic;
        signal araddr   : out std_logic_vector(31 downto 0);
        signal arvalid  : out std_logic;
        signal arready  : in  std_logic;
        signal rdata    : in  std_logic_vector(31 downto 0);
        signal rresp    : in  std_logic_vector(1 downto 0);
        signal rvalid   : in  std_logic;
        signal rready   : out std_logic;
        variable status_data : out std_logic_vector(31 downto 0)
    ) is
        variable got_sprf : boolean := false;
    begin
        got_sprf := false;

        for i in 0 to 3000 loop
            axi_read(
                clk,
                araddr,
                arvalid,
                arready,
                rdata,
                rresp,
                rvalid,
                rready,
                REG_STATUS,
                status_data
            );

            if status_data(1) = '1' then
                got_sprf := true;
                exit;
            end if;

            wait until rising_edge(clk);
        end loop;

        assert got_sprf
            report "FAIL: SPI receive buffer full flag never asserted"
            severity error;
    end procedure;

begin

    --------------------------------------------------------------------------
    -- Clock generation
    --------------------------------------------------------------------------
    clk_gen : process
    begin
        while true loop
            saxi_aclk <= '0';
            wait for CLK_PERIOD / 2;
            saxi_aclk <= '1';
            wait for CLK_PERIOD / 2;
        end loop;
    end process;

    --------------------------------------------------------------------------
    -- Fake SPI slave MISO driver
    --
    -- This is only for the external-MISO test.
    -- It models one external SPI slave on one chip select, ssn.
    --
    -- Behavior:
    --   - Waits until ssn goes low.
    --   - Drives MISO_RESPONSE, MSB first, onto miso.
    --   - Updates miso on falling edges of sclk.
    --   - Returns miso to 0 when ssn goes high.
    --
    -- In the external MISO test, LOOPBACK = 0, so the DUT samples this signal.
    --------------------------------------------------------------------------
    spi_slave_miso_driver : process
        variable bit_index : integer;
    begin
        miso <= '0';

        wait until saxi_aresetn = '1';

        loop
            wait until ssn = '0';

            bit_index := 7;
            miso <= MISO_RESPONSE(bit_index);

            while ssn = '0' loop
                wait on sclk, ssn;

                if ssn = '1' then
                    exit;
                elsif falling_edge(sclk) then
                    if bit_index > 0 then
                        bit_index := bit_index - 1;
                        miso <= MISO_RESPONSE(bit_index);
                    end if;
                end if;
            end loop;

            miso <= '0';
        end loop;
    end process;

    --------------------------------------------------------------------------
    -- DUT
    --------------------------------------------------------------------------
    dut : entity work.axi_spi_simple
        generic map (
            ACTIVE_LOW_SS   => true,
            GPIO_WIDTH      => 32,
            USE_GPIO        => true,
            SAXI_DATA_WIDTH => 32,
            SAXI_ADDR_WIDTH => 2
        )
        port map (
            mosi => mosi,
            ss   => ss,
            ssn  => ssn,
            sclk => sclk,
            miso => miso,
            gpo  => gpo,

            saxi_aclk    => saxi_aclk,
            saxi_aresetn => saxi_aresetn,

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

    --------------------------------------------------------------------------
    -- Main test sequence
    --------------------------------------------------------------------------
    stim : process
        variable rd         : std_logic_vector(31 downto 0);
        variable ctrl_word  : std_logic_vector(31 downto 0);
        variable saw_ss_low : boolean;
    begin
        report "Applying reset..." severity note;

        saxi_aresetn <= '0';
        wait for 10 * CLK_PERIOD;

        saxi_aresetn <= '1';
        wait for 5 * CLK_PERIOD;

        ----------------------------------------------------------------------
        -- Check reset status
        ----------------------------------------------------------------------
        report "Checking reset status..." severity note;

        axi_read(
            saxi_aclk,
            saxi_araddr,
            saxi_arvalid,
            saxi_arready,
            saxi_rdata,
            saxi_rresp,
            saxi_rvalid,
            saxi_rready,
            REG_STATUS,
            rd
        );

        assert rd(0) = '1'
            report "FAIL: SPTEF should be 1 after reset"
            severity error;

        assert rd(1) = '0'
            report "FAIL: SPRF should be 0 after reset"
            severity error;

        assert rd(2) = '0'
            report "FAIL: SPI busy should be 0 after reset"
            severity error;

        ----------------------------------------------------------------------
        -- Test GPIO register
        ----------------------------------------------------------------------
        report "Testing GPIO register..." severity note;

        axi_write(
            saxi_aclk,
            saxi_awaddr,
            saxi_awvalid,
            saxi_awready,
            saxi_wdata,
            saxi_wstrb,
            saxi_wvalid,
            saxi_wready,
            saxi_bresp,
            saxi_bvalid,
            saxi_bready,
            REG_GPIO,
            x"DEADBEEF",
            "1111"
        );

        wait for 2 * CLK_PERIOD;

        assert gpo = x"DEADBEEF"
            report "FAIL: GPO output does not match GPIO register"
            severity error;

        axi_read(
            saxi_aclk,
            saxi_araddr,
            saxi_arvalid,
            saxi_arready,
            saxi_rdata,
            saxi_rresp,
            saxi_rvalid,
            saxi_rready,
            REG_GPIO,
            rd
        );

        assert rd = x"DEADBEEF"
            report "FAIL: GPIO register readback mismatch"
            severity error;

        ----------------------------------------------------------------------
        -- TEST 1: SPI transmit test using internal loopback
        --
        -- LOOPBACK = 1, so external MISO is ignored.
        -- TX byte = A5.
        -- Expected RX byte = A5.
        ----------------------------------------------------------------------
        report "TEST 1: Starting SPI internal loopback transfer. TX A5, expect RX A5..." severity note;

        ctrl_word := (others => '0');
        ctrl_word(15 downto 0) := CTRL_BAUD_DVSR; -- baud divisor = 2
        ctrl_word(CTRL_CPOL) := '0';
        ctrl_word(CTRL_CPHA) := '0';
        ctrl_word(CTRL_LOOPBACK) := '1';
        ctrl_word(CTRL_LSBF) := '0';

        axi_write(
            saxi_aclk,
            saxi_awaddr,
            saxi_awvalid,
            saxi_awready,
            saxi_wdata,
            saxi_wstrb,
            saxi_wvalid,
            saxi_wready,
            saxi_bresp,
            saxi_bvalid,
            saxi_bready,
            REG_CTRL,
            ctrl_word,
            "1111"
        );

        axi_read(
            saxi_aclk,
            saxi_araddr,
            saxi_arvalid,
            saxi_arready,
            saxi_rdata,
            saxi_rresp,
            saxi_rvalid,
            saxi_rready,
            REG_CTRL,
            rd
        );

        assert rd(19 downto 0) = ctrl_word(19 downto 0)
            report "FAIL: Control register readback mismatch for loopback test"
            severity error;

        axi_write(
            saxi_aclk,
            saxi_awaddr,
            saxi_awvalid,
            saxi_awready,
            saxi_wdata,
            saxi_wstrb,
            saxi_wvalid,
            saxi_wready,
            saxi_bresp,
            saxi_bvalid,
            saxi_bready,
            REG_TX_RX,
            x"000000A5",
            "0001"
        );

        saw_ss_low := false;

        for i in 0 to 100 loop
            wait until rising_edge(saxi_aclk);
            wait for 1 ns;

            if ssn = '0' then
                saw_ss_low := true;
                exit;
            end if;
        end loop;

        assert saw_ss_low
            report "FAIL: SSN did not assert low during SPI loopback transfer"
            severity error;

        wait_for_sprf(
            saxi_aclk,
            saxi_araddr,
            saxi_arvalid,
            saxi_arready,
            saxi_rdata,
            saxi_rresp,
            saxi_rvalid,
            saxi_rready,
            rd
        );

        assert rd(2) = '0'
            report "WARNING: SPI busy still high after loopback receive complete"
            severity warning;

        axi_read(
            saxi_aclk,
            saxi_araddr,
            saxi_arvalid,
            saxi_arready,
            saxi_rdata,
            saxi_rresp,
            saxi_rvalid,
            saxi_rready,
            REG_TX_RX,
            rd
        );

        assert rd(7 downto 0) = x"A5"
            report "FAIL: SPI loopback RX data mismatch. Expected A5."
            severity error;

        ----------------------------------------------------------------------
        -- TEST 2: SPI receive test using external MISO
        --
        -- LOOPBACK = 0, so the DUT samples the external MISO pin.
        -- The fake SPI slave process above drives MISO_RESPONSE = 3C.
        --
        -- The AXI write to TX/RX uses dummy data 00. This dummy byte is only
        -- used to start the SPI transaction and generate SCLK.
        --
        -- Expected RX byte = 3C.
        ----------------------------------------------------------------------
        report "TEST 2: Starting SPI external MISO receive. Slave drives 3C, expect RX 3C..." severity note;

        ctrl_word := (others => '0');
        ctrl_word(15 downto 0) := x"0004"; -- slower clock for MISO synchronizer margin
        ctrl_word(CTRL_CPOL) := '0';
        ctrl_word(CTRL_CPHA) := '1';       -- sample later for this simple MISO model
        ctrl_word(CTRL_LOOPBACK) := '0';   -- use actual MISO input
        ctrl_word(CTRL_LSBF) := '0';       -- MSB first

        axi_write(
            saxi_aclk,
            saxi_awaddr,
            saxi_awvalid,
            saxi_awready,
            saxi_wdata,
            saxi_wstrb,
            saxi_wvalid,
            saxi_wready,
            saxi_bresp,
            saxi_bvalid,
            saxi_bready,
            REG_CTRL,
            ctrl_word,
            "1111"
        );

        axi_read(
            saxi_aclk,
            saxi_araddr,
            saxi_arvalid,
            saxi_arready,
            saxi_rdata,
            saxi_rresp,
            saxi_rvalid,
            saxi_rready,
            REG_CTRL,
            rd
        );

        assert rd(19 downto 0) = ctrl_word(19 downto 0)
            report "FAIL: Control register readback mismatch for external MISO test"
            severity error;

        -- Dummy transmit byte. SPI is full-duplex, so the master must transmit
        -- something to generate the SCLK edges needed to receive from MISO.
        axi_write(
            saxi_aclk,
            saxi_awaddr,
            saxi_awvalid,
            saxi_awready,
            saxi_wdata,
            saxi_wstrb,
            saxi_wvalid,
            saxi_wready,
            saxi_bresp,
            saxi_bvalid,
            saxi_bready,
            REG_TX_RX,
            x"00000000",
            "0001"
        );

        saw_ss_low := false;

        for i in 0 to 100 loop
            wait until rising_edge(saxi_aclk);
            wait for 1 ns;

            if ssn = '0' then
                saw_ss_low := true;
                exit;
            end if;
        end loop;

        assert saw_ss_low
            report "FAIL: SSN did not assert low during external MISO transfer"
            severity error;

        wait_for_sprf(
            saxi_aclk,
            saxi_araddr,
            saxi_arvalid,
            saxi_arready,
            saxi_rdata,
            saxi_rresp,
            saxi_rvalid,
            saxi_rready,
            rd
        );

        assert rd(2) = '0'
            report "WARNING: SPI busy still high after external MISO receive complete"
            severity warning;

        axi_read(
            saxi_aclk,
            saxi_araddr,
            saxi_arvalid,
            saxi_arready,
            saxi_rdata,
            saxi_rresp,
            saxi_rvalid,
            saxi_rready,
            REG_TX_RX,
            rd
        );

        assert rd(7 downto 0) = MISO_RESPONSE
            report "FAIL: SPI external MISO RX data mismatch. Expected 3C."
            severity error;

        ----------------------------------------------------------------------
        -- Final status check
        ----------------------------------------------------------------------
        axi_read(
            saxi_aclk,
            saxi_araddr,
            saxi_arvalid,
            saxi_arready,
            saxi_rdata,
            saxi_rresp,
            saxi_rvalid,
            saxi_rready,
            REG_STATUS,
            rd
        );

        assert rd(0) = '1'
            report "FAIL: SPTEF should be 1 after final transfer"
            severity error;

        report "ALL TESTS PASSED." severity note;

        wait;
    end process;

end sim;
