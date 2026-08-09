-- MIT License
--
-- Copyright (c) 2021 Douglas H. Summerville, Department of Electical and Computer Engineering, Binghamton University
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in all
-- copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
-- SOFTWARE.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axi_spi_simple is
    generic (
        GPIO_WIDTH     : integer := 32;
        USE_GPIO       : boolean := false;
        SAXI_DATA_WIDTH : integer := 32;
        SAXI_ADDR_WIDTH : integer := 2
    );
    port (
        mosi : out std_logic;
        ssn  : out std_logic;
        sclk : out std_logic;
        miso : in std_logic;
        gpo  : out std_logic_vector(GPIO_WIDTH-1 downto 0);

        saxi_aclk    : in std_logic;
        saxi_aresetn : in std_logic;

        saxi_awaddr  : in std_logic_vector(31 downto 0);
        saxi_awprot  : in std_logic_vector(2 downto 0);
        saxi_awvalid : in std_logic;
        saxi_awready : out std_logic;

        saxi_wdata   : in std_logic_vector(SAXI_DATA_WIDTH-1 downto 0);
        saxi_wstrb   : in std_logic_vector((SAXI_DATA_WIDTH/8)-1 downto 0);
        saxi_wvalid  : in std_logic;
        saxi_wready  : out std_logic;

        saxi_bresp   : out std_logic_vector(1 downto 0);
        saxi_bvalid  : out std_logic;
        saxi_bready  : in std_logic;

        saxi_araddr  : in std_logic_vector(31 downto 0);
        saxi_arprot  : in std_logic_vector(2 downto 0);
        saxi_arvalid : in std_logic;
        saxi_arready : out std_logic;

        saxi_rdata   : out std_logic_vector(SAXI_DATA_WIDTH-1 downto 0);
        saxi_rresp   : out std_logic_vector(1 downto 0);
        saxi_rvalid  : out std_logic;
        saxi_rready  : in std_logic
    );
end axi_spi_simple;

architecture arch_imp of axi_spi_simple is

    component axi4_lite_interface_v1_0 is
        generic (
            DATA_BUS_IS_64_BITS : integer range 0 to 1 := 0;
            ADDR_WIDTH : integer range 1 to 12 := 2;
            USE_WRITE_STROBES : boolean := false;
            SUBORDINATE_SYNCHRONOUS_READ_PORT : boolean := true
        );
        port (
            SAXI_ACLK    : in std_logic;
            SAXI_ARESETN : in std_logic;

            SAXI_AWADDR  : in std_logic_vector(31 downto 0);
            SAXI_AWPROT  : in std_logic_vector(2 downto 0);
            SAXI_AWVALID : in std_logic;
            SAXI_AWREADY : out std_logic;

            SAXI_WDATA   : in std_logic_vector(32*(1+DATA_BUS_IS_64_BITS)-1 downto 0);
            SAXI_WSTRB   : in std_logic_vector((4*(1+DATA_BUS_IS_64_BITS))-1 downto 0);
            SAXI_WVALID  : in std_logic;
            SAXI_WREADY  : out std_logic;

            SAXI_BRESP   : out std_logic_vector(1 downto 0);
            SAXI_BVALID  : out std_logic;
            SAXI_BREADY  : in std_logic;

            SAXI_ARADDR  : in std_logic_vector(31 downto 0);
            SAXI_ARPROT  : in std_logic_vector(2 downto 0);
            SAXI_ARVALID : in std_logic;
            SAXI_ARREADY : out std_logic;

            SAXI_RDATA   : out std_logic_vector(32*(1+DATA_BUS_IS_64_BITS)-1 downto 0);
            SAXI_RRESP   : out std_logic_vector(1 downto 0);
            SAXI_RVALID  : out std_logic;
            SAXI_RREADY  : in std_logic;

            read_address  : out std_logic_vector(ADDR_WIDTH-1 downto 0);
            write_address : out std_logic_vector(ADDR_WIDTH-1 downto 0);
            read_enable   : out std_logic;
            write_enable  : out std_logic;
            write_strobe  : out std_logic_vector((4*(1+DATA_BUS_IS_64_BITS))-1 downto 0);
            read_data     : in std_logic_vector(32*(1+DATA_BUS_IS_64_BITS)-1 downto 0);
            write_data    : out std_logic_vector(32*(1+DATA_BUS_IS_64_BITS)-1 downto 0);

            clk    : out std_logic;
            resetn : out std_logic
        );
    end component;

    component spi_peripheral is
        generic (
            GPIO_WIDTH    : integer := 32;
            USE_GPIO      : boolean := false
        );
        port (
            mosi : out std_logic;
            ssn  : out std_logic;
            sclk : out std_logic;
            miso : in std_logic;
            gpo  : out std_logic_vector(GPIO_WIDTH-1 downto 0);

            write_strobe  : in std_logic_vector(3 downto 0);
            read_address  : in std_logic_vector(1 downto 0);
            write_address : in std_logic_vector(1 downto 0);
            write_enable  : in std_logic;
            read_enable   : in std_logic;
            read_data     : out std_logic_vector(31 downto 0);
            write_data    : in std_logic_vector(31 downto 0);
            clk           : in std_logic;
            resetn        : in std_logic
        );
    end component;

    signal read_address  : std_logic_vector(1 downto 0);
    signal write_address : std_logic_vector(1 downto 0);
    signal read_enable   : std_logic;
    signal write_enable  : std_logic;
    signal read_data     : std_logic_vector(31 downto 0);
    signal write_data    : std_logic_vector(31 downto 0);
    signal write_strobe  : std_logic_vector(3 downto 0);

    signal clk    : std_logic;
    signal resetn : std_logic;

    constant is_64b_data_bus : integer := (SAXI_DATA_WIDTH / 32) - 1;

begin

    U0: spi_peripheral
        generic map (
            USE_GPIO   => USE_GPIO,
            GPIO_WIDTH => GPIO_WIDTH
        )
        port map (
            mosi => mosi,
            miso => miso,
            ssn  => ssn,
            sclk => sclk,
            gpo  => gpo,

            read_address  => read_address,
            write_address => write_address,
            read_data     => read_data,
            write_data    => write_data,
            write_enable  => write_enable,
            read_enable   => read_enable,
            write_strobe  => write_strobe,
            clk           => clk,
            resetn        => resetn
        );

    U1: axi4_lite_interface_v1_0
        generic map (
            DATA_BUS_IS_64_BITS => is_64b_data_bus,
            ADDR_WIDTH => 2,
            USE_WRITE_STROBES => true,
            SUBORDINATE_SYNCHRONOUS_READ_PORT => true
        )
        port map (
            SAXI_ACLK    => saxi_aclk,
            SAXI_ARESETN => saxi_aresetn,

            SAXI_AWADDR  => saxi_awaddr,
            SAXI_AWPROT  => saxi_awprot,
            SAXI_AWVALID => saxi_awvalid,
            SAXI_AWREADY => saxi_awready,

            SAXI_WDATA   => saxi_wdata,
            SAXI_WSTRB   => saxi_wstrb,
            SAXI_WVALID  => saxi_wvalid,
            SAXI_WREADY  => saxi_wready,

            SAXI_BRESP   => saxi_bresp,
            SAXI_BVALID  => saxi_bvalid,
            SAXI_BREADY  => saxi_bready,

            SAXI_ARADDR  => saxi_araddr,
            SAXI_ARPROT  => saxi_arprot,
            SAXI_ARVALID => saxi_arvalid,
            SAXI_ARREADY => saxi_arready,

            SAXI_RDATA   => saxi_rdata,
            SAXI_RRESP   => saxi_rresp,
            SAXI_RVALID  => saxi_rvalid,
            SAXI_RREADY  => saxi_rready,

            read_address  => read_address,
            read_enable   => read_enable,
            write_address => write_address,
            write_enable  => write_enable,
            read_data     => read_data,
            write_data    => write_data,
            write_strobe  => write_strobe,

            clk    => clk,
            resetn => resetn
        );

end arch_imp;
