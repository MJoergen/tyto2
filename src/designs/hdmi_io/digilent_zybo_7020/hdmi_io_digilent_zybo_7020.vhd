--------------------------------------------------------------------------------
-- hdmi_io_digilent_zybo_7020.vhd                                             --
-- Board specific top level wrapper for the hdmi_io design.                   --
--------------------------------------------------------------------------------
-- (C) Copyright 2023 Adam Barnes <ambarnes@gmail.com>                        --
-- This file is part of The Tyto Project. The Tyto Project is free software:  --
-- you can redistribute it and/or modify it under the terms of the GNU Lesser --
-- General Public License as published by the Free Software Foundation,       --
-- either version 3 of the License, or (at your option) any later version.    --
-- The Tyto Project is distributed in the hope that it will be useful, but    --
-- WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY --
-- or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public     --
-- License for more details. You should have received a copy of the GNU       --
-- Lesser General Public License along with The Tyto Project. If not, see     --
-- https://www.gnu.org/licenses/.                                             --
--------------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;

library unisim;
  use unisim.vcomponents.all;

library work;
  use work.tyto_types_pkg.all;
  use work.mmcm_pkg.all;
  use work.hdmi_rx_selectio_pkg.all;
  use work.hdmi_tx_selectio_pkg.all;

entity hdmi_io_digilent_zybo_7020 is
  port (
    -- clock 125 MHz
    sysclk_i            : in    std_logic;

    -- LEDs, buttons and switches
    sw_i                : in    std_logic_vector(3 downto 0);
    btn_i               : in    std_logic_vector(3 downto 0);
    led_o               : out   std_logic_vector(3 downto 0);
    led5_r_o            : out   std_logic;
    led5_g_o            : out   std_logic;
    led5_b_o            : out   std_logic;
    led6_r_o            : out   std_logic;
    led6_g_o            : out   std_logic;
    led6_b_o            : out   std_logic;

    -- HDMI RX
    hdmi_rx_hpd_o       : out   std_logic;
    hdmi_rx_scl_io      : inout std_logic;
    hdmi_rx_sda_io      : inout std_logic;
    hdmi_rx_clk_p_i     : in    std_logic;
    hdmi_rx_clk_n_i     : in    std_logic;
    hdmi_rx_p_i         : in    std_logic_vector(0 to 2);
    hdmi_rx_n_i         : in    std_logic_vector(0 to 2);
    hdmi_rx_cec_i       : in    std_logic;

    -- HDMI TX
    hdmi_tx_hpd_i       : in    std_logic;
    hdmi_tx_scl_io      : inout std_logic;
    hdmi_tx_sda_io      : inout std_logic;
    hdmi_tx_clk_p_o     : out   std_logic;
    hdmi_tx_clk_n_o     : out   std_logic;
    hdmi_tx_p_o         : out   std_logic_vector(0 to 2);
    hdmi_tx_n_o         : out   std_logic_vector(0 to 2);
    hdmi_tx_cec_o       : out   std_logic;

    -- PMODs
    ja_io               : inout std_logic_vector(7 downto 0);
    jb_io               : inout std_logic_vector(7 downto 0);
    jc_io               : inout std_logic_vector(7 downto 0);
    jd_io               : inout std_logic_vector(7 downto 0);
    je_io               : inout std_logic_vector(7 downto 0);

    -- Audio codex
    -- SSM2603CPZ
    -- I2C address 0011010
    ac_bclk_io          : inout std_logic;
    ac_mclk_i           : in    std_logic;
    ac_muten_o          : out   std_logic;
    ac_pbdat_o          : out   std_logic;
    ac_pblrc_io         : inout std_logic;
    ac_recdat_i         : in    std_logic;
    ac_reclrc_io        : inout std_logic;
    ac_scl_o            : out   std_logic;
    ac_sda_io           : inout std_logic;

    -- RTL8211E-VL
    eth_int_pu_b_i      : in    std_logic; -- pin 20, INTB
    eth_rst_b_o         : out   std_logic; -- pin 29, PHYRSTB

    -- Jumper J14
    fan_fb_pu_i         : in    std_logic;

    -- Jumper J2
    cam_clk_i           : in    std_logic;
    cam_gpio_i          : in    std_logic;
    cam_scl_io          : inout std_logic;
    cam_sda_io          : inout std_logic;
--    dphy_clk_lp_n       : in    std_logic;
--    dphy_clk_lp_p       : in    std_logic;
--    dphy_data_lp_n      : in    std_logic_vector(1 downto 0);
--    dphy_data_lp_p      : in    std_logic_vector(1 downto 0);
--    dphy_hs_clock_clk_n : in    std_logic;
--    dphy_hs_clock_clk_p : in    std_logic;
--    dphy_data_hs_n      : in    std_logic_vector(1 downto 0);
--    dphy_data_hs_p      : in    std_logic_vector(1 downto 0);

    -- ATSHA204A-SSHCZ-T
    crypto_sda_io       : inout std_logic
  );
end entity hdmi_io_digilent_zybo_7020;

architecture synth of hdmi_io_digilent_zybo_7020 is

  signal rst_a          : std_logic;
  signal clk_100m       : std_logic;
  signal clk_200m       : std_logic;
  signal rst_200m_s     : std_logic_vector(0 to 1);
  signal rst_200m       : std_logic;
  signal idelayctrl_rdy : std_logic;
  signal rst_100m_s     : std_logic_vector(0 to 1);
  signal rst_100m       : std_logic;

  signal hdmi_rx_clku   : std_logic;
  signal hdmi_rx_clk    : std_logic;
  signal hdmi_rx_d      : std_logic_vector(0 to 2);
  signal sclk           : std_logic;
  signal prst           : std_logic;
  signal pclk           : std_logic;
  signal tmds           : slv10_vector(0 to 2);
  signal status         : hdmi_rx_selectio_status_t;
  signal hdmi_tx_clk    : std_logic;
  signal hdmi_tx_d      : std_logic_vector(0 to 2);

begin

  led_o(0) <= not rst_100m   when sw_i(0) = '0' else status.align_s(0);
  led_o(1) <= status.lock    when sw_i(0) = '0' else status.align_s(1);
  led_o(2) <= status.band(0) when sw_i(0) = '0' else status.align_s(2);
  led_o(3) <= status.band(1) when sw_i(0) = '0' else status.align_p;

--  led(4) <= status.align_s(0);
--  led(5) <= status.align_s(1);
--  led(6) <= status.align_s(2);
--  led(7) <= status.align_p;

  --------------------------------------------------------------------------------
  -- clock and reset generation

  U_MMCM: component mmcm
    generic map (
      mul         => 8.0,
      div         => 1,
      num_outputs => 2,
      odiv0       => 10.0,
      odiv        => (5,10,10,10,10,10)
    )
    port map (
      rsti        => not btn_i(0),
      clki        => sysclk_i,
      rsto        => rst_a,
      clko(0)     => clk_100m,
      clko(1)     => clk_200m
    );

  process(rst_a,clk_200m)
  begin
    if rst_a = '1' then
      rst_200m_s(0 to 1) <= (others => '1');
      rst_200m <= '1';
    elsif rising_edge(clk_200m) then
      rst_200m_s(0 to 1) <= rst_a & rst_200m_s(0);
      rst_200m <= rst_200m_s(1);
    end if;
  end process;

  process(rst_200m,clk_100m)
  begin
    if rst_200m = '1' then
      rst_100m_s(0 to 1) <= (others => '1');
      rst_100m <= '1';
    elsif rising_edge(clk_100m) then
      rst_100m_s(0 to 1) <= (rst_200m or not idelayctrl_rdy) & rst_100m_s(0);
      rst_100m <= rst_100m_s(1);
    end if;
  end process;

  --------------------------------------------------------------------------------
  -- HDMI I/O

  U_HDMI_RX: component hdmi_rx_selectio
    generic map (
      fclk  => 100.0
    )
    port map (
      rst    => rst_100m,
      clk    => clk_100m,
      pclki  => hdmi_rx_clk,
      si     => hdmi_rx_d,
      sclko  => sclk,
      prsto  => prst,
      pclko  => pclk,
      po     => tmds,
      status => status
    );

  U_HDMI_TX: component hdmi_tx_selectio
    port map (
      sclki => sclk,
      prsti => prst,
      pclki => pclk,
      pi    => tmds,
      pclko => hdmi_tx_clk,
      so    => hdmi_tx_d
    );

  --------------------------------------------------------------------------------
  -- I/O primitives

  -- required to use I/O delay primitives
  U_IDELAYCTRL: idelayctrl
    port map (
      rst    => rst_200m, -- assumption!!! is asserted for >60ns
      refclk => clk_200m,
      rdy    => idelayctrl_rdy
    );

  -- HDMI input and output differential buffers

  U_IBUFDS: component ibufds
    port map (
      i  => hdmi_rx_clk_p_i,
      ib => hdmi_rx_clk_n_i,
      o  => hdmi_rx_clku
    );

  U_BUFG: component bufg
    port map (
      i  => hdmi_rx_clku,
      o  => hdmi_rx_clk
    );

  U_OBUFDS: component obufds
    port map (
      i  => hdmi_tx_clk,
      o  => hdmi_tx_clk_p_o,
      ob => hdmi_tx_clk_n_o
    );

  GEN_CH: for i in 0 to 2 generate

    U_IBUFDS: component ibufds
      port map (
        i  => hdmi_rx_p_i(i),
        ib => hdmi_rx_n_i(i),
        o  => hdmi_rx_d(i)
      );

    U_OBUFDS: component obufds
      port map (
        i  => hdmi_tx_d(i),
        o  => hdmi_tx_p_o(i),
        ob => hdmi_tx_n_o(i)
      );

  end generate GEN_CH;

end architecture synth;

