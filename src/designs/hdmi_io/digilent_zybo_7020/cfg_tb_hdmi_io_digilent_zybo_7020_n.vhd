-- for simulators without encrypted (secure) IP support

configuration cfg_oserdese2 of serialiser_10to1_selectio is
  for synth
    for U_SER_M: oserdese2
      use entity work.oserdese2(model);
    end for;
    for U_SER_S: oserdese2
      use entity work.oserdese2(model);
    end for;
  end for;
end configuration cfg_oserdese2;

configuration cfg_hdmi_io of hdmi_io is
  for synth
    for HDMI_TX_CLK: serialiser_10to1_selectio
      use configuration work.cfg_oserdese2;
    end for;
    for GEN_HDMI_TX_DATA
        for HDMI_TX_DATA: serialiser_10to1_selectio
          use configuration work.cfg_oserdese2;
        end for;
    end for;
  end for;
end configuration cfg_hdmi_io;

configuration cfg_hdmi_io_digilent_zybo_7020 of hdmi_io_digilent_zybo_7020 is
  for synth
    for MAIN: hdmi_io
      use configuration work.cfg_hdmi_io;
    end for;
  end for;
end configuration cfg_hdmi_io_digilent_zybo_7020;

configuration cfg_tb_hdmi_io_digilent_zybo_7020 of tb_hdmi_io_digilent_zybo_7020 is
  for sim
    for DUT: hdmi_io_digilent_zybo_7020
      use configuration work.cfg_hdmi_io_digilent_zybo_7020;
    end for;
  end for;
end configuration cfg_tb_hdmi_io_digilent_zybo_7020;

