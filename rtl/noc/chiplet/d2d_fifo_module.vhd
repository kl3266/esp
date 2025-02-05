library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.esp_global.all;
use work.amba.all;
use work.stdlib.all;
use work.sld_devices.all;
use work.devices.all;
use work.gencomp.all;
use work.leon3.all;
use work.uart.all;
use work.misc.all;
use work.net.all;
-- pragma translate_off
use work.sim.all;
library unisim;
use unisim.all;
-- pragma translate_on
use work.monitor_pkg.all;
use work.sldacc.all;
use work.nocpackage.all;
use work.tile.all;
use work.coretypes.all;
use work.grlib_config.all;
use work.socmap.all;

entity d2d_fifo_module is
  generic (
    TILES   : integer := 4;
    PLANES  : integer := 6
  );
  port (
    clk     : in std_ulogic;
    rstn    : in std_ulogic;

    noc_snd_wrreq : in std_logic_vector(0 to TILES*PLANES-1);
    noc_snd_data_in : in coh_noc_flit_vector(0 to TILES*PLANES-1);
    noc_snd_full  : out std_logic_vector(0 to TILES*PLANES-1);
    noc_snd_rdreq : in std_logic_vector(0 to TILES*PLANES-1);
    noc_snd_data_out  : out coh_noc_flit_vector(0 to TILES*PLANES-1);
    noc_snd_empty : out std_logic_vector(0 to TILES*PLANES-1);
    noc_data_void_in  : in std_logic_vector(0 to TILES*PLANES-1);
    noc_stop_out  : out std_logic_vector(0 to TILES*PLANES-1);
    );
end d2d_fifo_module;

architecture rtl of d2d_fifo_module is
  signal fifo_head : coh_noc_flit_vector(0 to TILES*PLANES-1);
  signal in_unvalid_flit, in_valid_head : std_logic_vector(0 to TILES*PLANES-1);

