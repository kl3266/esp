-- Copyright (c) 2011-2024 Columbia University, System Level Design Group
-- SPDX-License-Identifier: Apache-2.0

-------------------------------------------------------------------------------
-- FPGA Proxy for chip testing and DDR access
--
-- This module can only handle LLC requests and non-coherent DMA transactions.
-- Hence, the ESP cache hierarchy must be enabled when using this proxy.
-- In addition, the EDCL module in the I/O tile  won't be able to access memory.
-- To load programs and data into main memory, a second EDCL must be available
-- in the FPGA design that hosts the DDR controllers.
--
-- To improve link performance, the read/write bit is combined with the address.
-- Trasnsactions are assumed to be of an integer number of entire words (64 or
-- 32 bits depending on the global variable ARCH_BITS). The address is,
-- therefore, forced to be aligned with one word (LSBs are zeroed) and address
-- bit zero is used as write (not read) flag. The assumption on hsize is always
-- valid for LLC requests (full cache-line access) and DMA transactions.
-------------------------------------------------------------------------------

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

entity d2dtx_top is
  generic (
    TXCHANNELS        : integer                                               := 4;
    TILES             : integer                                               := 3;
    PLANES            : integer                                               := 6;
    FifoBypass        : std_ulogic                                            := '1'
          );
  port (
    clk               : in  std_ulogic;
    rstn              : in  std_ulogic;

    -- D2D Tx --> D2D Rx
    d2d_snd_data_out  : out coh_noc_flit_vector(TXCHANNELS-1 downto 0);
    d2d_snd_valid_out : out std_logic_vector(TXCHANNELS-1 downto 0);
    d2d_snd_void_out  : out std_logic_vector(TXCHANNELS-1 downto 0);
    d2d_clk_out       : out std_ulogic;
    d2d_snd_ready     : out std_logic_vector(TXCHANNELS-1 downto 0);

    -- D2D Rx --> D2D Tx
    d2d_clk_in        : in std_ulogic;
    d2d_credit_in     : in std_logic_vector(TX_CHANNELS-1 downto 0);
    
    -- NoC --> D2D
    noc_data_in       : in  coh_noc_flit_vector(TILES*PLANES-1 downto 0);
    noc_data_void_in  : in  std_logic_vector(TILES*PLANES-1 downto 0);
    -- D2D --> NoC
    noc_stop_out      : out std_logic_vector(TILES*PLANES-1 downto 0);
    );
end d2dtx_top;

architecture rtl of d2dtx_top is

  constant TxP            : integer := TILES*PLANES;
  constant W              : integer := TXCHANNELS;
  constant FifoBypassEnable : std_ulogic := FifoBypass;
  constant LSB            : integer := GLOB_BYTE_OFFSET_BITS;
  constant NOC_QUEUE_DEPTH: integer := 4;
  constant QUEUE_DEPTH    : integer := 8;
  constant CreditsWidth   : positive(ceil(log2(real(QUEUE_DEPTH+1))));

  -- Synchronized to clk
  -- Synchronous FIFO
  signal noc_snd_wrreq    : std_logic_vector(TxP-1 downto 0);
  signal noc_snd_full     : std_logic_vector(TxP-1 downto 0);
  signal noc_snd_data_out : coh_noc_flit_vector(TxP-1 downto 0);
  signal noc_snd_empty    : std_logic_vector(TxP-1 downto 0);
  signal fifo_head        : coh_noc_flit_vector(TxP-1 downto 0);
  signal rd_fifo_or       : std_logic_vector(TxP-1 downto 0);

  -- Asynchronous FIFO
  signal d2d_snd_wrreq    : std_logic_vector(TxP-1 downto 0);
  signal d2d_snd_data_in  : coh_noc_flit_vector(TxP-1 downto 0);
  signal d2d_snd_full     : std_logic_vector(TxP-1 downto 0);
  -- At Input
  signal in_unvalid_flit  : std_logic_vector(TxP-1 downto 0);
  signal in_valid_head  : std_logic_vector(TxP-1 downto 0);

  -- Synchronized to fpga_clk_in
  -- Asynchronous FIFO
  signal d2d_snd_rdreq    : std_logic_vector(W-1 downto 0);
  signal d2d_snd_data_out : coh_noc_flit_vector(W-1 downto 0);
  signal d2d_snd_empty    : std_logic_vector(W-1 downto 0);

  -- Sync FIFO - Async FIFO credits
  type credits_t is array (W-1 downto 0) of std_logic_vector(CreditsWidth-1 downto 0);
  signal noc_credits      : credits_t;
  signal channel_credits  : credits_t;
  signal credits          : integer range 0 to QUEUE_DEPTH;
  signal credit_out       : std_ulogic;
  signal credit_out_empty : std_ulogic;

  signal d2d_credit_in_int: std_logic_vector(W-1 downto 0); -- not used yet

  subtype routing_table_row_t is integer(0 to TxP-1);
--  subtype inv_routing_table_row_t is integer(0 to W-1);
  type routing_table_t is array (natural range <>) of routing_table_row_t;
--  type inv_routing_table_t is array (natural range <>) of inv_routing_table_row_t;
  signal routing_table    : routing_table_t(W-1 downto 0);
  signal routing_table_valid  : std_logic_vector(W-1 downto 0);
--  signal inv_routing_table  : inv_routing_table_t(TxP-1 downto 0);
--  signal inv_routing_table_valid  : std_logic_vector(TxP-1 downto 0);
  signal final_routing_table    : routing_table_t(W-1 downto 0);
  signal final_routing_table_valid  : std_logic_vector(W-1 downto 0);
--  signal final_inv_routing_table  : inv_routing_table_t(TxP-1 downto 0);
--  signal final_inv_routing_table_valid  : std_logic_vector(TxP-1 downto 0);

  signal last_flit        : coh_noc_flit_vector(W-1 downto 0);

  --destination table to prevent packets from same chip, tile, plane to occupy more than one channel?
--  type chip_coordinate is std_logic_vector(2 to 0);
--  type tile_coordinate is std_logic_vector(3 to 0);
--  type plane_number is std_logic_vector(2 to 0);
--  TYPE dest_type is RECORD
--    chip_x    : chip_coordinate;
--    chip_y    : chip_coordinate;
--    tile_x    : tile_coordinate;
--    tile_y    : tile_coordinate;
--    plane_num : plane_number;
--  END RECORD;
  
--  type dest_type_t is array (natural range<>) of dest_type;
--  signal dest_table : dest_type_t(0 to W-1);

  -- arbiter input and control
  signal request_input    : std_logic_vector(TxP-1 downto 0);
  signal request_channel  : std_logic_vector(W-1 downto 0);
  signal input_enable     : std_logic_vector(TxP-1 downto 0);
  signal channel_enable   : std_logic_vector(W-1 downto 0);
  signal grant_valid_input  : std_ulogic;
  signal grant_valid_channel: std_ulogic;

  -- arbiter output indices
  signal grant_index_i    : std_logic_vector(positive(ceil(log2(real(TxP))))-1 downto 0);
  signal grant_index_c    : std_logic_vector(positive(ceil(log2(real(W))))-1 downto 0);
  signal grant_index_input  : integer(0 to TxP-1);
  signal grant_index_channel  : integer(0 to W-1);

  -- channel output
  signal channel_reset    : std_logic_vector(W-1 downto 0);

  -- crossbar
  type op_ip_arr_t is array (W-1 downto 0) of std_logic_vector(TxP-1 downto 0);

  signal data_out_crossbar  : coh_noc_flit_vector(W-1 downto 0);
  signal rd_fifo            : op_ip_arr_t;
  signal out_unvalid_flit   : std_logic_vector(W-1 downto 0);
  signal no_backpressure    : std_logic_vector(W-1 downto 0);

begin  -- architecture rtl

  -----------------------------------------------------------------------------
  -- Drive fpga_clk_out (should be lenght-matched w/ credit and data)
  fpga_clk_out <= fpga_clk_in;
  grant_index_input <= to_integer(unsigned(grant_index_i));
  grant_index_channel <= to_integer(unsigned(grant_index_c));
  --rd_fifo <= (others => (others => '0'));

  gen_fifos : for g_i in 0 to TxP-1 generate
  begin
    NoC_FIFO_i: router_fifo
    generic map (
      BypassEnable => '1',
      g_data_width => coh_noc_flit_width,
      g_size       => QUEUE_DEPTH)
    port map (
      clk       => clk,
      rst       => rstn,
      rdreq     => rd_fifo_or(g_i),
      wrreq     => noc_snd_wrreq(g_i),
      data_in   => noc_data_in(g_i),
      empty     => noc_snd_empty(g_i),
      full      => noc_snd_full(g_i),
      data_out  => fifo_head(g_i)
    );

    in_unvalid_flit(g_i) <= (noc_snd_empty(g_i) and noc_data_void_in(g_i)) when FifoBypassEnable = '1' else noc_snd_empty(g_i);
    in_valid_head(g_i) <= fifo_head(g_i)(COH_NOC_FLIT_SIZE-1) and (not in_unvalid_flit(g_i));
    noc_snd_wrreq(g_i) <= not noc_data_void_in(g_i);
    noc_stop_out(g_i) <= noc_snd_full(g_i) when FifoBypassEnable = '1' else not (rd_fifo_or(g_i) and (not in_unvalid_flit(g_i)));
    request_input(g_i) <= input_enable(g_i) and in_valid_head(g_i);

    process (rd_fifo) is
    begin
      rd_fifo_or(g_i) <= '0';
      for i in 0 to W-1 loop
        rd_fifo_or(g_i) <= rd_fifo_or(g_i) or rd_fifo(i)(g_i);
      end loop;
    end process;

  end generate gen_fifos;

  gen_channels : for g_i in 0 to W-1 generate
  begin
    D2D_Channel_i: d2d_channel
    generic map (
      FifoBypassEnable  => FifoBypassEnable,
      g_data_width      => coh_noc_flit_width,
      g_size            => QUEUE_DEPTH)
    port map (
      clk               => clk,
      rstn              => rstn,
      sync_data_in      => data_out_crossbar(g_i),
      sync_wrreq        => ch_wrreq(g_i),
      sync_full         => ch_full(g_i),
      async_rdreq       => ch_rdreq(g_i),
      async_empty       => ch_empty(g_i),
      async_data_out    => d2d_snd_data_out(g_i),
      async_void_out    => d2d_snd_void_out(g_i),
      async_valid_out   => d2d_snd_valid_out(g_i) -- may not be necessary
      --need credit in/out
    );

    no_backpressure(g_i) <= not d2d_stop_out(g_i) when FifoBypassEnable = '1' else (noc_credits(g_i) /= (others => '0')); -- stop_in from async fifo
    forwarding_head(g_i) <= data_out_crossbar(g_i)(COH_NOC_FLIT_SIZE-1) and (not out_unvalid_flit(g_i)) and no_backpressure(g_i);
    forwarding_tail(g_i) <= data_out_crossbar(g_i)(COH_NOC_FLIT_SIZE-2) and (not out_unvalid_flit(g_i)) and no_backpressure(g_i);

    if FifoBypassEnable = '1' then
      process (rstn, clk) is
      begin
        if rstn = '0' then
          data_void_out(g_i) <= '1';
        else
          if rd_fifo(g_i)(final_routing_table(g_i)) = '0' then --rethink this if
            data_void_out(g_i) <= '1';
          elsif no_backpressure(g_i) = '1' then
            data_void_out(g_i) <= out_unvalid_flit(g_i);
          end if;
        end if;
      end process;
      noc_credits(g_i) <= (others => '0');
    else
      data_void_out(g_i) <= out_unvalid_flit(g_i) when rd_fifo(g_i)(final_routing_table(g_i)) = '1' else '1'; --rethink this when condition
      process (rstn, clk) is
      begin
        if rstn = '0' then
          noc_credits(g_i) <= std_logic_vector(to_unsigned(QUEUE_DEPTH, noc_credits(g_i)'length));
        elsif clk'event and clk = '1' then
          if data_void_out(g_i) <= '0' then
            noc_credits(g_i) <= std_logic_vector(unsigned(noc_credits(g_i)) - resize(unsigned(stop_in(g_i)), noc_credits(g_i)'length));
          else
            noc_credits(g_i) <= std_logic_vector(unsigned(noc_credits(g_i)) + resize(unsigned(not stop_in(g_i)), noc_credits(g_i)'length));
          end if;
        end if;
      end process;
    end if;
        
        
    process (rstn, clk) is
    begin
      if rstn = '0' then
        last_flit(g_i) <= (others => '0');
      elsif clk'event and clk = '1' then
        if FifoBypassEnable = '1' then
          if out_unvalid_flit(g_i) = '0' and rd_fifo(g_i)(final_routing_table(g_i)) = '1' then
            last_flit(g_i) <= data_out_crossbar(g_i);
          end if;
        else
          if data_void_out(g_i) = '0' then
            last_flit(g_i) <= data_out_crossbar(g_i);
          end if;
        end if;
      end if;
    end process;

    data_out(g_i) <= last_flit(g_i) when FifoBypassEnable = '1' else data_out_crossbar(g_i);

    -- Crossbar
    process (fifo_head, final_routing_table_valid, final_routing_table, no_backpressure, in_unvalid_flit) is
    begin
      data_out_crossbar(g_i) <= fifo_head(final_routing_table(g_i)) and (others => final_routing_table_valid(g_i)); -- routing table stuff comes 1 cycle later... make routing_table wire and reg so the change is registered in the same cycle?
      --rd_fifo(g_i)(final_routing_table(g_i)) <= no_backpressure(g_i) and final_routing_table_valid(g_i);  -- separated this from the crossbar to a separate process outside gen
      out_unvalid_flit(g_i) <= in_unvalid_flit(final_routing_table(g_i)) and final_routing_table_valid(g_i);
    end process;
  end generate gen_channels;

  process (no_backpressure, final_routing_table_valid) is
  begin
    rd_fifo <= (others => others => '0');
    for i 0 to W-1 loop
      rd_fifo(i)(final_routing_table(i)) <= no_backpressure(i) and final_routing_table_valid(i);
    end loop;
  end process;

  arbiter_input_i: d2d_arbiter
  generic map (
    CHANNELS => TxP
  )
  port map (
    clk                       => clk,
    rstn                      => rstn,
    request                   => request_input,
    grant_valid_other_arbiter => grant_valid_channel,
    grant_index               => grant_index_i,
    grant_valid               => grant_valid_input
  );

  arbiter_channel_i: d2d_arbiter
  generic map (
    CHANNELS => W
  )
  port map (
    clk                       => clk,
    rstn                      => rstn,
    request                   => request_channel,
    grant_valid_other_arbiter => grant_valid_input,
    grant_index               => grant_index_c,
    grant_valid               => grant_valid_channel
  );

  process (rstn, clk) is
  begin
    if rstn = '0' then
      routing_table <= (others => (others => '0'));
      routing_table_valid <= (others => '0');
--      inv_routing_table <= (others => (others => '0'));
--      inv_routing_table_valid <= (others => '0');
      input_enable <= (others => '1');
      channel_enable <= (others => '1');
    elsif clk'event and clk = '1' then
      if grant_valid_channel = '1' and grant_valid_input = '1' then
        routing_table(grant_index_channel) <= grant_index_input;
        routing_table_valid(grant_index_channel) <= '1';
--        inv_routing_table(grant_index_input) <= grant_index_channel;
--        inv_routing_table_valid(grant_index_input) <= '1';
        input_enable(grant_index_input) <= '0';
        channel_enable(grant_index_channel) <= '0';
      else
        for j in 0 to W-1 loop
          if channel_reset(j) = '1' then
            routing_table_valid(j) <= '0';
--            inv_routing_table_valid(routing_table(j)) <= '0';
            input_enable(routing_table(j)) <= '1';
            channel_enable(j) <= '1';
          end if;
        end loop;
      end if;
    end if;
  end process;

  -- routing table management (To accommodate same-cycle processing)
  process (grant_valid_input, grant_valid_channel, routing_table, 
    routing_table_valid 
--    inv_routing_table, inv_routing_table_valid
  ) is
  begin

    final_routing_table <= routing_table;
    final_routing_table_valid <= routing_table_valid;
--    final_inv_routing_table <= inv_routing_table;
--    final_inv_routing_table_valid <= inv_routing_table_valid;

    -- ensure same-cycle processing for the routing table
    if grant_valid_channel = '1' and grant_valid_input = '1' then
      final_routing_table(grant_index_channel) <= grant_index_input;
      final_routing_table_valid(grant_index_channel) <= '1';
--      final_inv_routing_table(grant_index_input) <= grant_index_channel;
--      final_inv_routing_table_valid(grant_index_input) <= '1';
    end if;
  end process;

--  process (rstn, clk) is
--  begin
--    if rstn = '0' then
--      input_enable <= (others => '1');
--      channel_enable <= (others => '1');
--      routing_table <= (others => (others => '0'));
--      routing_table_valid <= (others => '0');
--    elsif clk'event and clk = '1' then
--      if grant_valid_channel = '1' and grant_valid_input = '1' then
--        for i 0 to TxP-1 loop
--          if grant_input(i) = '1' then
--            input_enable(i) <= '0';
--          end if;
--        end loop;
--        for j 0 to W-1 loop
--          if grant_channel(j) = '1' then
--            channel_enable(j) <= '0';
--            routing_table(j) <= index of the input...?;
--            routing_table_valid(j) <= '1';
--            for i 0 to TxO-1 loop
--              if grant_input(i)
--          end if;
--        end loop;
--      else
--        for j in 0 to W-1 loop
--          if channel_reset(j) = '1' then
--            routing_table_valid(j) <= '0';
--            input_enable(routing_table(j)) <= '1';
--            channel_enable(j) <= '1';
--          end if;
--        end loop;
--      end if;
--    end if;
--  end process;

--  --g_i integration (scratchpad)
--  process (rstn, clk) is
--  begin
--    if rstn = '0' then
--      routing_table(g_i) <= 0;
--      routing_table_valid(g_i) <= '0';
--      channel_enable(g_i) <= '1';
--    elsif clk'event and clk = '1' then
--      if grant_valid_channel = '1' and grant_valid_input = '1' then
--        routing_table(grant_index_channel) <= grant_index_input;
--        routing_table_valid(grant_index_channel) <= '1';
--        channel_enable(grant_index_channel) <= '0';
--      else
--        if channel_reset(g_i) = '1' then
--          routing_table_valid(g_i) <= '0';
--          channel_enable(g_i) <= '1';
--        end if;
--      end if;
--    end if;
--  end process;
--  -- output g_i integration seems fine.
--
--  process (rstn, clk) is
--  begin
--    if rstn = '0' then
--      input_enable(g_i) <= (others => '1');
--    elsif clk'event and clk = '1' then
--      if grant_valid_channel = '1' and grant_valid_input = '1' then
--        input_enable(grant_index_input) <= '0';
--      else
--        for j in 0 to W-1 loop
--          if channel_reset(inv_routing_table(g_i)) = '1' and inv_routing_table_valid(g_i) = '1' then
--            input_enable(g_i) <= '1';
--          end if;
--        end loop;
--      end if;
--    end if;
--  end process;
----This requires inv_routing_table... combined loop better?

end architecture rtl;
