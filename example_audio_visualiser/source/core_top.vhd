--
-- WWW.FPGAArcade.COM
--
-- REPLAY Retro Gaming Platform
-- No Emulation No Compromise
--
-- All rights reserved
-- Mike Johnson
-- Modified 2017 Gary Preston.
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS CODE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- You are responsible for any legal issues arising from your use of this code.
--
-- The original version of this file can be found at: www.FPGAArcade.com
--
-- Email support@fpgaarcade.com
--
library ieee;
  use ieee.std_logic_1164.all;
  use ieee.std_logic_unsigned.all;
  use ieee.numeric_std.all;

  use work.Replay_Pack.all;
  use work.Replay_Lib_Wrap_Pack.all;
  use work.Replay_VideoTiming_Pack.all;

library UNISIM;
  use UNISIM.Vcomponents.all;

entity Core_Top is
  port (
  ------------------------------------------------------
  -- To Lib
  ------------------------------------------------------

  -- Clocks
  o_ctrl                : out   r_Ctrl_fm_core;
  i_ctrl                : in    r_Ctrl_to_core;
  -- Config
  o_cfg                 : out   r_Cfg_fm_core;
  i_cfg                 : in    r_Cfg_to_core;
  -- Keyboard, Mouse and Joystick
  o_kb_ms_joy           : out   r_KbMsJoy_fm_core;
  i_kb_ms_joy           : in    r_KbMsJoy_to_core;
  -- DDR memory
  o_ddr                 : out   r_DDR_fm_core;
  i_ddr                 : in    r_DDR_to_core;
  -- File/Mem IO
  o_io                  : out   r_IO_fm_core;
  i_io                  : in    r_IO_to_core;
  -- Audio/Video
  o_av                  : out   r_AV_fm_core;
  i_av                  : in    r_AV_to_core;

  ------------------------------------------------------
  -- Other IO
  ------------------------------------------------------
  i_rs232_rxd           : in    bit1;
  o_rs232_txd           : out   bit1;
  i_rs232_cts           : in    bit1;
  o_rs232_rts           : out   bit1;

  b_2v5_io_1            : inout bit1;
  b_2v5_io_0            : inout bit1;

  o_clk_68k             : out   bit1;
  b_clk_aux             : out   bit1;

  b_io                  : inout word(54 downto 0);
  b_aux_io              : inout word(39 downto 0);
  i_aux_ip              : in    word(22 downto 0);

  o_disk_led            : out   bit1;
  o_pwr_led             : out   bit1
  );
end;

architecture RTL of Core_Top is

  signal clk_aud                : bit1;
  signal ena_aud                : bit1;
  signal rst_aud                : bit1;
  signal aud_cnt                : word(1 downto 0);

  signal pwr_led                : bit1;
  signal disk_led               : bit1;

begin
  o_ctrl.ena_vid <= '1';

  --
  -- Audio clock mapping
  --
  clk_aud <= i_ctrl.clk_aux; -- soft assign (could be sys clock)
  rst_aud <= i_ctrl.rst_aux;

  p_audio_ena_cnt : process
  begin
    wait until rising_edge(clk_aud);
    aud_cnt <= aud_cnt + "1";
    ena_aud <= '0';
    if (aud_cnt = "10") then
      ena_aud <= '1'; -- 1 in 4 to get 48K
    end if;
  end process;

  o_ctrl.clk_aud <= clk_aud;
  o_ctrl.ena_aud <= ena_aud;
  o_ctrl.rst_aud <= rst_aud;

  --
  -- The Core
  --
  u_Core : entity work.Example_Visualiser_Top
  port map (
  --  
  i_clk_sys             => i_ctrl.clk_sys,
  i_ena_sys             => i_ctrl.ena_sys,
  i_cph_sys             => i_ctrl.cph_sys,
  i_rst_sys             => i_ctrl.rst_sys,

  i_clk_ram             => i_ctrl.clk_ram,
  i_rst_ram             => i_ctrl.rst_ram,

  i_clk_aud             => clk_aud,
  i_ena_aud             => ena_aud,
  i_rst_aud             => rst_aud,

  i_clk_vid             => i_ctrl.clk_vid,
  i_rst_vid             => i_ctrl.rst_vid,

  --
  o_cfg_status          => o_cfg.cfg_status,
  i_cfg_static          => i_cfg.cfg_static,
  i_cfg_dynamic         => i_cfg.cfg_dynamic,

  i_tick_1us            => i_ctrl.tick_1us,
  i_tick_100us          => i_ctrl.tick_100us,
  i_halt                => i_ctrl.halt,
  i_dram_ref_panic      => i_ctrl.dram_ref_panic,
  o_rst_soft            => o_ctrl.rst_soft,

  --
  i_joy_a_l             => i_kb_ms_joy.joy_a_l,
  i_joy_b_l             => i_kb_ms_joy.joy_b_l,

  --
  o_kb_ps2_leds         => o_kb_ms_joy.kb_ps2_leds,
  i_kb_ps2_we           => i_kb_ms_joy.kb_ps2_we,
  i_kb_ps2_data         => i_kb_ms_joy.kb_ps2_data,
  i_kb_inhibit          => i_kb_ms_joy.kb_inhibit,

  --
  i_ms_we               => i_kb_ms_joy.ms_we,
  i_ms_posx             => i_kb_ms_joy.ms_posx,
  i_ms_posy             => i_kb_ms_joy.ms_posy,
  i_ms_posz             => i_kb_ms_joy.ms_posz,
  i_ms_butn             => i_kb_ms_joy.ms_butn,

  o_ddr_hp_fm_core      => o_ddr.ddr_hp_fm_core,
  i_ddr_hp_to_core      => i_ddr.ddr_hp_to_core,

  o_ddr_vp_fm_core      => o_ddr.ddr_vp_fm_core,
  i_ddr_vp_to_core      => i_ddr.ddr_vp_to_core,

  --
  i_fcha_cfg            => i_io.fcha_cfg,
  i_fcha_to_core        => i_io.fcha_to_core,
  o_fcha_fm_core        => o_io.fcha_fm_core,

  i_fchb_cfg            => i_io.fchb_cfg,
  i_fchb_to_core        => i_io.fchb_to_core,
  o_fchb_fm_core        => o_io.fchb_fm_core,

  --
  i_memio_to_core       => i_io.memio_to_core,
  o_memio_fm_core       => o_io.memio_fm_core,

  --
  o_vid_rgb             => o_av.vid_rgb,
  o_vid_sync            => o_av.vid_sync,

  --
  o_audio_l             => o_av.audio_l,
  o_audio_r             => o_av.audio_r,
  i_audio_taken         => i_av.audio_taken,

  ------------------------------------------------------
  -- Other IO
  ------------------------------------------------------
  i_rs232_rxd           => i_rs232_rxd,
  o_rs232_txd           => o_rs232_txd,
  i_rs232_cts           => i_rs232_cts,
  o_rs232_rts           => o_rs232_rts,
  --
  o_disk_led            => disk_led,
  o_pwr_led             => pwr_led
  );

  o_kb_ms_joy.ms_load <= '0';
  o_kb_ms_joy.ms_posx <= (others => '0');
  o_kb_ms_joy.ms_posy <= (others => '0');

   -- IO
  b_2v5_io_1  <= 'Z';
  b_2v5_io_0  <= 'Z';
  o_clk_68k   <= 'Z';
  b_clk_aux   <= 'Z';
  o_disk_led  <= disk_led;
  o_pwr_led   <= pwr_led;

  b_io(54 downto 0)     <= (others => 'Z');
  b_aux_io(39 downto 0) <= (others => 'Z');

end RTL;
