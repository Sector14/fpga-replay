--
-- WWW.FPGAArcade.COM
--
-- REPLAY Retro Gaming Platform
-- No Emulation No Compromise
--
-- All rights reserved
-- Mike Johnson
--
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
  use work.Replay_VideoTiming_Pack.all;

library UNISIM;
  use UNISIM.Vcomponents.all;

entity Example_Visualiser_Top is
  port (    
  ------------------------------------------------------
  -- To Lib
  ------------------------------------------------------

  -- System clock, enable and reset, generated from Clk A
  i_clk_sys             : in  bit1;
  i_ena_sys             : in  bit1;
  i_cph_sys             : in  word( 3 downto 0); -- four phased enables. (3) = ena_sys
  i_rst_sys             : in  bit1;

  i_clk_ram             : in  bit1;
  i_rst_ram             : in  bit1;

  -- Audio Clocking
  i_clk_aud             : in    bit1;
  i_ena_aud             : in    bit1;
  i_rst_aud             : in    bit1;
  
  -- Video clock, generated from Clk C. Used for all OSD Video/PHY data path.
  i_clk_vid             : in  bit1;
  i_rst_vid             : in  bit1;
  --
  -- Config/Control
  o_cfg_status          : out word(15 downto 0); -- status feedback to ARM
  i_cfg_static          : in  word(31 downto 0);
  i_cfg_dynamic         : in  word(31 downto 0);

  i_tick_1us            : in  bit1; -- on clk_sys with ena
  i_tick_100us          : in  bit1; -- on clk_sys with ena
  i_halt                : in  bit1;
  i_dram_ref_panic      : in  bit1;
  o_rst_soft            : out bit1 := '0';

  -- Joystick
  i_joy_a_l             : in  word( 5 downto 0);
  i_joy_b_l             : in  word( 5 downto 0);

  -- Keyboard
  o_kb_ps2_leds         : out word( 2 downto 0);
  i_kb_ps2_we           : in  bit1;
  i_kb_ps2_data         : in  word( 7 downto 0);
  i_kb_inhibit          : in  bit1; -- OSD active

  -- Mouse
  i_ms_we               : in  bit1;
  i_ms_posx             : in  word(11 downto 0);
  i_ms_posy             : in  word(11 downto 0);
  i_ms_posz             : in  word( 7 downto 0);
  i_ms_butn             : in  word( 2 downto 0);

  -- Mem interface high prio
  o_ddr_hp_fm_core      : out r_DDR_hp_fm_core; -- connect to z_DDR_hp_fm_core if not used
  i_ddr_hp_to_core      : in  r_DDR_hp_to_core;
  -- video port
  o_ddr_vp_fm_core      : out r_DDR_vp_fm_core; -- connect to z_DDR_vp_fm_core if not used
  i_ddr_vp_to_core      : in  r_DDR_vp_to_core;

  -- Fileio A
  i_fcha_cfg            : in  r_Cfg_fileio;
  i_fcha_to_core        : in  r_Fileio_to_core;
  o_fcha_fm_core        : out r_Fileio_fm_core;  -- connect to z_Fileio_fm_core if not used

  -- Fileio B
  i_fchb_cfg            : in  r_Cfg_fileio;
  i_fchb_to_core        : in  r_Fileio_to_core;
  o_fchb_fm_core        : out r_Fileio_fm_core;  -- connect to z_Fileio_fm_core if not used

  -- Fileio Mem
  i_memio_to_core       : in  r_Memio_to_core;
  o_memio_fm_core       : out r_Memio_fm_core;   -- connect to z_Memio_fm_core if not used

  -- Video (clk_vid)
  o_vid_rgb             : out word(23 downto 0);
  o_vid_sync            : out r_Vidsync;

  -- Audio (clk_aud)
  o_audio_l             : out word(23 downto 0); -- left  sample
  o_audio_r             : out word(23 downto 0); -- right sample
  i_audio_taken         : in  bit1;  -- sample ack

  ------------------------------------------------------
  -- Other IO
  ------------------------------------------------------
  i_rs232_rxd           : in  bit1;
  o_rs232_txd           : out bit1;
  i_rs232_cts           : in  bit1;
  o_rs232_rts           : out bit1;
  --
  o_disk_led            : out bit1;
  o_pwr_led             : out bit1  -- note these are active high outputs
  );
end;

architecture RTL of Example_Visualiser_Top is
  -- enable chipscope support?
  constant fileio_cs_enable : boolean := false;  

  signal led                    : bit1;
  signal tick_pre1              : bit1;
  signal tick                   : bit1;

  --
  -- fileio
  --
  signal fileio_addr            : word(31 downto 0);
  signal fileio_size            : word(15 downto 0);
  signal fileio_src_size        : word(31 downto 0);

  signal fileio_req             : bit1;
  signal fileio_ack_req         : bit1;
  signal fileio_ack_trans       : bit1;
  signal fileio_trans_err       : word( 2 downto 0);

  signal fifoio_tx_flush        : bit1;
  signal fileio_data            : word(17 downto 0);
  signal fileio_taken           : bit1;
  signal fileio_valid           : bit1;
  signal fileio_rx_level        : word(10 downto 0);
  signal fileio_rx_overfl       : bit1;

  type t_fileio_req_state is (S_IDLE, S_WAIT);
  signal fileio_req_state       : t_fileio_req_state;

  signal fileio_sample_cnt      : word( 1 downto 0);

  signal sample_audio_l         : word(23 downto 0);
  signal sample_audio_r         : word(23 downto 0);
  signal audio_taken_sync          : bit1;
  signal sys_audio_taken_sync      : bit1;
  signal sys_audio_taken_sync_old  : bit1;

  --
  -- Video
  --
  signal vid_enable             : bit1 := '1';

  signal dig_de                 : bit1;               -- active video
  signal dig_sof                : bit1;               -- start of frame
  signal dig_ha                 : bit1;               -- horiz active
  signal hpix                   : word(13 downto 0);
  signal vpix                   : word(13 downto 0);

  --
  -- Double buffered L/R audio samples
  --
  subtype t_sample_addr is word( 9 downto 0);
  subtype t_sample_data is word(17 downto 0);

  type t_addr_buffers is array (1 downto 0) of t_sample_addr;
  type t_data_buffers is array (1 downto 0) of t_sample_data;

  signal memio_samplebuffer_0 : r_Memio_fm_core;
  signal memio_samplebuffer_1 : r_Memio_fm_core;

  -- Active read/write buffer
  signal sample_read_buffer  : integer range 0 to 1;
  signal sample_write_buffer : integer range 0 to 1;

  signal sample_we_buffer0   : bit1;
  signal sample_we_buffer1   : bit1;

  signal sample_addr       : t_addr_buffers;
  signal sample_in         : t_data_buffers;
  signal sample_out        : t_data_buffers;

  -- ASync FIFO
  -- Domain cross for audio sample to BRAM double buffers
  signal bram_aud_fifo_w_data : word(15 downto 0);
  signal bram_aud_fifo_we     : bit1;
  signal bram_aud_fifo_hfull  : bit1;
  signal bram_aud_fifo_r_data : word(15 downto 0);
  signal bram_aud_fifo_re     : bit1;
  signal bram_aud_fifo_empty  : bit1;

begin

  o_cfg_status(15 downto  0) <= (others => '0');

  o_rst_soft            <= '0';
  o_kb_ps2_leds         <= "000";

  o_ddr_hp_fm_core      <= z_DDR_hp_fm_core;
  o_ddr_vp_fm_core      <= z_DDR_vp_fm_core;
  
  o_fchb_fm_core        <= z_Fileio_fm_core;

  o_rs232_txd           <= '0';
  o_rs232_rts           <= '0';

  -- ====================================================================
  -- Video Sync
  -- ====================================================================
  
  u_VideoTiming : entity work.Replay_VideoTiming
  generic map (
    g_enabledynamic       => '0',
    g_param               => c_Vidparam_720x480p_60
    )
  port map (
    i_clk                 => i_clk_vid,
    i_ena                 => vid_enable,
    i_rst                 => i_rst_vid,
    --
    i_param               => c_Vidparam_720x480p_60,
    i_sof                 => '0',
    i_f2_flip             => '0',
    --
    o_hactive             => open,
    o_hrep                => open,
    o_vactive             => open,
    --
    o_dig_hs              => o_vid_sync.dig_hs,
    o_dig_vs              => o_vid_sync.dig_vs,
    o_dig_de              => dig_de,
    o_dig_ha              => dig_ha,
    o_dig_va              => open,
    o_dig_sof             => dig_sof,
    o_dig_sol             => open,
    o_ana_hs              => o_vid_sync.ana_hs,
    o_ana_vs              => o_vid_sync.ana_vs,
    o_ana_de              => o_vid_sync.ana_de,
    --
    o_hpix                => hpix,
    o_vpix                => vpix,
    --
    o_f2                  => open,
    o_voddline            => open,
    o_stdprog             => open
  );
  o_vid_sync.dig_de <= dig_de;
  
  -- ASync FIFO for sample to cross from aud to vid domain before writing to RAM
  u_async_fifo : entity work.FIFO_Async_D16
    generic map (
      g_width => 16
    )
    port map (
      i_w_data    => bram_aud_fifo_w_data,
      i_w_ena     => bram_aud_fifo_we,
      o_w_full    => open,
      o_w_hfull   => bram_aud_fifo_hfull,
      --
      o_r_data    => bram_aud_fifo_r_data,
      i_r_ena     => bram_aud_fifo_re,
      o_r_empty   => bram_aud_fifo_empty,
      --
      -- TODO: Reconsider reset.
      i_rst       => i_rst_ram,
      --
      i_clk_w     => i_clk_aud,
      i_clk_r     => i_clk_vid
    );

  -- Buffer for snapshot of up to 1024 16bit (8b left, 8b right) audio samples
  u_SampleBuffer0 : entity work.RAM_D1024_W18
    generic map (
      g_addr  => x"00000000", -- 0x000 to 0x3FF
      g_mask  => x"000F0000"
      )
    port map (
      -- ARM interface
      i_memio_to_core             => i_memio_to_core,
      --
      i_memio_fm_core             => z_Memio_fm_core,
      o_memio_fm_core             => memio_samplebuffer_0,
      --
      i_clk_sys                   => i_clk_sys,
      i_ena_sys                   => i_ena_sys,
      
      --
      i_addr                      => sample_addr(0),
      i_data                      => sample_in(0),
      i_wen                       => sample_we_buffer0,
      o_data                      => sample_out(0),
      --
      i_ena                       => vid_enable,
      i_clk                       => i_clk_vid
      );
  u_SampleBuffer1 : entity work.RAM_D1024_W18 
    generic map ( 
      g_addr  => x"00010000", -- 0x000 to 0x3FF
      g_mask  => x"000F0000"
      )
    port map (
      -- ARM interface
      i_memio_to_core             => i_memio_to_core,
      --
      i_memio_fm_core             => memio_samplebuffer_0,
      o_memio_fm_core             => memio_samplebuffer_1,
      --
      i_clk_sys                   => i_clk_sys,
      i_ena_sys                   => i_ena_sys,
      
      --
      i_addr                      => sample_addr(1),
      i_data                      => sample_in(1),
      i_wen                       => sample_we_buffer1,
      o_data                      => sample_out(1),
      --
      i_ena                       => vid_enable,
      i_clk                       => i_clk_vid
      );
  o_memio_fm_core <= memio_samplebuffer_1;
  
  -- write buffer toggling
  sample_we_buffer0 <= '0' when sample_read_buffer = 0 else '1';
  sample_we_buffer1 <= '0' when sample_read_buffer = 1 else '1';

  -- _re needs to be valid before clk vid to enable sample to be taken.
  -- sample should be read out next clock (or lost)
  p_fifo_rd : process(bram_aud_fifo_empty, vid_enable)
  begin
    bram_aud_fifo_re <= (not bram_aud_fifo_empty) and vid_enable;
  end process;

  p_update_vid : process(i_clk_vid, i_rst_vid)
    variable sample_w_addr : t_sample_addr := (others => '0');
    variable frame_delay : integer := 0;
  begin
    if (i_rst_vid = '1') then
      -- Index 1 is the default for write buffer
      sample_in(1) <= (others => '0');
      sample_addr(1) <= (others => '0');
    elsif rising_edge(i_clk_vid) then
      sample_addr(sample_read_buffer) <= (others => 'Z');
      sample_in(sample_read_buffer) <= (others => 'Z');

      if (vid_enable = '1') then
        -- Transfer from FIFO to BRAM
        if (bram_aud_fifo_empty = '0') then

          sample_addr(sample_write_buffer) <= sample_w_addr;
          sample_in(sample_write_buffer) <= "00" & bram_aud_fifo_r_data;
          
          -- p_vid_out only needs 720 samples
          if (frame_delay > 2 and unsigned(sample_w_addr) < 720) then
            sample_w_addr := sample_w_addr + '1';
          end if;

        end if;
                  
        if (dig_sof = '1') then
          -- avoid filling new buffer for a number of frames to reduce the playback fps
          frame_delay := frame_delay + 1;
          -- buffer flip will only occur before new frame if write buffer was full
          -- frame_delay 10 gives 6fps instead of 60fps
          if (frame_delay > 2 and unsigned(sample_addr(sample_write_buffer)) = 720) then
            sample_w_addr := (others => '0');
            frame_delay := 0;
          end if;
        end if;

      end if;

    end if;
  end process;
  
  p_vid_out : process(i_clk_vid, i_rst_vid)
    variable sample_l_val : word(7 downto 0) := (others => '0');
    variable sample_r_val : word(7 downto 0) := (others => '0');
  begin
    if (i_rst_vid = '1') then
      sample_read_buffer <= 0;
      sample_write_buffer <= 1;
      sample_in(0) <= (others => '0');
      sample_addr(0) <= (others => '0');
    elsif rising_edge(i_clk_vid) then
    
      sample_addr(sample_write_buffer) <= (others => 'Z');
      sample_in(sample_write_buffer) <= (others => 'Z');

      if (vid_enable = '1') then

        o_vid_rgb <= (others => '0');

        if (dig_de = '1') then
          -- Using 256 range to avoid scaling the 8 bit sample with left channel at
          -- 0..255 Y and right at 224..479 Y. Slight visual overlap is acceptable :)
        
          -- default zero position is value of 128
          sample_l_val := x"80"; 
          sample_r_val := X"80";

          if (i_fcha_cfg.inserted(0) = '1') then
            -- Sample is -128..127 range. Bring to 0..255 for display
            sample_l_val := sample_out(sample_read_buffer)(15 downto 8) + 128;
            sample_r_val := sample_out(sample_read_buffer)( 7 downto 0) + 128;
          end if;
          
          if (sample_l_val = vpix) then
            o_vid_rgb <= x"00FFFF";
          end if;

          -- -225 to offset right channel to bottom of 480p screen
          if (sample_r_val = vpix - 225) then
            o_vid_rgb <= x"FFFF00";
          end if;

          sample_addr(sample_read_buffer) <= sample_addr(sample_read_buffer) + '1';
        end if;

        -- Prepare for start of next line (dig_sol would be a clock too late to use)
        if (dig_ha = '0') then
          sample_addr(sample_read_buffer) <= (others => '0');
        end if;

        -- Flip banks before new frame only if a full buffer available
        if (dig_sof = '1' and unsigned(sample_addr(sample_write_buffer)) = 720) then
          sample_read_buffer <= sample_read_buffer + 1;          
          sample_write_buffer <= sample_write_buffer + 1;

          sample_addr(sample_read_buffer) <= (others => '0');          
        end if;

      end if;
    end if;
  end process;

  -- ====================================================================
  -- Audio File I/O
  -- ====================================================================
  
  u_FileIO_FCh : entity work.Replay_FileIO_FCh_Generic
  port map (
    -- clocks
    i_clk                 => i_clk_sys,
    i_ena                 => i_ena_sys,
    i_rst                 => i_rst_sys,

    -- FileIO / Syscon interface
    i_fch_to_core         => i_fcha_to_core,
    o_fch_fm_core         => o_fcha_fm_core,

    -- to user space
    i_req                 => fileio_req,
    o_ack_req             => fileio_ack_req,
    o_ack_trans           => fileio_ack_trans, -- transfer done
    o_trans_err           => fileio_trans_err, -- aborted, truncated, seek error

    -- below latched on ack
    i_dir                 => '0', -- read only
    i_chan                => "00",
    i_addr                => fileio_addr,
    i_size                => fileio_size,
    o_size0               => fileio_src_size,

    -- Reading
    i_fifo_to_core_flush  => '0',
    o_fifo_to_core_data   => fileio_data,
    i_fifo_to_core_taken  => fileio_taken,
    o_fifo_to_core_valid  => fileio_valid,
    o_fifo_to_core_level  => fileio_rx_level,
    o_fifo_to_core_overfl => fileio_rx_overfl,

    -- Writing
    i_fifo_fm_core_flush  => '0',
    i_fifo_fm_core_data   => (others => '0'),
    i_fifo_fm_core_we     => '0',
    o_fifo_fm_core_level  => open
  );
  
  -- request size is 512 16 bits words
  fileio_size <= x"0400";

  -- Enable/pause data request via the Generic FileIO entity
  p_fileio_req : process(i_clk_sys, i_rst_sys)
  begin   
    if (i_rst_sys = '1') then
      fileio_addr <= (others => '0');
      fileio_req  <= '0';
      fileio_req_state <= S_IDLE;
    elsif rising_edge(i_clk_sys) then
      if (i_ena_sys = '1') then
        fileio_req  <= '0';

        if (i_fcha_cfg.inserted(0) = '0') then
          fileio_addr <= (others => '0');
          fileio_req_state <= S_IDLE;
        else
          case fileio_req_state is
            when S_IDLE =>
              if (fileio_rx_level(9) = '0') then -- <hf
                fileio_req  <= '1'; -- note, request only sent when ack received
              end if;
              if (fileio_ack_req = '1') then
                fileio_req_state <= S_WAIT;
              end if;

            when S_WAIT =>
              if (fileio_ack_trans = '1') then
                if (red_or(fileio_trans_err) = '1') then
                  fileio_addr <= (others => '0');
                else
                  fileio_addr <= fileio_addr + fileio_size;
                end if;
                fileio_req_state <= S_IDLE;
              end if;

            when others => null;
          end case;
        end if;
      end if;
    end if;
  end process;

  -- Transfer available sample data from FIFO out to audio subsystem  
  p_fileio_reader : process(i_clk_sys, i_rst_sys)
  begin
    if (i_rst_sys = '1') then
      fileio_sample_cnt <= "00";
      fileio_taken      <= '0';
      sample_audio_l   <= (others => '0');
      sample_audio_r   <= (others => '0');
    elsif rising_edge(i_clk_sys) then

      if (i_ena_sys = '1') then
        
        sys_audio_taken_sync_old <= sys_audio_taken_sync;
        sys_audio_taken_sync <= audio_taken_sync;
        
        fileio_taken <= '0';
        case fileio_sample_cnt is
          when "00" =>
            if (fileio_valid = '1') then
              fileio_taken <= '1';
              sample_audio_l   <= fileio_data( 7 downto 0) & fileio_data(15 downto 8) & x"00";
              fileio_sample_cnt <= "01";
            end if;
          when "01" => -- wait for taken to update valid
              fileio_sample_cnt <= "10";

          when "10" =>
            if (fileio_valid = '1') then
              fileio_taken <= '1';
              sample_audio_r   <= fileio_data( 7 downto 0) & fileio_data(15 downto 8) & x"00";
              fileio_sample_cnt <= "11";
            end if;
          when "11" => -- ready
            if (sys_audio_taken_sync /= sys_audio_taken_sync_old) then              
              
              fileio_sample_cnt <= "00";
            end if;
          when others => null;
        end case;
      end if;
    
    end if;

  end process;

  p_audio_out : process(i_clk_aud)
  begin    

    if rising_edge(i_clk_aud) then

      bram_aud_fifo_we <= '0';

      if (i_ena_aud = '1') then
        o_audio_l <= sample_audio_l;
        o_audio_r <= sample_audio_r;
        
        -- Audio will be taken @ 48kHz using 49.152MHz aud clock with 1:4 enable.
        if (i_audio_taken = '1') then
          -- Stretch taken pulse to sync with reader process
          audio_taken_sync <= not audio_taken_sync;

          if (i_fcha_cfg.inserted(0) = '1') then
            -- Scale 16 to 8bit per left and right
            bram_aud_fifo_w_data <= sample_audio_l(23 downto 16) & sample_audio_r(23 downto 16);
            -- TODO: Room in buffer?
            bram_aud_fifo_we <= '1';          
          end if;
        end if;
      end if;

    end if;
  end process;

  -- ====================================================================
  -- Activity LEDs
  -- ====================================================================

  b_tick : block
  signal precounter1 : word(15 downto 0);
  signal precounter2 : word(11 downto 0);
  begin

    p_count : process(i_clk_sys, i_rst_sys)
    begin
      if (i_rst_sys = '1') then
      precounter1 <= (others => '0');
      precounter2 <= (others => '0');
      tick_pre1   <= '0';
      tick        <= '0';
      elsif rising_edge(i_clk_sys) then

      if (i_ena_sys = '1') then
        precounter1 <= precounter1 - "1";

        tick_pre1 <= '0';
        if (precounter1 = x"0000") then
        tick_pre1 <= '1';
        end if;
        -- synopsys translate_off
        tick_pre1 <= '1';
        -- synopsys translate_on

        tick <= '0';
        if (tick_pre1 = '1') then
        if (precounter2 = x"000") then
          precounter2 <= x"19B";
          tick <= '1';
        else
          precounter2 <= precounter2 - "1";
        end if;
        end if;
      end if;
      end if;
    end process;
  end block;

  p_flash : process(i_clk_sys, i_rst_sys)
  begin
    if (i_rst_sys = '1') then
      led <= '0';
    elsif rising_edge(i_clk_sys) then
      if (i_ena_sys = '1') then
      if (tick = '1') then
        led  <= not led;
      end if;
      end if;
    end if;
  end process;

  o_disk_led        <=     led;
  o_pwr_led         <= not led;


  -- ====================================================================
  -- chipscope
  -- ====================================================================

  cs_debug : block
    component icon
      PORT (
        CONTROL0 : INOUT STD_LOGIC_VECTOR(35 DOWNTO 0)
        );
      end component;

    component ila_1024_63
      PORT (
        CONTROL : INOUT STD_LOGIC_VECTOR(35 DOWNTO 0);
        CLK : IN STD_LOGIC;
        TRIG0 : IN STD_LOGIC_VECTOR(62 DOWNTO 0)
        );
    end component;

    signal cs_clk  : bit1;
    signal cs_ctrl : word(35 downto 0);
    signal cs_trig : word(62 downto 0);

  begin -- cs_debug

    fileio_cs : if fileio_cs_enable=true generate

      cs_icon : icon
      port map (
        CONTROL0 => cs_ctrl
        );

      cs_ila : ila_1024_63
      port map (
        CONTROL => cs_ctrl,
        CLK     => cs_clk,
        TRIG0   => cs_trig
        );

      -- cs_clk  <= i_clk_sys;
      cs_clk  <= i_clk_aud;
      cs_trig(62) <= i_ena_sys;
      cs_trig(61) <= i_ena_aud;
      cs_trig(60) <= sys_audio_taken_sync;
      cs_trig(59) <= sys_audio_taken_sync_old;
      cs_trig(58) <= i_audio_taken;
      cs_trig(57) <= audio_taken_sync;
      cs_trig(56) <= fileio_sample_cnt(0) and fileio_sample_cnt(1);
      cs_trig(55) <= '0';
      cs_trig(54 downto 0) <= (others => '0');
    end generate fileio_cs;

  end block cs_debug;

end RTL;
