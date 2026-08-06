
  local_irq_enable = ABSOLUTE(0xc2718);
  local_irq_disable = ABSOLUTE(0xc271c);
  bit_set_swi = ABSOLUTE(0xc2720);
  bit_clr_swi = ABSOLUTE(0xc2724);
  p33_buf = ABSOLUTE(0xc2728);
  _p33_xor_1byte = ABSOLUTE(0xc272c);
  _p33_and_1byte = ABSOLUTE(0xc2730);
  _p33_or_1byte = ABSOLUTE(0xc2734);
  _p33_tx_1byte = ABSOLUTE(0xc2738);
  _p33_rx_1byte = ABSOLUTE(0xc273c);
  _P33_CON_SET = ABSOLUTE(0xc2740);
  memmem = ABSOLUTE(0xc2744);
  memcpy = ABSOLUTE(0xc2748);
  memmove = ABSOLUTE(0xc274c);
  memcmp = ABSOLUTE(0xc2750);
  memset = ABSOLUTE(0xc2754);
  strcmp = ABSOLUTE(0xc2758);
  strcpy = ABSOLUTE(0xc275c);
  strlen = ABSOLUTE(0xc2760);
  strncmp = ABSOLUTE(0xc2764);
  strstr = ABSOLUTE(0xc2768);
  strchr = ABSOLUTE(0xc276c);
  cbuf_read = ABSOLUTE(0xc2770);
  _cbuf_write = ABSOLUTE(0xc2774);
  cbuf_is_write_able = ABSOLUTE(0xc2778);
  cbuf_write_alloc = ABSOLUTE(0xc277c);
  cbuf_write_updata = ABSOLUTE(0xc2780);
  cbuf_read_alloc = ABSOLUTE(0xc2784);
  cbuf_read_updata = ABSOLUTE(0xc2788);
  cbuf_init = ABSOLUTE(0xc278c);
  cbuf_clear = ABSOLUTE(0xc2790);
  cbuf_rewrite = ABSOLUTE(0xc2794);
  sys_clock_init = ABSOLUTE(0xc2798);
  clk_out_pa3 = ABSOLUTE(0xc279c);
  _sfc_suspend = ABSOLUTE(0xc27a0);
  sfc_resume = ABSOLUTE(0xc27a4);
  sfc_drop_cache = ABSOLUTE(0xc27a8);
  chip_crc16 = ABSOLUTE(0xc27ac);
  CrcDecode = ABSOLUTE(0xc27b0);
  mask_init = ABSOLUTE(0xc27b4);
  wdt_clear = ABSOLUTE(0xc27b8);
  nvram_set_boot_state = ABSOLUTE(0xc27bc);
  chip_reset = ABSOLUTE(0xc27c0);
  flash_poweron = ABSOLUTE(0xc27c4);
  exception_irq_handler = ABSOLUTE(0xc27c8);
  print = ABSOLUTE(0xc27cc);
  sprintf = ABSOLUTE(0xc27d0);
  xprintf = ABSOLUTE(0xc27d4);
  puts = ABSOLUTE(0xc27d8);
  doe = ABSOLUTE(0xc27dc);
  LZ4_setStreamDecode = ABSOLUTE(0xc27e0);
  LZ4_decompress_safe_continue = ABSOLUTE(0xc27e4);
  LZ4_decompress_generic = ABSOLUTE(0xc27e8);
  LZ4_decompress_safe_usingDict = ABSOLUTE(0xc27ec);
  _mdelay = ABSOLUTE(0xc27f0);
  _udelay = ABSOLUTE(0xc27f4);
  uart_set_dma_dir = ABSOLUTE(0xc27f8);
  uart_tx_data = ABSOLUTE(0xc27fc);
  boot_arg_list = ABSOLUTE(0x4ed0);
  _IRQ_MEM_ADDR = ABSOLUTE(0x4f00);
  _MASK_MEM_BEGIN = ABSOLUTE(0x4eb0);
  _MASK_MEM_SIZE = ABSOLUTE(0x48);


_BOOT_RAM_BEGIN = _MASK_MEM_BEGIN - 0x100 - 0x50 - 0x24;
MEMORY
{
 app_code(rx) : ORIGIN = 0x100100, LENGTH = 64M-0x100
    ram0(rw) : ORIGIN = 0x0, LENGTH = _BOOT_RAM_BEGIN
    boot_ram(rw) : ORIGIN = _BOOT_RAM_BEGIN, LENGTH = 0x24
}

ENTRY(_start)

SECTIONS
{


    . = ORIGIN(boot_ram);
    .boot_data ALIGN(4):
    {
         *(.boot_info)
    } > boot_ram

    . = ORIGIN(ram0);
    .data ALIGN(4):
    {
        PROVIDE(data_buf_start = .);
        *(.data*)
        *(.*.data)
        *(.common)
        *(.ram_code)
  *(.volatile_ram_code)

        *(.*.text.cache.L1)
        *(.*.text.cache.L2)
        *(.*.text.cache.L3)
    } > ram0

 .debug_data ALIGN(4):
 {
        PROVIDE(debug_buf_start = .);
        *(.debug_bss)
        *(.debug_data)
    } > ram0


    .bss (NOLOAD) : SUBALIGN(4)
    {
        PROVIDE(bss_buf_start = .);
        . = ALIGN(32);
        *(.intr_stack)
        . = ALIGN(32);
  *(.stack_magic);
        . = ALIGN(32);
        *(.stack)
        . = ALIGN(32);
  *(.stack_magic0);
        . = ALIGN(32);
  _system_data_begin = .;
        *(.bss)
        *(.*.data.bss)
     *(.non_volatile_ram)
        _system_data_end = .;
        *(.DAC_BUFFER)
        *(.AUDIO_ADC_BUFFER)
    } > ram0

    .effect_buf ALIGN(4):
    {
        PROVIDE(effect_buf_start = .);
        . = ALIGN(4);
        *(.sp_data)
        . = ALIGN(4);
        *(.rs_data)
    } > ram0

    .a_dec_buf ALIGN(32):
    {
        PROVIDE(a_buf_start = .);
        *(.a_data);
        PROVIDE(a_buf_end = .);
    } > ram0


    OVERLAY : AT(0x200000)
    {
        .d_toy_music
        {
            PROVIDE(toy_music_buf_start = .);
            *(.toy_music_data);
            PROVIDE(toy_music_buf_end = .);
        }
        .d_midi_ctrl
        {
            PROVIDE(midi_ctrl_buf_start = .);
            *(.midi_ctrl_buf);
            PROVIDE(midi_ctrl_buf_end = .);
        }
        .d_midi
        {
            PROVIDE(midi_buf_start = .);
            *(.midi_buf);
            PROVIDE(midi_buf_end = .);
        }
        .d_ump3
        {
            . = toy_music_buf_end;
            PROVIDE(ump3_buf_start = .);
            *(.ump3_data);
            PROVIDE(ump3_buf_end = .);
        }
        .d_f1a
        {
            . = toy_music_buf_end;
            PROVIDE(f1a_1_buf_start = .);
            *(.f1a_1_buf);
            PROVIDE(f1a_1_buf_end = .);
           PROVIDE(f1a_2_buf_start = .);
            *(.f1a_2_buf);
           PROVIDE(f1a_2_buf_end = .);
        }
    } > ram0

    .heap_buf ALIGN(4):
    {
        PROVIDE(_free_start = .);
        . = LENGTH(ram0) + ORIGIN(ram0) - 1;
        PROVIDE(_free_end = .);
    } > ram0


    _ram_end = .;


    . = ORIGIN(app_code);
    .app_code ALIGN(32):
    {
        *startup.o(.text)
        . = ALIGN(32);
        *(*.f1a_code)
  _VERSION_BEGIN = .;
        KEEP(*(.version))
  _VERSION_END = .;
        *(.debug)
        *(.debug_const)
        *(.debug_code)
        *(.debug_string)


        *(*.text.const)
        *(*.text)
        *(.text)
        *(.app_root)
        *(.vm)
        . = ALIGN(32);
        _SPI_CODE_START = .;
        *(.spi_code)
        . = ALIGN(4);
        _SPI_CODE_END = .;
        *(.rodata*)
        *(.ins)

        . = ALIGN(4);
  loop_detect_handler_begin = .;
  KEEP(*(.loop_detect_region))
  loop_detect_handler_end = .;

        . = ALIGN(4);
        device_node_begin = .;
        PROVIDE(device_node_begin = .);
        KEEP(*(.device))
        _device_node_end = .;
        PROVIDE(device_node_end = .);

  vfs_ops_begin = .;
  KEEP(*(.vfs_operations))
  vfs_ops_end = .;

  . = ALIGN(4);
     lp_target_begin = .;
     PROVIDE(lp_target_begin = .);
     KEEP(*(.lp_target))
     lp_target_end = .;
     PROVIDE(lp_target_end = .);


     app_size = .;
        . = ALIGN(32);
        text_end = .;
    } >app_code


    bss_begin = ADDR(.bss);
    bss_size = SIZEOF(.bss);


    bss_size1 = _system_data_end - _system_data_begin;
    bss_begin1 = _system_data_begin;

 data_addr = ADDR(.data) ;
 data_begin = text_end ;
 data_size = SIZEOF(.data) + SIZEOF(.debug_data);

    text_size = SIZEOF(.app_code);

    _sdk_text_addr = ADDR(.app_code);
    _sdk_text_size = text_size;
    _sdk_data_size = data_size;
}
