#!/usr/bin/env sed

/^MEMORY$/ {
    n;
    n;
    s/^/  BKP_SRAM (r)       : ORIGIN = 0x40024400, LENGTH = 1024\n/;
    s/^/  \/* Backup SRAM, safe against RDP2->RDP1 downgrade *\/\n/;
    s/^/  NOUPGRADE_DFU_FLASH_KEY_IV (r) : ORIGIN = 0x08100c00, LENGTH = 1024\n/;
    s/^/  \/* DFU flash over-encryption storage, not upgradable through DFU *\/\n/;
}


$s/^\}$/\
  OVERLAY ORIGIN(BKP_SRAM) : NOCROSSREFS AT (ORIGIN(NOUPGRADE_DFU_FLASH_KEY_IV))\
  {\
      .noupgrade_dfu_flash_key_iv_bkup { *(.noupgrade.dfu_flash_key_iv) }\
  }\n}/
