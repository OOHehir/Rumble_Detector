# Parse elf file
~/.platformio/packages/toolchain-xtensa-esp32@8.4.0+2021r2-patch5/bin/xtensa-esp32-elf-objdump -x .pio/build/esp-wrover/firmware.elf > firmware.elf.txt

# Trace location of a memory address in the elf file (useful for tracing reason for a crash)
~/.platformio/packages/toolchain-xtensa-esp32@8.4.0+2021r2-patch5/bin/xtensa-esp32-elf-addr2line -pfiaC -e .pio/build/esp-wrover/firmware.elf 0x400d752c
