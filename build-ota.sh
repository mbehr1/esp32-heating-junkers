#!/bin/bash
set -e

cargo build --release
espflash save-image --chip esp32c6 -s 8mb \
  target/riscv32imac-unknown-none-elf/release/hzg-roon-junkers \
  hzg-roon-junkers.ota

echo "✅ OTA image generated: hzg-roon-junkers.ota. Flash with 'nc hzg-roon-junkers.ota|nc <device ip> 65456'"
