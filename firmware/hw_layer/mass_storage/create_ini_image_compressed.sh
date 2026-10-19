#!/bin/bash

# from firmware folder:
# hw_layer/mass_storage/create_ini_image_compressed.sh tunerstudio/generated/rusefi.ini generated/ramdisk_image.h  1024 test https://rusefi.com/s/test

# fail on error
set -euo pipefail

if [ "$(expr substr $(uname -s) 1 6)" == "CYGWIN" ] || [ "$(expr substr $(uname -s) 1 5)" == "MINGW" ]; then
  echo No image on $(uname -s)
  exit 0
fi

if [[ "$OSTYPE" == darwin* ]]; then
  echo "Skipping INI image creation on macOS (missing dosfstools)"
  exit 0
fi

FULL_INI=$1
H_OUTPUT=$2
FS_SIZE=$3
SHORT_BOARDNAME=$4
BOARD_SPECIFIC_URL=$5

IMAGE=ramdisk.image
DEFLATE_WINDOW_BITS=14
DEFLATE_WINDOW_SIZE=$((1 << DEFLATE_WINDOW_BITS))

# mkfs.fat and fatlabel are privileged on some systems
PATH="$PATH:/usr/sbin"

echo "create_ini_image_compressed: ini $FULL_INI to $H_OUTPUT size $FS_SIZE for $SHORT_BOARDNAME [$BOARD_SPECIFIC_URL]"

cleanup() {
  rm -f "$IMAGE" "$IMAGE.gz"
}
trap cleanup EXIT
cleanup

# copy *FS_SIZE*KB of zeroes
dd if=/dev/zero of=$IMAGE bs=1024 count=$FS_SIZE

# create a FAT filesystem inside, name it "FOME EFI"
mkfs.fat $IMAGE
# labels can be no longer than 11 characters
fatlabel $IMAGE "FOME EFI"



cp hw_layer/mass_storage/filesystem_contents/README.nozip.template.txt hw_layer/mass_storage/readme.temp
echo ${BOARD_SPECIFIC_URL}       >> hw_layer/mass_storage/readme.temp

if [[ $OSTYPE == 'darwin'* ]]; then
  # Mac OS comes with Bash version 3 which is quite limiting and lack key features
  current_date='huh-MAC'
else
  bash --version
  # bash >=4.3 magic
  printf -v current_date '%(%Y-%m-%d)T' -1
  echo "create_ini_image_compressed.sh says [${current_date}]"
fi

# Put the zip inside the filesystem
mcopy -i $IMAGE $FULL_INI ::
# Put a readme text file in there too
mcopy -i $IMAGE hw_layer/mass_storage/readme.temp ::README-${current_date}.txt
mcopy -i $IMAGE hw_layer/mass_storage/filesystem_contents/FOME\ Wiki.url ::
mcopy -i $IMAGE hw_layer/mass_storage/filesystem_contents/FOME\ Discord.url ::
mcopy -i $IMAGE hw_layer/mass_storage/filesystem_contents/FOME\ Releases.url ::

# Compress the image as DEFLATE in a gzip wrapper with a fixed 16 KiB window
python3 - "$IMAGE" "$IMAGE.gz" "$DEFLATE_WINDOW_BITS" <<'PY'
import sys
import zlib

source_path, output_path, window_bits = sys.argv[1:]
compressor = zlib.compressobj(level=9, wbits=16 + int(window_bits))

with open(source_path, "rb") as source, open(output_path, "wb") as output:
    while True:
        chunk = source.read(1024 * 1024)
        if not chunk:
            break
        output.write(compressor.compress(chunk))
    output.write(compressor.flush())
PY

# write out the window contract and C array, with "static const" on the array
{
    echo "#define RAMDISK_IMAGE_DEFLATE_WINDOW_SIZE $DEFLATE_WINDOW_SIZE"
    echo -n "static const "
    xxd -i "$IMAGE.gz"
} > "$H_OUTPUT"

exit 0
