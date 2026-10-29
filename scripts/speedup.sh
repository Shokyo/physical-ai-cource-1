#!/usr/bin/env bash
# Speed up a recording N times (video only). Usage: scripts/speedup.sh in.mp4 out.mp4 [factor=2]
set -e
IN=$1; OUT=$2; F=${3:-2}
ffmpeg -loglevel warning -i "$IN" -filter:v "setpts=PTS/${F}" -an -c:v libx264 -crf 20 -pix_fmt yuv420p "$OUT"
echo "wrote $OUT (x${F})"
