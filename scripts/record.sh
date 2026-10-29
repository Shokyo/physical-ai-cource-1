#!/usr/bin/env bash
# Record the screen (X11) with ffmpeg. Usage: scripts/record.sh [output.mp4] [fps]
# Stop with Ctrl+C. Edit/speed-up afterwards with scripts/speedup.sh.
set -e
OUT=${1:-$HOME/pai_final_ws/media/raw_$(date +%Y%m%d_%H%M%S).mp4}
FPS=${2:-30}
SIZE=$(xdpyinfo | awk '/dimensions/{print $2}')
echo "recording $SIZE @ ${FPS}fps -> $OUT"
ffmpeg -loglevel warning -video_size "$SIZE" -framerate "$FPS" -f x11grab -i "${DISPLAY:-:1}" \
  -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p "$OUT"
