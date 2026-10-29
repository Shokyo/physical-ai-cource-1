#!/usr/bin/env bash
# Compose the final demo video from the off-screen recordings:
#   left  = scene_camera (1280x720), right = YOLO detections (800x600 -> 960x720)
# plus a 5 s title card. Usage: scripts/compose_video.sh <rec_dir> <out.mp4> [seconds_to_keep]
set -e
REC=${1:-$HOME/pai_final_ws/media/rec}
OUT=${2:-$HOME/pai_final_ws/media/PAI_final_demo.mp4}
DUR=${3:-}   # optional: trim the recordings to this many seconds
TRIM=(); [ -n "$DUR" ] && TRIM=(-t "$DUR")
FONT=/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc
W=2240; H=720
TITLE=$(mktemp --suffix=.mp4)
ffmpeg -loglevel error -y -f lavfi -i "color=c=0x1b1f2a:s=${W}x${H}:d=5:r=20" -vf "\
drawtext=fontfile=$FONT:text='PAI最終課題':fontsize=56:fontcolor=white:x=(w-text_w)/2:y=150,\
drawtext=fontfile=$FONT:text='安川 MOTOMAN-GP8 × YOLO による物体仕分けシミュレーション':fontsize=50:fontcolor=white:x=(w-text_w)/2:y=250,\
drawtext=fontfile=$FONT:text='ROS 2 Kilted / Gazebo / MoveIt 2 / ros2_control':fontsize=36:fontcolor=0xa0c8ff:x=(w-text_w)/2:y=370,\
drawtext=fontfile=$FONT:text='認識\: YOLO11n（シミュレータ真値で自動ラベル付けしたデータで fine-tune）':fontsize=32:fontcolor=0xdddddd:x=(w-text_w)/2:y=440,\
drawtext=fontfile=$FONT:text='左\: シミュレーション全景   右\: 天井カメラと検出結果':fontsize=32:fontcolor=0xdddddd:x=(w-text_w)/2:y=500,\
drawtext=fontfile=$FONT:text='ランダム配置した 3 回の試行 / 等倍速':fontsize=32:fontcolor=0xdddddd:x=(w-text_w)/2:y=560" \
  -c:v libx264 -crf 20 -pix_fmt yuv420p "$TITLE"
BODY=$(mktemp --suffix=.mp4)
ffmpeg -loglevel error -y "${TRIM[@]}" -i "$REC/scene_camera_image.mp4" "${TRIM[@]}" -i "$REC/detections_image.mp4" -filter_complex "\
[0:v]scale=1280:720[l];[1:v]scale=960:720[r];[l][r]hstack=2,\
drawtext=fontfile=$FONT:text='Gazebo (scene camera)':fontsize=24:fontcolor=white:box=1:boxcolor=0x00000088:boxborderw=6:x=12:y=12[v]" \
  -map "[v]" -r 20 -c:v libx264 -crf 20 -pix_fmt yuv420p "$BODY"
ffmpeg -loglevel error -y -i "$TITLE" -i "$BODY" -filter_complex "[0:v][1:v]concat=n=2:v=1:a=0[v]" -map "[v]" \
  -c:v libx264 -crf 20 -pix_fmt yuv420p -movflags +faststart "$OUT"
rm -f "$TITLE" "$BODY"
ffprobe -v error -show_entries format=duration:stream=width,height -of csv=p=0 "$OUT"
echo "wrote $OUT"
