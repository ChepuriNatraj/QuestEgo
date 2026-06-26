#!/usr/bin/env bash
# Convert Quest Recorder frame folders into MP4 videos.
#
# For each Session_*/Take_*/video/ folder:
#   - reads timestamps from video_metadata.csv
#   - encodes frames as <Session>/<Take>.mp4
#
# FPS strategy:
#   Compute the true recording rate from the first/last timestamp_ns in
#   video_metadata.csv (real wall-clock FPS). If metadata is missing, fall back
#   to TARGET_FPS. The output fps is capped at TARGET_FPS to avoid absurd
#   stretching, and floored so that the duration stays >= MIN_DURATION_SEC.

set -euo pipefail

SRC_ROOT="/home/botforgelabs2/Desktop/QuestEgo"
OUT_ROOT="/home/botforgelabs2/Desktop/QuestEgo/Videos"
TARGET_FPS=60
MIN_DURATION_SEC=1.0

mkdir -p "$OUT_ROOT"

log() { echo "[$(date +%H:%M:%S)] $*"; }

compute_fps_from_metadata() {
  local meta="$1"
  local fc="$2"
  python3 - "$meta" "$fc" "$TARGET_FPS" "$MIN_DURATION_SEC" <<'PY'
import csv, sys
meta, frame_count_s, target_fps_s, min_dur_s = sys.argv[1:5]
frame_count = int(frame_count_s)
target_fps = float(target_fps_s)
min_dur = float(min_dur_s)
fps = target_fps
try:
    with open(meta) as f:
        rows = list(csv.DictReader(f))
    if len(rows) >= 2:
        first = int(rows[0]['timestamp_ns'])
        last  = int(rows[-1]['timestamp_ns'])
        dur = (last - first) / 1e9
        if dur > 0:
            fps = frame_count / dur
except Exception:
    pass
# Cap at target, floor so duration >= min_dur
if fps > target_fps:
    fps = target_fps
if fps * min_dur > frame_count and frame_count > 0:
    fps = frame_count / min_dur
if fps < 1:
    fps = 1.0
print(f"{fps:.4f}")
PY
}

shopt -s nullglob
sessions=("$SRC_ROOT"/Session_*/)
shopt -u nullglob

if [ ${#sessions[@]} -eq 0 ]; then
  log "No Session_* folders found in $SRC_ROOT"
  exit 1
fi

total_takes=0
for sdir in "${sessions[@]}"; do
  session_name=$(basename "$sdir")
  takes=()
  for tdir in "$sdir"/Take_*/; do
    [ -d "$tdir/video" ] || continue
    takes+=("$tdir")
  done
  [ ${#takes[@]} -gt 0 ] || { log "SKIP $session_name (no takes)"; continue; }

  out_dir="$OUT_ROOT/$session_name"
  mkdir -p "$out_dir"

  for tdir in "${takes[@]}"; do
    take_name=$(basename "$tdir")
    frames_dir="$tdir/video"
    meta="$tdir/video_metadata.csv"
    frame_count=$(find "$frames_dir" -maxdepth 1 -type f -name "frame_*.jpg" | wc -l)
    if [ "$frame_count" -lt 1 ]; then
      log "SKIP $session_name/$take_name (no frames)"
      continue
    fi

    fps="60"
    if [ -f "$meta" ]; then
      fps=$(compute_fps_from_metadata "$meta" "$frame_count")
    fi

    out_file="$out_dir/${take_name}.mp4"
    log "Encoding $session_name/$take_name  ($frame_count frames @ ${fps}fps) -> ${take_name}.mp4"

    ffmpeg -y -loglevel error \
      -framerate "$fps" \
      -i "$frames_dir/frame_%06d.jpg" \
      -c:v libx264 -preset veryfast -crf 20 \
      -pix_fmt yuv420p \
      -movflags +faststart \
      "$out_file"

    total_takes=$((total_takes + 1))
  done
done

log "DONE. Encoded $total_takes take(s). Output: $OUT_ROOT"