#!/usr/bin/env bash
# Extract Ubuntu ARM64 OpenCV 4.5.4 development files without changing system packages.
set -euo pipefail
if [[ $(dpkg --print-architecture) != arm64 ]]; then
  echo 'This helper targets AGX ARM64 only.' >&2
  exit 1
fi
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
local_root="$script_dir/../../third_party/opencv_4_5"
mkdir -p "$local_root/debs" "$local_root/prefix"
cd "$local_root/debs"
version='4.5.4+dfsg-9ubuntu4'
packages=(libopencv-dev libopencv-core-dev libopencv-calib3d-dev libopencv-contrib-dev
  libopencv-dnn-dev libopencv-features2d-dev libopencv-flann-dev libopencv-highgui-dev
  libopencv-imgcodecs-dev libopencv-imgproc-dev libopencv-ml-dev libopencv-objdetect-dev
  libopencv-photo-dev libopencv-shape-dev libopencv-stitching-dev libopencv-superres-dev
  libopencv-video-dev libopencv-videoio-dev libopencv-videostab-dev libopencv-viz-dev
  libopencv-videostab4.5d libopencv-superres4.5d)
for package in "${packages[@]}"; do
  file="${package}_${version}_arm64.deb"
  if [[ ! -f $file ]]; then
    apt-get download "$package=$version"
  fi
  dpkg-deb -x "$file" ../prefix
done
/usr/bin/python3 - <<'PY'
from pathlib import Path
root = Path('../prefix/usr/lib/aarch64-linux-gnu')
for library in Path('/usr/lib/aarch64-linux-gnu').glob('libopencv*.so.4.5*'):
    target = root / library.name
    if not target.exists() and not target.is_symlink():
        target.symlink_to(library)
PY
printf 'OpenCV_DIR=%s\n' "$(cd ../prefix && pwd)/usr/lib/aarch64-linux-gnu/cmake/opencv4"
