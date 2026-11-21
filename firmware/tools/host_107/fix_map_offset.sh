#!/bin/bash
export LC_ALL=C
F=/root/ros2_web/src/components/SlamMap.vue
cp "$F" "$F.bak3"

python3 - "$F" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()

# 1) 在 updateOffscreenCanvas 里记录渲染时使用的原点/尺寸
old_marker = "  mapInfo.value = { width, height, resolution: info.resolution }"
new_marker = """  mapInfo.value = { width, height, resolution: info.resolution }
  renderedOrigin = {
    x: info.origin && info.origin.position ? info.origin.position.x : 0,
    y: info.origin && info.origin.position ? info.origin.position.y : 0,
    res: info.resolution,
    w: width,
    h: height
  }"""
if old_marker not in s:
    print("MARKER1 NOT FOUND"); sys.exit(1)
s = s.replace(old_marker, new_marker, 1)

# 2) 声明变量
decl = "let lastMapGeom = null"
if decl in s and "let renderedOrigin = null" not in s:
    s = s.replace(decl, "let renderedOrigin = null\nlet lastMapGeom = null", 1)
elif "let renderedOrigin = null" not in s:
    s = s.replace("function updateOffscreenCanvas(occupancyGrid) {",
                  "let renderedOrigin = null\nfunction updateOffscreenCanvas(occupancyGrid) {", 1)

# 3) 绘制时按原点差值平移图像，保证与车的位置对齐
old_draw = """  if (isMapVisible) {
    ctx.drawImage(offscreenCanvas, 0, 0)
  }"""
new_draw = """  if (isMapVisible) {
    // 图像渲染用的原点和当前地图原点可能不同（地图 1Hz、车 30Hz），
    // 按差值平移，避免车和地图错位几米。
    const curInfo = getEffectiveMapInfo()
    if (renderedOrigin && curInfo.origin && curInfo.origin.position) {
      const res = curInfo.resolution
      const px = (renderedOrigin.x - curInfo.origin.position.x) / res
      const topY = (renderedOrigin.y + renderedOrigin.h * renderedOrigin.res - curInfo.origin.position.y) / res
      const py = (curInfo.height - 1) - topY
      ctx.drawImage(offscreenCanvas, px, py)
    } else {
      ctx.drawImage(offscreenCanvas, 0, 0)
    }
  }"""
if old_draw not in s:
    print("MARKER2 NOT FOUND"); sys.exit(1)
s = s.replace(old_draw, new_draw, 1)

open(p, "w").write(s)
print("patched SlamMap.vue")
PY

cd /root/ros2_web || exit 1
npm run build > /tmp/web_build2.log 2>&1
echo "build rc=$?"
tail -3 /tmp/web_build2.log | tr -cd '[:print:]\n'
