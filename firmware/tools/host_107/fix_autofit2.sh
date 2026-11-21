#!/bin/bash
export LC_ALL=C
F=/root/ros2_web/src/components/SlamMap.vue
python3 - "$F" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
a = "  const s = Math.min(cw / mw, ch / mh)"
b = """  const _th = viewRot.value || 0
  const _bw = Math.abs(mw * Math.cos(_th)) + Math.abs(mh * Math.sin(_th))
  const _bh = Math.abs(mw * Math.sin(_th)) + Math.abs(mh * Math.cos(_th))
  const s = Math.min(cw / _bw, ch / _bh) * 0.95"""
if a not in s:
    print("s-line not found"); sys.exit(1)
s = s.replace(a, b, 1)
s = s.replace("  offsetX.value = (cw - mw * s) / 2", "  offsetX.value = cw / 2 - (mw / 2) * s", 1)
s = s.replace("  offsetY.value = (ch - mh * s) / 2", "  offsetY.value = ch / 2 - (mh / 2) * s", 1)
open(p, 'w').write(s)
print("autoFit rotation-aware")
PY
grep -n '_bw\|offsetX.value = cw' "$F" | head -4
cd /root/ros2_web || exit 1
npm run build > /tmp/web_build7.log 2>&1
echo "build rc=$?"
tail -2 /tmp/web_build7.log | tr -cd '[:print:]\n'
