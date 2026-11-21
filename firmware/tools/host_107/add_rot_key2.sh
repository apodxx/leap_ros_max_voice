#!/bin/bash
export LC_ALL=C
F=/root/ros2_web/src/components/SlamMap.vue

python3 - "$F" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
if "viewRot.value += Math.PI / 2" in s:
    print("already has key handler")
else:
    marker = "onMounted(() => {"
    idx = s.find(marker)
    if idx < 0:
        print("onMounted not found")
        sys.exit(1)
    insertion = marker + """
  window.addEventListener('keydown', (e) => {
    const k = e.key
    if (k === 'r' || k === 'R') {
      viewRot.value += Math.PI / 2
      requestDraw()
    }
  })"""
    s = s[:idx] + insertion + s[idx + len(marker):]
    open(p, 'w').write(s)
    print("inserted R key handler into onMounted")
PY
grep -n "viewRot.value += Math.PI" "$F" | head -3
cd /root/ros2_web || exit 1
npm run build > /tmp/web_build5.log 2>&1
echo "build rc=$?"
tail -2 /tmp/web_build5.log | tr -cd '[:print:]\n'
