#!/bin/bash
export LC_ALL=C
F=/root/ros2_web/src/components/SlamMap.vue
python3 - "$F" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace("const viewRot = ref(-Math.PI / 2)", "const viewRot = ref(0)", 1)
open(p, 'w').write(s)
print("viewRot default -> 0")
PY
grep -n "const viewRot" "$F" | head -2
cd /root/ros2_web || exit 1
npm run build > /tmp/web_build8.log 2>&1
echo "build rc=$?"
tail -2 /tmp/web_build8.log | tr -cd '[:print:]\n'
