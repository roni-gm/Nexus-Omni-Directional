#!/bin/bash
# debug_system.sh — Nexus Omni Nav Debug Checker
# ================================================
# Jalankan ini di terminal setelah sistem berjalan:
#   chmod +x debug_system.sh
#   ./debug_system.sh
#
# Output: status ✅/❌ untuk setiap komponen

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

ok()   { echo -e "${GREEN}  ✅ $1${NC}"; }
fail() { echo -e "${RED}  ❌ $1${NC}"; }
warn() { echo -e "${YELLOW}  ⚠️  $1${NC}"; }

echo "========================================"
echo "  NEXUS OMNI SYSTEM DEBUG CHECKER"
echo "========================================"

# ============================================================
echo ""
echo "── 1. TOPIC CHECK ──────────────────────"

check_topic() {
    local topic=$1
    local timeout=2
    if ros2 topic hz "$topic" --window 3 2>&1 | grep -q "average rate"; then
        rate=$(ros2 topic hz "$topic" --window 3 2>&1 | grep "average rate" | awk '{print $3}')
        ok "$topic aktif (${rate} Hz)"
    else
        fail "$topic TIDAK AKTIF"
    fi
}

# Cek topic kritis (timeout 3s per topic)
for topic in /odom /scan /cmd_vel /path /map; do
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        ok "$topic ada di list"
    else
        fail "$topic TIDAK ADA"
    fi
done

# Cek cmd_vel_safe (mungkin ada, mungkin tidak)
if ros2 topic list 2>/dev/null | grep -q "/cmd_vel_safe"; then
    warn "/cmd_vel_safe ada → pastikan path_follower publish ke sini DAN plugin subscribe sini"
else
    warn "/cmd_vel_safe tidak ada → path_follower harus publish ke /cmd_vel"
fi

# ============================================================
echo ""
echo "── 2. MAP SERVER CHECK ─────────────────"

if ros2 lifecycle get /map_server 2>/dev/null | grep -q "Active"; then
    ok "map_server: Active"
elif ros2 lifecycle get /map_server 2>/dev/null | grep -q "Inactive"; then
    fail "map_server: Inactive — jalankan: ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate"
else
    fail "map_server: tidak berjalan"
fi

if ros2 topic list | grep -q "^/map$"; then
    ok "/map topic ada"
    # Cek apakah ada subscriber
    sub_count=$(ros2 topic info /map | grep "Subscription count" | awk '{print $3}')
    if [ "$sub_count" -gt "0" ] 2>/dev/null; then
        ok "/map punya $sub_count subscriber (A* planner terhubung)"
    else
        warn "/map tidak ada subscriber — A* planner belum terima map"
    fi
else
    fail "/map tidak ada — map_server belum active"
fi

# ============================================================
echo ""
echo "── 3. TF CHAIN CHECK ───────────────────"

check_tf() {
    local from=$1
    local to=$2
    if ros2 run tf2_ros tf2_echo "$from" "$to" 2>&1 | grep -q "Translation"; then
        ok "TF: $from → $to OK"
    else
        fail "TF: $from → $to PUTUS"
    fi
}

# Test TF chain (timeout 2s)
timeout 3 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "Translation" && \
    ok "TF: map → odom OK" || fail "TF: map → odom PUTUS (localization tidak jalan?)"

timeout 3 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation" && \
    ok "TF: odom → base_link OK" || fail "TF: odom → base_link PUTUS (planar_move tidak aktif?)"

timeout 3 ros2 run tf2_ros tf2_echo base_link laser 2>&1 | grep -q "Translation" && \
    ok "TF: base_link → laser OK" || fail "TF: base_link → laser PUTUS (robot_state_publisher bermasalah?)"

# ============================================================
echo ""
echo "── 4. NODE CHECK ───────────────────────"

for node in /astar_planner /path_follower /omni_kinematics /map_server /robot_state_publisher; do
    if ros2 node list 2>/dev/null | grep -q "$node"; then
        ok "Node $node jalan"
    else
        fail "Node $node TIDAK JALAN"
    fi
done

# ============================================================
echo ""
echo "── 5. PLUGIN TOPIC MISMATCH ────────────"

# Cek planar_move subscribe ke /cmd_vel atau /cmd_vel_safe
PF_PUBLISH=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Publisher count" | awk '{print $3}')
PF_SUB=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Subscription count" | awk '{print $3}')

echo "  /cmd_vel: publishers=$PF_PUBLISH, subscribers=$PF_SUB"
if [ "$PF_PUBLISH" -gt "0" ] && [ "$PF_SUB" -gt "0" ] 2>/dev/null; then
    ok "/cmd_vel flow lengkap (publisher → subscriber)"
elif [ "$PF_PUBLISH" -gt "0" ] && [ "$PF_SUB" -eq "0" ] 2>/dev/null; then
    fail "/cmd_vel ada publisher tapi TIDAK ADA subscriber (plugin tidak subscribe)"
elif [ "$PF_PUBLISH" -eq "0" ] && [ "$PF_SUB" -gt "0" ] 2>/dev/null; then
    fail "/cmd_vel ada subscriber tapi TIDAK ADA publisher (path_follower tidak publish)"
else
    warn "/cmd_vel tidak ada aktivitas"
fi

# ============================================================
echo ""
echo "── 6. PERFORMA CPU ─────────────────────"

cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2 + $4}')
echo "  CPU usage: ${cpu_usage}%"

if (( $(echo "$cpu_usage > 80" | bc -l) )); then
    fail "CPU > 80% — sistem berat! Jalankan Gazebo headless atau kurangi sensor rate"
elif (( $(echo "$cpu_usage > 60" | bc -l) )); then
    warn "CPU 60-80% — masih bisa tapi mungkin ada lag"
else
    ok "CPU < 60% — performa OK"
fi

# ============================================================
echo ""
echo "── 7. QUICK FIX COMMANDS ───────────────"
echo ""
echo "  # Fix map_server manual:"
echo "  ros2 lifecycle set /map_server configure"
echo "  ros2 lifecycle set /map_server activate"
echo ""
echo "  # Test robot gerak manual:"
echo "  ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}}' --once"
echo ""
echo "  # Send goal ke A*:"
echo "  ros2 topic pub /goal_pose geometry_msgs/PoseStamped \\"
echo "    '{header:{frame_id:\"map\"}, pose:{position:{x:2.0,y:1.0},orientation:{w:1.0}}}' --once"
echo ""
echo "  # Monitor semua topic sekaligus:"
echo "  ros2 topic hz /odom /scan /cmd_vel /path &"
echo ""
echo "========================================"
echo "  SELESAI"
echo "========================================"
