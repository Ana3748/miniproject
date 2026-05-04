#!/bin/bash
# cleanup.sh — Run this ONCE to remove old grid-network files that are no
# longer needed by the single-junction simulation.
#
# Usage (from ~/traffic_control):
#   bash cleanup.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NET_DIR="$SCRIPT_DIR/sumo_network"

FILES_TO_DELETE=(
    "$NET_DIR/simulation.sumocfg"   # old config pointing at grid.net.xml
    "$NET_DIR/grid.net.xml"         # multi-junction grid — not used
    "$NET_DIR/routes.rou.xml"       # grid-specific routes (A0B0, B0C0 edges)
    "$NET_DIR/nodes.nod.xml"        # source builder file — not needed at runtime
    "$NET_DIR/edges.edg.xml"        # source builder file — not needed at runtime
)

echo "=== Adaptive Traffic Control — Cleanup Script ==="
echo ""
for f in "${FILES_TO_DELETE[@]}"; do
    if [ -f "$f" ]; then
        rm -v "$f"
    else
        echo "  [skip] $f — already gone"
    fi
done

echo ""
echo "✅ Cleanup complete. sumo_network/ now contains only single-junction files."
ls -lh "$NET_DIR/"
