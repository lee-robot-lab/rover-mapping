for i in $(seq 1 720); do n=$(timeout 25 ssh -o BatchMode=yes -o ConnectTimeout=8 hpi-lab "docker ps --format '{{.Names}}' | grep -c rover-site || true" 2>/dev/null | tail -1); [ "$n" = "0" ] && break; sleep 20; done
up rf_u1 outputs/site_sim/cy_missions/rect_fillet25; up rf_u2 outputs/site_sim/cy_missions/rect_fillet25
st rf_u1 --gpu 0 --port 8912 $BASE $FAST --visual-turn-scale 1 --max-wall-s 3600
st rf_u2 --gpu 1 --port 8913 $BASE $FAST --visual-turn-scale 1 --max-wall-s 3600
wait2 rf_u1 rf_u2; okrun rf_u1 rf_u2; echo "정상 종료"
for l in rf_u1 rf_u2; do out=$(timeout 500 bash tools/run_site_sim_server.sh fetch $l 2>&1 | tail -1); echo "$out" | grep -q "^view:" || die "fetch $l: $out"; done
echo ALLDONE
