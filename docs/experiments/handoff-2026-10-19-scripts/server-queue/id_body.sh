for i in $(seq 1 720); do n=$(timeout 25 ssh -o BatchMode=yes -o ConnectTimeout=8 hpi-lab "docker ps --format '{{.Names}}' | grep -c rover-site || true" 2>/dev/null | tail -1); [ "$n" = "0" ] && break; sleep 20; done
for l in id_norl_1 id_rl_1 id_norl_2 id_rl_2; do up $l; done
st id_norl_1 --gpu 0 --port 8900 $BASE_I
st id_rl_1   --gpu 1 --port 8901 $BASE_I --rl-policy res8_mlp_v1_rc4000
wait2 id_norl_1 id_rl_1; okrun id_norl_1 id_rl_1; echo "파동1 정상 종료"
st id_norl_2 --gpu 0 --port 8902 $BASE_I
st id_rl_2   --gpu 1 --port 8903 $BASE_I --rl-policy res8_mlp_v1_rc4000
wait2 id_norl_2 id_rl_2; okrun id_norl_2 id_rl_2; echo "파동2 정상 종료"
for l in id_norl_1 id_rl_1 id_norl_2 id_rl_2; do out=$(timeout 500 bash tools/run_site_sim_server.sh fetch $l 2>&1 | tail -1); echo "$out" | grep -q "^view:" || die "fetch $l: $out"; done
echo ALLDONE
