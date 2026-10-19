#!/bin/bash
S=/tmp/claude-1000/-home-su-rover/2f374bec-eda0-450c-87e7-cb87e1d65215/scratchpad
until grep -qE "ALLDONE|FAIL" $S/id2.log; do sleep 30; done   # id2가 완전히 끝난 뒤에만 시작(동료·내 실험과 GPU 충돌 방지)
echo "$(date +%T) id2 종료, id3 시작"
bash $S/id3_waves.sh
