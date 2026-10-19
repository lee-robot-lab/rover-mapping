#!/bin/bash
# VPN 유휴 종료(30분)를 막으려고 4분마다 서버로 SSH 한 번 + ping 한 번을 보낸다. 종료: kill $(cat vpn_keepalive.pid)
LOG=/tmp/claude-1000/-home-su-rover/2f374bec-eda0-450c-87e7-cb87e1d65215/scratchpad/vpn_keepalive.log
while true; do
  ts=$(date '+%m-%d %H:%M:%S')
  if timeout 20 ssh -o BatchMode=yes -o ConnectTimeout=8 hpi-lab 'echo ok' >/dev/null 2>&1; then s=ssh_ok; else s=ssh_FAIL; fi
  if ping -c1 -W3 166.104.224.241 >/dev/null 2>&1; then p=ping_ok; else p=ping_FAIL; fi
  echo "$ts $s $p" >> $LOG
  sleep 240
done
