#!/bin/bash
# Prototype experiment matrix (see reports/autopilots2026_10_24).
# usage: matrix.sh <case> [<case>...]   -- runs in parallel, logs to tmp/ap/
cd "$(dirname "$0")/../.." || exit 1
SHIP=res/ships/mars_explorer_mk1.json
T=${AP_TIMEOUT:-180}
ACC=${AP_ACCEL:-4}

run() {  # name startopt pilotopts
  local n=$1; shift
  SDL_VIDEODRIVER=offscreen xvfb-run -a ./osp --timeout "$T" --time-accel "$ACC" \
    --orbit-log --orbit-interval 30 "$@" \
    > "tmp/ap/$n.log" 2>&1
}

for n in "$@"; do
  case "$n" in
    L_moho_law1)      run "$n" --startship mex,$SHIP,Moho,pad --pilot ascent --pilot-alt 50000 --pilot-law 1 ;;
    M_moho_law0)      run "$n" --startship mex,$SHIP,Moho,pad --pilot ascent --pilot-alt 50000 --pilot-law 0 ;;
    N_moho_inc90)     run "$n" --startship mex,$SHIP,Moho,pad --pilot ascent --pilot-alt 50000 --pilot-law 1 --pilot-inc 90 ;;
    O_kerbin_top)     run "$n" --startship mex,$SHIP,Kerbin,pad --pilot ascent --pilot-alt 50000 --pilot-law 1 ;;
    P_kerbin_low)     run "$n" --startship mex,$SHIP,Kerbin,pad --pilot ascent --pilot-alt 50000 --pilot-law 1 --pilot-clear 5000 ;;
    Q_kerbin_mid)     run "$n" --startship mex,$SHIP,Kerbin,pad --pilot ascent --pilot-alt 50000 --pilot-law 1 --pilot-clear 20000 ;;
    R_moho_land)      run "$n" --startship mex,$SHIP,Moho,rot-orbit --pilot land ;;
    S_kerbin_land)    run "$n" --startship mex,$SHIP,Kerbin,rot-orbit --pilot land ;;
    U_moho_land2)     run "$n" --startship mex,$SHIP,Moho,rot-orbit --pilot land ;;
    V_kerbin_land2)   run "$n" --startship mex,$SHIP,Kerbin,rot-orbit --pilot land ;;
    W_moho_law1g)     run "$n" --startship mex,$SHIP,Moho,pad --pilot ascent --pilot-alt 50000 --pilot-law 1 ;;
    X_moho_law0g)     run "$n" --startship mex,$SHIP,Moho,pad --pilot ascent --pilot-alt 50000 --pilot-law 0 ;;
    Y_moho_law1s)     run "$n" --startship mex,$SHIP,Moho,pad --pilot ascent --pilot-alt 50000 --pilot-law 1 ;;
    Z_kerbin_tops)    run "$n" --startship mex,$SHIP,Kerbin,pad --pilot ascent --pilot-alt 50000 --pilot-law 1 ;;
    AA_moho_land3)    run "$n" --startship mex,$SHIP,Moho,rot-orbit --pilot land ;;
    AB_kerbin_mids)   run "$n" --startship mex,$SHIP,Kerbin,pad --pilot ascent --pilot-alt 50000 --pilot-law 1 --pilot-clear 20000 ;;
    *) echo "unknown case $n"; exit 1 ;;
  esac &
done
wait
echo done
