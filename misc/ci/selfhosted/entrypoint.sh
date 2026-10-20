#!/usr/bin/env bash
set -euo pipefail
umask 077

readonly runner_dir=/opt/actions-runner

configure_firewall() {
    # The container has a fresh network namespace. Do not permit private,
    # link-local, multicast, CGNAT, documentation, or reserved destinations.
    iptables -F OUTPUT || return 1
    iptables -P OUTPUT DROP || return 1
    iptables -A OUTPUT -o lo -j ACCEPT || return 1
    iptables -A OUTPUT -d 127.0.0.11/32 -p udp --dport 53 -j ACCEPT || return 1
    iptables -A OUTPUT -d 127.0.0.11/32 -p tcp --dport 53 -j ACCEPT || return 1

    for network in \
        0.0.0.0/8 \
        10.0.0.0/8 \
        100.64.0.0/10 \
        169.254.0.0/16 \
        172.16.0.0/12 \
        192.0.0.0/24 \
        192.0.2.0/24 \
        192.168.0.0/16 \
        198.18.0.0/15 \
        198.51.100.0/24 \
        203.0.113.0/24 \
        224.0.0.0/4 \
        240.0.0.0/4 \
        255.255.255.255/32; do
        iptables -A OUTPUT -d "$network" -j REJECT || return 1
    done
    iptables -A OUTPUT -j ACCEPT || return 1

    local ipv6_entries=
    if [[ -e /proc/net/if_inet6 ]]; then
        ipv6_entries=$(cat /proc/net/if_inet6) || return 1
    fi
    if [[ -n "$ipv6_entries" ]]; then
        command -v ip6tables >/dev/null || return 1
        ip6tables -F OUTPUT || return 1
        ip6tables -P OUTPUT DROP || return 1
        ip6tables -A OUTPUT -o lo -j ACCEPT || return 1
        ip6tables -A OUTPUT -d ::1/128 -j ACCEPT || return 1
        ip6tables -A OUTPUT -j REJECT || return 1
    fi
}

drop_privileges() {
    exec setpriv \
        --reuid=1001 \
        --regid=1001 \
        --init-groups \
        --bounding-set=-all \
        --inh-caps=-all \
        --ambient-caps=-all \
        --no-new-privs \
        -- "$@"
}

configure_firewall || {
    echo "could not configure fail-closed network policy" >&2
    exit 1
}

case "${1:-runner}" in
    probe)
        drop_privileges /bin/bash -ceu '
            echo "uid=$(id -u) gid=$(id -g)"
            for cap in CapInh CapPrm CapEff CapBnd CapAmb; do
                value=$(awk -v key="$cap:" "\$1 == key {print \$2}" /proc/self/status)
                [[ -n "$value" && "$value" =~ ^0+$ ]] || {
                    echo "$cap is not zero: ${value:-missing}" >&2
                    exit 1
                }
                printf "%s=%s\n" "$cap" "$value"
            done
            nnp=$(awk "\$1 == \"NoNewPrivs:\" {print \$2}" /proc/self/status)
            [[ "$nnp" == 1 ]] || {
                echo "NoNewPrivs is not enabled: ${nnp:-missing}" >&2
                exit 1
            }
            echo "NoNewPrivs=$nnp"
            echo "job-net-admin=absent"
            curl --fail --silent --show-error --max-time 15 https://api.github.com/zen >/dev/null
            echo "public-network=ok"
            probe_blocked() {
                local address=$1 label=$2
                if curl --silent --show-error --noproxy "*" \
                    --connect-timeout 3 --max-time 5 \
                    --output /dev/null "http://$address/" 2>/dev/null; then
                    echo "$label was reachable at $address" >&2
                    exit 1
                fi
                echo "$label=blocked"
            }
            probe_blocked 172.17.0.1 host-gateway
            probe_blocked 192.168.0.1 private-network
            echo "make=$(command -v make) JOBS=$JOBS NUMJOBS=$NUMJOBS"
            make_flags=$(make -j20 -C /tmp --no-print-directory \
                --eval='\''all:; @printf "%s\n" "$$MAKEFLAGS"'\'' all)
            last_jobs=
            for flag in $make_flags; do
                [[ "$flag" == -j* ]] && last_jobs=$flag
            done
            [[ "$last_jobs" == -j2 ]] || {
                echo "make did not cap jobs last at -j2: $make_flags" >&2
                exit 1
            }
            echo "make-flags=$make_flags"
            echo "make-cap=ok"
        '
        ;;
    runner)
        IFS= read -r encoded_jit
        if IFS= read -r extra; then
            echo "expected exactly one base64 line on stdin" >&2
            exit 2
        fi
        if [[ ! "$encoded_jit" =~ ^[A-Za-z0-9+/]+={0,2}$ ]] || (( ${#encoded_jit} > 262144 )); then
            echo "invalid JIT input" >&2
            exit 2
        fi
        jit_config=$encoded_jit
        unset encoded_jit
        exec 0</dev/null
        cd "$runner_dir"
        drop_privileges ./run.sh --jitconfig "$jit_config"
        ;;
    *)
        echo "usage: $0 [runner|probe]" >&2
        exit 2
        ;;
esac
