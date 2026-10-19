#!/usr/bin/env bash
set -euo pipefail
umask 077

readonly runner_dir=/opt/actions-runner

configure_firewall() {
    # The container has a fresh network namespace. Do not permit private,
    # link-local, multicast, CGNAT, documentation, or reserved destinations.
    iptables -F OUTPUT
    iptables -P OUTPUT DROP
    iptables -A OUTPUT -o lo -j ACCEPT
    iptables -A OUTPUT -d 127.0.0.11/32 -p udp --dport 53 -j ACCEPT
    iptables -A OUTPUT -d 127.0.0.11/32 -p tcp --dport 53 -j ACCEPT

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
        iptables -A OUTPUT -d "$network" -j REJECT
    done
    iptables -A OUTPUT -j ACCEPT

    if command -v ip6tables >/dev/null && ip6tables -L OUTPUT >/dev/null 2>&1; then
        ip6tables -F OUTPUT
        ip6tables -P OUTPUT DROP
        ip6tables -A OUTPUT -o lo -j ACCEPT
        ip6tables -A OUTPUT -d ::1/128 -j ACCEPT
        ip6tables -A OUTPUT -j REJECT
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

configure_firewall

case "${1:-runner}" in
    probe)
        drop_privileges /bin/bash -ceu '
            echo "uid=$(id -u) gid=$(id -g)"
            caps=$(awk "/^CapEff:/ {print \$2}" /proc/self/status)
            [[ "$caps" =~ ^0+$ ]] || { echo "effective capabilities are not zero: $caps" >&2; exit 1; }
            echo "CapEff=$caps"
            curl --fail --silent --show-error --max-time 15 https://api.github.com/zen >/dev/null
            echo "public-network=ok"
            if curl --fail --silent --connect-timeout 3 --max-time 5 http://192.168.0.1/ >/dev/null 2>&1; then
                echo "private-network was reachable" >&2
                exit 1
            fi
            echo "private-network=blocked"
            echo "make=$(command -v make) JOBS=$JOBS NUMJOBS=$NUMJOBS"
            make -j20 -C /tmp --no-print-directory --eval='all:; @:' all >/dev/null
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
