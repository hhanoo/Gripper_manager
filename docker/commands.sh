# ===== Environment =====
[ -f /workspace/docker/config.sh ] && source /workspace/docker/config.sh

# ===== Config =====
source-config() {
    [ -f /workspace/docker/config.sh ] && source /workspace/docker/config.sh
}

# ===== Run =====
run-zimmer() {
    cd /workspace || return 1
    python3 zimmer_window.py "$@"
}

run-koras() {
    cd /workspace || return 1
    python3 koras_window.py "$@"
}

run-egh() {
    cd /workspace || return 1
    python3 egh.py "$@"
}

# ===== Help =====
cmd-help() {
    printf "\n[gripper-manager] Commands:\n\n"

    printf "  Run:\n"
    printf "    %-20s - %s\n" "run-zimmer" "Zimmer gripper GUI (zimmer_window.py)"
    printf "    %-20s - %s\n" "run-koras"  "KORAS gripper GUI (koras_window.py)"
    printf "    %-20s - %s\n" "run-egh"    "SCHUNK EGH CLI (egh.py)  [ARGS]"
    printf "\n"

    printf "  Config / Help:\n"
    printf "    %-20s - %s\n" "source-config" "Reload /workspace/docker/config.sh"
    printf "    %-20s - %s\n" "cmd-help"      "Show this help"
    printf "\n"
}

# ===== Show help on interactive shell =====
case $- in
    *i*) cmd-help ;;
esac
