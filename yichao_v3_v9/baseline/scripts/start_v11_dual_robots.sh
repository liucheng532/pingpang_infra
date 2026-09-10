#!/usr/bin/env bash
set -Eeuo pipefail

readonly SESSION_NAME=v11_dual_robots
readonly REMOTE_ROOT=/home/unitree/haoran/doubles-v11-r2i299-resume42500-dual-runtime
readonly SSH_KEY=/home/odl/.ssh/pingpang
readonly STARTUP_LOG_ROOT=/home/odl/logs/v11_dual_robots
readonly ROS_MASTER_URI_CURRENT=http://192.168.123.165:11311
readonly POLICY_DIR=policy/v11_resume_i42500
readonly POLICY_NAME=student_v11_r2i299_commonhold_1666_resume42500.onnx
readonly POLICY_SHA256=d7be604c882307f6951c4649607dd6a07e6dbbd31bd3d80a481920ff5f762eaa
readonly SIDECAR_SHA256=0603032faa7b0f5c2a81823a1d8db16f4689c399b5950a536cd811905acd7674
readonly CHECKPOINT_SHA256=04461633b8ad75a2b173ceacf34257fae437cbf935f6effe8b279464586bea12
readonly COMMON_HOLD_SHA256=9565a8ed1ba22cfc0d759d0a8327dc7dd37989c242d4f4337dcc79c796ea3a8f
readonly HIT_ARM7_ROOT=/home/unitree/haoran/v01-arm7-residual-deploy-safe-20260810-50hz
readonly HIT_ARM7_RUNTIME_SHA256=e199d57c3e90d54c01cfc9bb810da9e03ad53330d80eda03f57f57e46c247240
readonly HIT_ARM7_ONNX_SHA256=e3ef9aa8adcbbb317e010b4175be76199d71203bb55f18d2bc4e978e9c0f9045
readonly HIT_ARM7_CHECKPOINT_SHA256=ed348b0923f8b4d618553eb734d0b57e70d880870d84d1c1942854d881f3d2ed
readonly V11_ARM7_DIR=$REMOTE_ROOT/policy/v11_teacher_arm7_i15000
readonly V11_ARM7_ONNX_SHA256=0cace9a4b950c82f2f5b5468f61b2d31d319518f641b5627d3bb6048514fc102
readonly V11_ARM7_CHECKPOINT_SHA256=f08fffa370e47ce783087a53534b4e69a293e60133e6b8ad3ca36b817fd4ee9a
readonly V11_ARM7_SIDECAR_SHA256=d654644c064149e7d7c41889fd9f91c069d4b0236ccc602e2bc1ff64a0012502
readonly HOST_198=192.168.124.164
readonly HOST_66=192.168.123.164
readonly SSH_OPTIONS=(
  -i "$SSH_KEY"
  -o BatchMode=yes
  -o StrictHostKeyChecking=yes
  -o ConnectTimeout=5
)

usage() {
  cat <<'EOF'
Usage:
  scripts/start_v11_dual_robots.sh [start] [shadow|active] [normal|hit_home] [off|shadow|active] [scale] [v01|v11-teacher]
  scripts/start_v11_dual_robots.sh check [shadow|active] [normal|hit_home] [off|shadow|active] [scale] [v01|v11-teacher]
  scripts/start_v11_dual_robots.sh dry-run [shadow|active] [normal|hit_home] [off|shadow|active] [scale] [v01|v11-teacher]
  scripts/start_v11_dual_robots.sh status
  scripts/start_v11_dual_robots.sh stop

The default is "start shadow normal shadow 1.0 v01".
v01 selects Arm7 i9500; v11-teacher selects Teacher i15000 with gate=1
during HIT/POST and invalid-ball fallback to the Teacher.
Example: scripts/start_v11_dual_robots.sh dry-run active normal active 1.0 v11-teacher
This script starts only the two robots; start the predictor/planner/monitor stack
separately before running it.
EOF
}

action=${1:-start}
mode=${2:-shadow}
profile=${3:-normal}
residual_mode=${4:-shadow}
residual_scale=${5:-1.0}
residual_family=${6:-v01}
if [[ $action == shadow || $action == active ]]; then
  profile=${2:-normal}
  residual_mode=${3:-shadow}
  residual_scale=${4:-1.0}
  residual_family=${5:-v01}
  mode=$action
  action=start
fi

case "$action" in
  start|check|dry-run) ;;
  status|stop)
    if (($# > 1)); then
      usage >&2
      exit 64
    fi
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    printf 'Unknown action: %s\n' "$action" >&2
    usage >&2
    exit 64
    ;;
esac

if [[ $action == start || $action == check || $action == dry-run ]]; then
  case "$mode" in
    shadow|active) ;;
    *)
      printf 'Mode must be shadow or active, got: %s\n' "$mode" >&2
      exit 64
      ;;
  esac
  case "$profile" in
    normal|hit_home) ;;
    *)
      printf 'Profile must be normal or hit_home, got: %s\n' "$profile" >&2
      exit 64
      ;;
  esac
  case "$residual_mode" in
    off|shadow|active) ;;
    *)
      printf 'Residual mode must be off, shadow or active, got: %s\n' \
        "$residual_mode" >&2
      exit 64
      ;;
  esac
  case "$residual_family" in
    v01|v11-teacher) ;;
    *) printf 'Residual family must be v01 or v11-teacher, got: %s\n' "$residual_family" >&2; exit 64 ;;
  esac
  if [[ ! $residual_scale =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] \
    || ! awk -v value="$residual_scale" 'BEGIN { exit !(value >= 0.0 && value <= 1.0) }'; then
    printf 'Residual scale must be within [0, 1], got: %s\n' "$residual_scale" >&2
    exit 64
  fi
fi

ssh_robot() {
  local host=$1
  shift
  ssh "${SSH_OPTIONS[@]}" "unitree@${host}" "$@"
}

remote_g1_command() {
  local robot_id=$1
  printf 'exec %q %q' \
    "$REMOTE_ROOT/g1_gym_deploy/scripts/start_v9_g1_control.sh" \
    "$robot_id"
}

remote_policy_command() {
  local script_name=$1
  local selected_mode=$2
  local selected_profile=$3
  local selected_residual_mode=$4
  local selected_residual_scale=$5
  local selected_residual_family=$6
  printf 'exec %q %q %q %q %q %q' \
    "$REMOTE_ROOT/g1_gym_deploy/scripts/$script_name" \
    "$selected_mode" \
    "$selected_profile" \
    "$selected_residual_mode" \
    "$selected_residual_scale" \
    "$selected_residual_family"
}

ssh_shell_command() {
  local host=$1
  local remote_command=$2
  local joined
  printf -v joined '%q ' \
    ssh -tt "${SSH_OPTIONS[@]}" "unitree@${host}" "$remote_command"
  printf 'exec %s' "$joined"
}

require_ros_topic() {
  local topic=$1
  if ! rostopic info "$topic" 2>/dev/null \
    | sed -n '/^Publishers:/,/^Subscribers:/p' \
    | grep -q '^ \* '; then
    printf 'Preflight FAILED: ROS topic has no publisher: %s\n' "$topic" >&2
    return 1
  fi
}

require_residual_ball_topic() {
  local topic=/residual/mocap_ball_state
  local info
  local publisher_count
  local topic_type
  info=$(rostopic info "$topic" 2>/dev/null) || {
    printf 'Preflight FAILED: residual ball topic is unavailable: %s\n' "$topic" >&2
    return 1
  }
  publisher_count=$(sed -n '/^Publishers:/,/^Subscribers:/p' <<<"$info" \
    | grep -c '^ \* ' || true)
  if [[ $publisher_count != 1 ]]; then
    printf 'Preflight FAILED: residual ball topic requires exactly one publisher: %s (got %s)\n' \
      "$topic" "$publisher_count" >&2
    return 1
  fi
  topic_type=$(rostopic type "$topic" 2>/dev/null) || return 1
  if [[ $topic_type != std_msgs/Float64MultiArray ]]; then
    printf 'Preflight FAILED: residual ball topic has type %s, expected std_msgs/Float64MultiArray\n' \
      "$topic_type" >&2
    return 1
  fi
}

preflight_robot() {
  local label=$1
  local host=$2
  local policy_script=$3
  local expected_role=$4
  ssh_robot "$host" "set -Eeuo pipefail
test -x '$REMOTE_ROOT/unitree_sdk2/build/bin/g1_control'
test -x '$REMOTE_ROOT/g1_gym_deploy/scripts/start_v9_g1_control.sh'
test -x '$REMOTE_ROOT/g1_gym_deploy/scripts/$policy_script'
test -f '$REMOTE_ROOT/$POLICY_DIR/$POLICY_NAME'
test -f '$REMOTE_ROOT/$POLICY_DIR/$POLICY_NAME.json'
test -f '$REMOTE_ROOT/$POLICY_DIR/student_iteration_042500.pt'
test -f '$REMOTE_ROOT/$POLICY_DIR/source29_0368_upright_fk_v1.npz'
actual=\$(sha256sum '$REMOTE_ROOT/$POLICY_DIR/$POLICY_NAME' | awk '{print \$1}')
test \"\$actual\" = '$POLICY_SHA256'
test \"\$(sha256sum '$REMOTE_ROOT/$POLICY_DIR/$POLICY_NAME.json' | awk '{print \$1}')\" = '$SIDECAR_SHA256'
test \"\$(sha256sum '$REMOTE_ROOT/$POLICY_DIR/student_iteration_042500.pt' | awk '{print \$1}')\" = '$CHECKPOINT_SHA256'
test \"\$(sha256sum '$REMOTE_ROOT/$POLICY_DIR/source29_0368_upright_fk_v1.npz' | awk '{print \$1}')\" = '$COMMON_HOLD_SHA256'
if [[ '$residual_mode' != off ]]; then
  runtime='$HIT_ARM7_ROOT/g1_gym_deploy/utils/residual_policy.py'
  residual='$HIT_ARM7_ROOT/policy/v01_arm7_i9500.onnx'
  checkpoint='$HIT_ARM7_ROOT/policy/provenance/v01_arm7_i9500.pt'
  residual_sha='$HIT_ARM7_ONNX_SHA256'
  checkpoint_sha='$HIT_ARM7_CHECKPOINT_SHA256'
  if [[ '$residual_family' == v11-teacher ]]; then
    residual='$V11_ARM7_DIR/v11_teacher_arm7_i15000_hard1.onnx'
    checkpoint='$V11_ARM7_DIR/model_15000.pt'
    residual_sha='$V11_ARM7_ONNX_SHA256'
    checkpoint_sha='$V11_ARM7_CHECKPOINT_SHA256'
    test -f '$REMOTE_ROOT/g1_gym_deploy/utils/v11_teacher_residual_adapter.py'
    grep -q -- '--hit-arm7-residual-family' '$REMOTE_ROOT/g1_gym_deploy/scripts/$policy_script'
    grep -q -- '--hit-arm7-residual-family' '$REMOTE_ROOT/g1_gym_deploy/scripts/deploy_policy.py'
    test \"\$(sha256sum \"\$residual.json\" | awk '{print \$1}')\" = '$V11_ARM7_SIDECAR_SHA256'
  fi
  test -f \"\$runtime\" && test -f \"\$residual\" && test -f \"\$checkpoint\"
  test \"\$(sha256sum \"\$runtime\" | awk '{print \$1}')\" = '$HIT_ARM7_RUNTIME_SHA256'
  test \"\$(sha256sum \"\$residual\" | awk '{print \$1}')\" = \"\$residual_sha\"
  test \"\$(sha256sum \"\$checkpoint\" | awk '{print \$1}')\" = \"\$checkpoint_sha\"
fi
if pgrep -x g1_control >/dev/null; then
  printf 'Preflight FAILED: g1_control is already running on %s\n' '$label' >&2
  exit 69
fi
if pgrep -f '[p]ython3 .*scripts/deploy_policy.py' >/dev/null; then
  printf 'Preflight FAILED: deploy_policy.py is already running on %s\n' '$label' >&2
  exit 69
fi
timeout 3 bash -c '</dev/tcp/192.168.123.165/11311'
printf 'Preflight OK: robot=%s role=%s policy_sha256=%s\n' \
  '$label' '$expected_role' \"\$actual\""
}

preflight_all() {
  test -r "$SSH_KEY"
  command -v tmux >/dev/null
  command -v rostopic >/dev/null
  if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    printf 'Preflight FAILED: tmux session already exists: %s\n' "$SESSION_NAME" >&2
    return 1
  fi
  require_ros_topic /doubles/table_left/torso_pose_origin
  require_ros_topic /doubles/table_right/torso_pose_origin
  if [[ $residual_mode != off ]]; then
    require_residual_ball_topic
  fi
  preflight_robot 198 "$HOST_198" start_v11_resume42500_hitteacher_198.sh \
    table_left-canonical-outward
  preflight_robot 66 "$HOST_66" start_v11_resume42500_hitteacher_leftmirror_66.sh \
    table_right-leftmirror-home-first-hitter
}

wait_for_remote_process() {
  local label=$1
  local host=$2
  local pattern=$3
  ssh_robot "$host" "for _attempt in \$(seq 1 50); do
  if pgrep -f '$pattern' >/dev/null; then exit 0; fi
  sleep 0.2
done
printf 'Timed out waiting for %s on %s\n' '$pattern' '$label' >&2
exit 1"
}

wait_for_window_output() {
  local label=$1
  local window_name=$2
  local expected=$3
  for _attempt in $(seq 1 150); do
    if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
      printf 'Startup FAILED: tmux session disappeared while waiting for %s\n' "$label" >&2
      return 1
    fi
    if [[ $(tmux display-message -p -t "$SESSION_NAME:$window_name" '#{pane_dead}') == 1 ]]; then
      printf 'Startup FAILED: %s exited before readiness marker: %s\n' \
        "$label" "$expected" >&2
      tmux capture-pane -p -t "$SESSION_NAME:$window_name" -S - >&2 || true
      return 1
    fi
    if tmux capture-pane -p -t "$SESSION_NAME:$window_name" -S - \
      | grep -Fq "$expected"; then
      printf 'Ready: %s (%s)\n' "$label" "$expected"
      return 0
    fi
    sleep 0.2
  done
  printf 'Startup FAILED: timed out waiting for %s marker: %s\n' \
    "$label" "$expected" >&2
  tmux capture-pane -p -t "$SESSION_NAME:$window_name" -S - >&2 || true
  return 1
}

create_window() {
  local window_name=$1
  local host=$2
  local remote_command=$3
  local command
  command=$(ssh_shell_command "$host" "$remote_command")
  tmux new-window -d -t "$SESSION_NAME" -n "$window_name" "$command"
  tmux set-window-option -t "$SESSION_NAME:$window_name" remain-on-exit on >/dev/null
}

capture_startup_logs() {
  if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    return 0
  fi
  local log_dir="$STARTUP_LOG_ROOT/failed_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$log_dir"
  local window_name
  while IFS= read -r window_name; do
    tmux capture-pane -p -t "$SESSION_NAME:$window_name" -S - \
      >"$log_dir/$window_name.log" 2>&1 || true
  done < <(tmux list-windows -t "$SESSION_NAME" -F '#{window_name}')
  printf 'Saved failed startup logs: %s\n' "$log_dir" >&2
}

interrupt_window() {
  local window_name=$1
  tmux send-keys -t "$SESSION_NAME:$window_name" C-c 2>/dev/null || true
}

stop_session() {
  if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    printf 'No tmux session named %s\n' "$SESSION_NAME"
    return 0
  fi
  interrupt_window policy-66
  interrupt_window policy-198
  sleep 2
  interrupt_window g1-66
  interrupt_window g1-198
  sleep 2
  tmux kill-session -t "$SESSION_NAME"
  printf 'Stopped tmux session %s\n' "$SESSION_NAME"
}

show_status() {
  if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
    tmux list-windows -t "$SESSION_NAME" \
      -F '#{window_name} pane_dead=#{pane_dead} exit=#{pane_dead_status} pid=#{pane_pid}'
  else
    printf 'tmux session %s is not running\n' "$SESSION_NAME"
  fi
  ssh_robot "$HOST_198" \
    "printf '198 processes:\\n'; pgrep -af '[g]1_control|[d]eploy_policy.py' || true"
  ssh_robot "$HOST_66" \
    "printf '66 processes:\\n'; pgrep -af '[g]1_control|[d]eploy_policy.py' || true"
}

show_dry_run() {
  printf 'action=start mode=%s profile=%s residual=%s scale=%s family=%s session=%s\n' \
    "$mode" "$profile" "$residual_mode" "$residual_scale" "$residual_family" "$SESSION_NAME"
  printf '198 g1: host=%s start_v9_g1_control.sh table_left\n' "$HOST_198"
  printf '198 policy: host=%s start_v11_resume42500_hitteacher_198.sh %s %s %s %s %s\n' \
    "$HOST_198" "$mode" "$profile" "$residual_mode" "$residual_scale" "$residual_family"
  printf '66 g1: host=%s start_v9_g1_control.sh table_right\n' "$HOST_66"
  printf '66 policy: host=%s start_v11_resume42500_hitteacher_leftmirror_66.sh %s %s %s %s %s\n' \
    "$HOST_66" "$mode" "$profile" "$residual_mode" "$residual_scale" "$residual_family"
}

case "$action" in
  dry-run)
    show_dry_run
    exit 0
    ;;
  check)
    set +u
    source /opt/ros/noetic/setup.bash
    set -u
    export ROS_MASTER_URI=$ROS_MASTER_URI_CURRENT
    export ROS_IP=192.168.123.165
    preflight_all
    exit 0
    ;;
  status)
    show_status
    exit 0
    ;;
  stop)
    stop_session
    show_status
    exit 0
    ;;
esac

set +u
source /opt/ros/noetic/setup.bash
set -u
export ROS_MASTER_URI=$ROS_MASTER_URI_CURRENT
export ROS_IP=192.168.123.165

preflight_all
if [[ $mode == active ]]; then
  printf '%s\n' \
    'WARNING: ACTIVE mode can publish q_des after each robot passes its R2 admission sequence.' >&2
fi
if [[ $residual_mode == active ]]; then
  printf 'WARNING: ACTIVE Arm7 residual scale=%s will modify HIT/POST_DELAY actions.\n' \
    "$residual_scale" >&2
fi

cleanup_failed_start=1
trap 'if ((cleanup_failed_start)); then capture_startup_logs; stop_session; fi' ERR INT TERM

first_command=$(ssh_shell_command "$HOST_198" "$(remote_g1_command table_left)")
tmux new-session -d -s "$SESSION_NAME" -n g1-198 "$first_command"
tmux set-window-option -t "$SESSION_NAME:g1-198" remain-on-exit on >/dev/null
create_window g1-66 "$HOST_66" "$(remote_g1_command table_right)"

wait_for_remote_process 198 "$HOST_198" \
  "$REMOTE_ROOT/unitree_sdk2/build/bin/g1_control"
wait_for_remote_process 66 "$HOST_66" \
  "$REMOTE_ROOT/unitree_sdk2/build/bin/g1_control"
wait_for_window_output 198-g1 g1-198 'G1 type:'
wait_for_window_output 66-g1 g1-66 'G1 type:'
sleep 2

create_window policy-198 "$HOST_198" \
  "$(remote_policy_command start_v11_resume42500_hitteacher_198.sh \
    "$mode" "$profile" "$residual_mode" "$residual_scale" "$residual_family")"
wait_for_remote_process 198 "$HOST_198" '[p]ython3 .*scripts/deploy_policy.py'
if [[ $mode == active ]]; then
  policy_ready_marker='About to calibrate'
else
  policy_ready_marker='[DEPLOY]'
fi
wait_for_window_output 198-policy policy-198 "$policy_ready_marker"

create_window policy-66 "$HOST_66" \
  "$(remote_policy_command start_v11_resume42500_hitteacher_leftmirror_66.sh \
    "$mode" "$profile" "$residual_mode" "$residual_scale" "$residual_family")"
wait_for_remote_process 66 "$HOST_66" '[p]ython3 .*scripts/deploy_policy.py'
wait_for_window_output 66-policy policy-66 "$policy_ready_marker"

sleep 2
wait_for_remote_process 198 "$HOST_198" '[p]ython3 .*scripts/deploy_policy.py'
wait_for_remote_process 66 "$HOST_66" '[p]ython3 .*scripts/deploy_policy.py'
require_ros_topic /doubles/table_left/state
require_ros_topic /doubles/table_right/state

cleanup_failed_start=0
trap - ERR INT TERM
printf 'Started both V11 robots: mode=%s profile=%s residual=%s scale=%s family=%s.\n' \
  "$mode" "$profile" "$residual_mode" "$residual_scale" "$residual_family"
if [[ $mode == active ]]; then
  printf 'Both policies are ready and waiting for their R2 admission sequence.\n'
fi
printf 'View: tmux attach -t %s\n' "$SESSION_NAME"
printf 'Status: %s status\n' "$0"
printf 'Stop: %s stop\n' "$0"
