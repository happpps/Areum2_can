#!/usr/bin/env bash

cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" || exit 1

CAN_INT_0="can0"
CAN_INT_1="can1"
BITRATE="1000000"
SAMPLE_POINT="0.875"
LOG_DIR="log"

echo "--- $CAN_INT 인터페이스 초기화 시작 ---"

ipcrm -M 13563267


sudo ip link set ${CAN_INT_0} up type can bitrate ${BITRATE} sample-point ${SAMPLE_POINT} restart-ms 1
sudo ip link set ${CAN_INT_1} up type can bitrate ${BITRATE} sample-point ${SAMPLE_POINT} restart-ms 1


echo "프로젝트 빌드 상태를 확인합니다..."
make || exit 1

# Grant CAN interface control without another sudo prompt in the new terminal.
sudo setcap cap_net_admin=ep ./Areum2_can || exit 1

# 나중에 디버깅용. log 디렉토리가 없으면 만들고, 그 안에 candump-<타임스탬프>.log 로 남는다.
mkdir -p "${LOG_DIR}"
( cd "${LOG_DIR}" && exec candump -l ${CAN_INT_0} ${CAN_INT_1} ) &
CANDUMP_PID=$!

# 이 셸을 Ctrl+C 로 끄면 로깅도 같이 멈춘다.
trap 'kill ${CANDUMP_PID} 2>/dev/null; rm -f "${BRIDGE_PID_FILE:-}"' INT TERM EXIT



echo "CAN 통신 프로그램을 별도 터미널에서 실행합니다."
#./Areum2_can > /dev/null 2>&1 &
gnome-terminal -- ./Areum2_can

sleep 3

gnome-terminal -- ./areumii_move_pose ready

sleep 6

BRIDGE_PID_FILE=$(mktemp) || exit 1
gnome-terminal -- bash -c '
  echo "$$" > "$1"
  exec ./areumii_udp_shm_bridge
' bridge "$BRIDGE_PID_FILE"


MODEL_PROMPT="어떤 모델을 추론할까요?"
while true; do
  read -r -p "${MODEL_PROMPT} [1=이클립스 / 2=보틀 / 3=더블 / n=down] " MODEL_CHOICE || exit 1
  case "${MODEL_CHOICE}" in
    1)
      POLICY_PATH="1ys1/areumii-smolvla-handover-eclipse-v2-46"
      DATASET_REPO_ID="1ys1/rollout_handover_eclipse_v2_46"
      ;;
    2)
      POLICY_PATH="1ys1/areumii-smolvla-handover-bottle-v2-99"
      DATASET_REPO_ID="1ys1/rollout_handover_bottle_v2_99"
      ;;
    3)
      POLICY_PATH="1ys1/areumii-smolvla-handover-doubletask-v2-145"
      DATASET_REPO_ID="1ys1/rollout_handover_doubletask_v2_145"
      ;;
    n|N) break ;;
    *)
      echo "1, 2, 3 또는 n을 입력하세요."
      continue
      ;;
  esac
  echo "선택한 정책: ${POLICY_PATH}"
  gnome-terminal --wait -- bash -c '
    POLICY_PATH="$1"
    DATASET_REPO_ID="$2"

    source "$HOME/projects/env_lerobot/bin/activate" || exit 1
    cd "$HOME/projects/areumii_lerobot_plugins" || exit 1

    python areumii_rollout_prompt.py \
      --policy.path="$POLICY_PATH" \
      --robot.type=areumii \
      --robot.id=areumii_c1 \
      --robot.cmd_port=5005 \
      --robot.feedback_port=5006 \
      --robot.enable_command=true \
      --fps=30 \
      --device=cuda \
      --display_data=false \
      --interpolation_multiplier=1 \
      --play_sounds=false \
      --strategy.type=episodic \
      --dataset.repo_id="$DATASET_REPO_ID" \
      --dataset.num_episodes=5 \
      --dataset.episode_time_s=50 \
      --dataset.reset_time_s=0 \
      --dataset.push_to_hub=true \
      --inference.type=rtc \
      --inference.rtc.execution_horizon=5 \
      --inference.rtc.max_guidance_weight=10.0 \
      --rename_map="{\"observation.images.head\":\"observation.images.camera1\",\"observation.images.left_wrist\":\"observation.images.camera2\",\"observation.images.right_wrist\":\"observation.images.camera3\"}" \
      --speech.enabled=true \
      --speech.task_map=speech_tasks.json

    rollout_status=$?
    printf "\nRollout 종료 (exit code: %s)\n" "$rollout_status"
    exit "$rollout_status"
  ' rollout "$POLICY_PATH" "$DATASET_REPO_ID"
  MODEL_PROMPT="다음 모델은 어떤걸 추론할까요?"
done

BRIDGE_PID=$(cat "${BRIDGE_PID_FILE}")
if [[ ! "${BRIDGE_PID}" =~ ^[0-9]+$ ]]; then
  echo "브리지 PID를 확인할 수 없어 pose down을 실행하지 않습니다." >&2
  exit 1
fi
echo "UDP 브리지를 종료합니다."
if kill -0 "${BRIDGE_PID}" 2>/dev/null; then
  kill "${BRIDGE_PID}" || exit 1
  for ((i = 0; i < 50; i++)); do
    kill -0 "${BRIDGE_PID}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    echo "브리지가 아직 실행 중이어서 pose down을 실행하지 않습니다." >&2
    exit 1
  fi
fi
gnome-terminal -- ./areumii_move_pose down

echo "로깅 중... 종료하려면 Ctrl+C를 누르세요."
wait ${CANDUMP_PID}
