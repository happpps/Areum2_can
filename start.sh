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
make

# 나중에 디버깅용. log 디렉토리가 없으면 만들고, 그 안에 candump-<타임스탬프>.log 로 남는다.
mkdir -p "${LOG_DIR}"
( cd "${LOG_DIR}" && exec candump -l ${CAN_INT_0} ${CAN_INT_1} ) &
CANDUMP_PID=$!

# 이 셸을 Ctrl+C 로 끄면 로깅도 같이 멈춘다.
trap 'kill ${CANDUMP_PID} 2>/dev/null' INT TERM EXIT

echo "CAN 통신 프로그램을 별도 터미널에서 실행합니다."
#./Areum2_can > /dev/null 2>&1 &
gnome-terminal -- ./Areum2_can

echo "로깅 중... 종료하려면 Ctrl+C"
wait ${CANDUMP_PID}
