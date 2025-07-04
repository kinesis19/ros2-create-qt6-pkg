#!/bin/bash

CMD_NAME="ros2_create_qt6_pkg"
# 스크립트와 템플릿 파일이 설치될 경로. (예: /home/user/.ros2_create_qt6_pkg)
INSTALL_DIR="$HOME/.$CMD_NAME"

GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}### ROS 2 Qt6 패키지 생성기 설치를 시작합니다. ###${NC}"

# 1. 이전 설치 제거 및 설치 디렉터리 생성
if [ -d "$INSTALL_DIR" ]; then
    echo "기존 설치를 제거합니다: $INSTALL_DIR"
    rm -rf "$INSTALL_DIR"
fi
echo "설치 디렉터리를 생성합니다: $INSTALL_DIR"
mkdir -p "$INSTALL_DIR"

# 2. 스크립트 및 템플릿 파일 복사
echo "필요한 파일들을 설치 디렉터리로 복사합니다."
cp ./create_pkg.sh "$INSTALL_DIR/"
cp -r ./templates "$INSTALL_DIR/"

# 3. .bashrc에 명령어 등록
if ! grep -q "function $CMD_NAME()" "$HOME/.bashrc"; then
    echo "전역 명령어 '$CMD_NAME'을 ~/.bashrc에 추가합니다."
    echo "
# ROS2 Qt6 package creator command
function $CMD_NAME() {
    # 모든 인자를 그대로 INSTALL_DIR 안의 실제 스크립트로 전달합니다.
    \"$INSTALL_DIR/create_pkg.sh\" \"\$@\"
}" >> "$HOME/.bashrc"
else
    echo "명령어 '$CMD_NAME'가 이미 ~/.bashrc에 존재합니다."
fi

echo -e "${GREEN}### 설치가 완료되었습니다! ###${NC}"
echo "터미널을 새로 열거나 'source ~/.bashrc' 명령을 실행하여 변경사항을 적용해주세요."