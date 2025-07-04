#!/bin/bash

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

replace_placeholder_robustly() {
    local target_file="$1"
    local placeholder="$2"
    local replacement_str="$3"
    sed -i "s|${placeholder}|${replacement_str}|g" "$target_file"
}

# --- 메인 로직 ---
if [ -z "$1" ]; then
    echo -e "${RED}오류: 패키지 이름을 입력해야 합니다.${NC}"
    echo "사용법: ros2_create_qt6_pkg <package_name>"
    exit 1
fi
PROJECT_NAME=$1

if [ -d "$PWD/$PROJECT_NAME" ]; then
    echo -e "${RED}오류: '$PROJECT_NAME' 디렉터리가 현재 위치에 이미 존재합니다.${NC}"
    exit 1
fi

echo -e "${BLUE}추가할 의존성 패키지들을 스페이스로 구분하여 입력하세요. (예: rclcpp std_msgs)${NC}"
echo -e "${BLUE}의존성이 없다면 그냥 Enter를 누르세요:${NC}"
read -r dependencies

echo -e "${GREEN}ROS2 Qt6 패키지 '$PROJECT_NAME'을(를) 생성합니다...${NC}"

TEMPLATE_DIR="$(dirname "$0")/templates"

# ... 파일 복사 로직 ...
mkdir -p "$PROJECT_NAME/src"
mkdir -p "$PROJECT_NAME/include/$PROJECT_NAME"
mkdir -p "$PROJECT_NAME/resources/images"
mkdir -p "$PROJECT_NAME/ui"
cp "$TEMPLATE_DIR/CMakeLists.txt" "$PROJECT_NAME/"
cp "$TEMPLATE_DIR/package.xml" "$PROJECT_NAME/"
cp "$TEMPLATE_DIR/src/main.cpp" "$PROJECT_NAME/src/"
cp "$TEMPLATE_DIR/src/mainwindow.cpp" "$PROJECT_NAME/src/"
cp "$TEMPLATE_DIR/include/mainwindow.h" "$PROJECT_NAME/include/$PROJECT_NAME/"
cp "$TEMPLATE_DIR/ui/mainwindow.ui" "$PROJECT_NAME/ui/"
cp "$TEMPLATE_DIR/resources/images.qrc" "$PROJECT_NAME/resources/"
cp "$TEMPLATE_DIR/resources/images/icon.png" "$PROJECT_NAME/resources/images/"
cp "$TEMPLATE_DIR/src/qnode.cpp" "$PROJECT_NAME/src/"
cp "$TEMPLATE_DIR/include/qnode.hpp" "$PROJECT_NAME/include/$PROJECT_NAME/"

# --- 이름 치환 로직 ---
sed -i "s/__PROJECT_NAME__/${PROJECT_NAME}/g" "$PROJECT_NAME/CMakeLists.txt"
sed -i "s/__PROJECT_NAME__/${PROJECT_NAME}/g" "$PROJECT_NAME/package.xml"
# ... 모든 파일에 대한 이름 치환 ...
sed -i "s/__PROJECT_NAME__/${PROJECT_NAME}/g" "$PROJECT_NAME/src/main.cpp"
sed -i "s/__PROJECT_NAME__/${PROJECT_NAME}/g" "$PROJECT_NAME/src/mainwindow.cpp"
sed -i "s/__PROJECT_NAME__/${PROJECT_NAME}/g" "$PROJECT_NAME/include/${PROJECT_NAME}/mainwindow.h"
sed -i "s/__PROJECT_NAME__/${PROJECT_NAME}/g" "$PROJECT_NAME/src/qnode.cpp"
sed -i "s/__PROJECT_NAME__/${PROJECT_NAME}/g" "$PROJECT_NAME/include/${PROJECT_NAME}/qnode.hpp"

# --- 의존성 처리 ---
if [ -n "$dependencies" ]; then
    echo "의존성을 추가합니다: $dependencies"

    find_pkgs_str=""
    include_dirs_str=""
    link_libs_str=""
    pkg_xml_deps_str=""

    for dep in $dependencies; do
        find_pkgs_str+="find_package(${dep} REQUIRED)\n"
        # CMake 변수(${dep}_INCLUDE_DIRS)를 사용하도록 문자열 생성
        include_dirs_str+="target_include_directories(\${PROJECT_NAME} PRIVATE \${${dep}_INCLUDE_DIRS})\n"
        # CMake 변수(${dep}_LIBRARIES)를 사용하도록 문자열 생성
        link_libs_str+="    \${${dep}_LIBRARIES}\n"
        pkg_xml_deps_str+="    <depend>${dep}</depend>\n"
    done
    
    replace_placeholder_robustly "$PROJECT_NAME/CMakeLists.txt" "%(find_packages)s" "$find_pkgs_str"
    replace_placeholder_robustly "$PROJECT_NAME/CMakeLists.txt" "%(include_directories)s" "$include_dirs_str"
    replace_placeholder_robustly "$PROJECT_NAME/CMakeLists.txt" "%(link_libraries)s" "$link_libs_str"
    
    robust_multiline_replace() {
        local target_file="$1"
        local placeholder="$2"
        local replacement_str="$3"
        local tmp_file="tmp.$$.$RANDOM"
        if [ "$(grep -c "$placeholder" "$target_file")" -ne 1 ]; then return 1; fi
        local line_num=$(grep -n "$placeholder" "$target_file" | cut -d: -f1)
        head -n $((line_num - 1)) "$target_file" > "$tmp_file"
        echo -e "$replacement_str" >> "$tmp_file"
        tail -n +$((line_num + 1)) "$target_file" >> "$tmp_file"
        mv "$tmp_file" "$target_file"
    }
    robust_multiline_replace "$PROJECT_NAME/package.xml" "%(depends)s" "$pkg_xml_deps_str"

else
    sed -i "/%(find_packages)s/d" "$PROJECT_NAME/CMakeLists.txt"
    sed -i "/%(include_directories)s/d" "$PROJECT_NAME/CMakeLists.txt"
    sed -i "/%(link_libraries)s/d" "$PROJECT_NAME/CMakeLists.txt"
    sed -i "/%(depends)s/d" "$PROJECT_NAME/package.xml"
fi

echo -e "${GREEN}패키지 '$PROJECT_NAME' 생성이 완료되었습니다.${NC}"

echo ""
echo "다음 명령어로 빌드할 수 있습니다:"
echo "cd ../.."
echo "colcon build --packages-select $PROJECT_NAME"