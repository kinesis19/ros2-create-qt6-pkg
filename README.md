# ros2-create-qt6-pkg

ROS 2 Humble 이상 환경에서 Qt6 GUI 애플리케이션 패키지의 기본 뼈대를 자동으로 생성해주는 간단한 셸 스크립트입니다.

## 🌟 주요 기능

* 명령어 하나로 C++ 기반의 ROS 2 Qt6 패키지 기본 구조 생성
* `CMakeLists.txt`, `package.xml` 등 필수 파일 자동 설정
* ROS 2 통신을 위한 `qnode` 클래스 기본 템플릿 포함
* 아이콘 리소스(`.qrc`) 및 `.ui` 파일 관리 구조 포함
* 패키지 생성 시 의존성을 대화형으로 간편하게 추가 가능

## 💻 요구사항

* Ubuntu 22.04 이상
* ROS 2 Humble Hawksbill 이상
* Qt6 관련 개발 라이브러리
    ```bash
    sudo apt-get update && sudo apt-get install qt6-base-dev qt6-base-private-dev
    ```

## 🚀 설치 방법

1.  **저장소 복제(Clone)**
    원하는 위치에 이 저장소를 복제합니다.
    ```bash
    git clone https://github.com/kinesis19/ros2-create-qt6-pkg.git
    cd ros2-create-qt6-pkg
    ```

2.  **설치 스크립트 실행**
    설치 스크립트에 실행 권한을 부여하고 실행합니다. 이 과정은 시스템의 모든 위치에서 `ros2_create_qt6_pkg` 명령어를 사용할 수 있도록 설정합니다.
    ```bash
    chmod +x install.sh
    ./install.sh
    ```

3.  **터미널 환경 적용**
    설치 완료 후, 새 터미널을 열거나 아래 명령어를 실행하여 변경사항을 적용합니다.
    ```bash
    source ~/.bashrc
    ```

## ▶️ 사용 방법

1.  생성하고 싶은 ROS 2 워크스페이스의 `src` 폴더로 이동합니다.
    ```bash
    cd ~/ros2_ws/src
    ```

2.  `ros2_create_qt6_pkg` 명령어를 `<패키지_이름>`과 함께 실행합니다.
    ```bash
    ros2_create_qt6_pkg my_awesome_qt_app
    ```

3.  스크립트의 안내에 따라 추가할 의존성을 입력합니다. (예: `rclcpp std_msgs`). 의존성이 없다면 그냥 Enter를 누릅니다.

## 📁 생성되는 패키지 구조

명령어를 실행하면 아래와 같은 구조의 패키지가 생성됩니다.
```
my_awesome_qt_app/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── my_awesome_qt_app/
│       ├── mainwindow.h
│       └── qnode.hpp
├── src/
│   ├── main.cpp
│   ├── mainwindow.cpp
│   └── qnode.cpp
├── ui/
│   └── mainwindow.ui
└── resources/
    ├── images.qrc
    └── images/
        └── icon.png
```

## 🛠️ 생성된 패키지 빌드 및 실행

1.  워크스페이스의 루트로 이동합니다.
    ```bash
    cd ~/ros2_ws
    ```
2.  `colcon`으로 생성된 패키지를 빌드합니다.
    ```bash
    colcon build --packages-select my_awesome_qt_app
    ```
3.  워크스페이스 환경을 소싱(source)합니다.
    ```bash
    source install/setup.bash
    ```
4.  `ros2 run`으로 GUI 애플리케이션을 실행합니다.
    ```bash
    ros2 run my_awesome_qt_app my_awesome_qt_app
    ```

## 📜 라이선스

이 프로젝트는 MIT 라이선스를 따릅니다.