# LidarProject

Driver 다운로드
https://cn.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

라이더 툴 다운
https://www.ydlidar.com/service_support/download.html

# 파트라슈 프로젝트 라이더 센서

# WSL 설정
windows 기능 켜기/끄기   
Linux용 Windows 하위 시스템 체크

WSL 패키지 설치
https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi

```bash
# CMD 관리자권한 실행 후 재부팅
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```

```bash
# 재부팅후 wsl 초기버전 1 설정
wsl --set-default-version 1
```
# Ubuntu 18.04.5 LTS 윈도우스토어에서 설치
윈도우 스토어에서 Ubuntu 18.04.5 LTS 설치

아이디 비밀번호 설정후

```bash
sudo apt install git
sudo apt install cmake pkg-config

sudo apt-get install swig
sudo apt-get install python swig
sudo apt install python-pip
sudo apt-get install python-pip

# Cmake 의존 라이브러리 설치 (make , gcc, gcc-c++ openssl, openssl-devel)
sudo apt-get update
sudo apt-get install make
sudo apt-get install gcc 
sudo apt-get install g++
sudo apt-get install libssl-dev openssl
sudo apt-get install libssl-dev


# Cmake 설치
sudo wget https://cmake.org/files/v3.15/cmake-3.15.2.tar.gz
sudo tar xvfz cmake-3.15.2.tar.gz
cd cmake-3.15.2
sudo ./bootstrap
sudo make
sudo make install

# YDLidar-SDK설치
sudo git clone https://github.com/YDLIDAR/YDLidar-SDK.git
sudo mkdir YDLidar-SDK/build
cd YDLidar-SDK/build
sudo cmake ..
sudo make
sudo make install

cd YDLidar-SDK
sudo pip install .
sudo python3 setup.py build
sudo python3 setup.py install
```
