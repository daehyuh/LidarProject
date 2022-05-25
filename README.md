# LidarProject

# 파트라슈 프로젝트 라이더 센서

# Driver 다운로드
https://cn.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

# 라이더 툴 다운
https://www.ydlidar.com/service_support/download.html

# wsl 우분투
https://webdir.tistory.com/541
오류뜰때
https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi

초기비밀번호 
ubuntu
ubuntu


// sudo apt install cmake 는 카카오 저장소에서 없어져서 공식사이트에서 다운받아줘야한다

# 필요한 CMAKE설치를 위한 c컴파일러
sudo add-apt-repository ppa:jonathonf/gcc-7.1   
sudo apt-get update   
sudo apt-get install gcc-7 g++-7   
g++ --version   

# 컴파일러가 우선 실행하게 하기위해
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 60 --slave /usr/bin/g++ g++ /usr/bin/g++-7   


# CMAKE 설치
wget https://github.com/Kitware/CMake/releases/download/v3.18.2/cmake-3.18.2.tar.gz   
tar -xvf cmake-3.18.2.tar.gz 
cd cmake-3.18.2   
./bootstrap 
make
cmake ..
cmake --version

#cmake 설치
sudo apt install cmake

원하는 디텍토리에 git 클론
git clone https://github.com/YDLIDAR/YDLidar-SDK.git

sudo chown -R <계정명> <작업폴더>

cd YDLidar-SDK/build
mkdir build
build 폴더는 만들어줘야함

cmake ..
make


cd YDLidar-SDK
pip install .

# Another method
python3 setup.py build
python3 setup.py install
