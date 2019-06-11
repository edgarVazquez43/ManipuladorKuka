#!/bin/bash
#FORMAT
FRM='\033['
BLACK='0;30'
RED='1;31'
GREEN='1;32'
YELLOW='1;33'
BLUE='1;34'
PURPLE='1;35'
CYAN='1;36'
WHITE='1;37'
BGBLACK=';40m'
BGRED=';41m'
BGGREEN=';42m'
BGYELLOW=';43m'
BGBLUE=';44m'
BGWHITE=';47m'
NC='\033[0m'

echo -e "${FRM}${GREEN}${BGBLUE} -- Installing libfreenect in home directory ${NC}"
cd $HOME
git https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2

echo -e "${FRM}${GREEN}${BGBLUE} -- Installing adittional packages ${NC}"
sudo apt-get install build-essential cmake pkg-config -Y
sudo apt-get install libusb-1.0-0-dev -Y
sudo apt-get install libturbojpeg libjpeg-turbo8-dev -Y
sudo apt-get install libglfw3-dev -Y
sudo apt-get install beignet-dev -Y
sudo apt-get install libva-dev libjpeg-dev -Y
sudo apt-get install libopenni2-dev -Y
echo -e "${FRM}${GREEN}${BGBLUE} -- All dependencies and extra packages have already installed ${NC}"

mkdir build && cd build
cmake .. -DENABLE_CXX11=ON -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
sudo make install
echo -e "${FRM}${GREEN}${BGWHITE} -- INSTALATION [Libfreenect] has already finished ${NC}"
