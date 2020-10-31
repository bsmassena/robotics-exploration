FROM ubuntu:18.04

RUN apt-get -y update
RUN apt-get install -y apt-utils make g++ unzip freeglut3-dev libfreeimage-dev 

COPY dependencies /dependencies
WORKDIR /dependencies

# Installing ARIA

RUN unzip Aria-2.7.2-ubuntu18.04.zip
RUN cd Aria-2.7.2-mod && make && make install

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/Aria/lib
RUN ldconfig

# Installing MobileSim

RUN apt-get install -y ./mobilesim_0.7.3+ubuntu12+gcc4.6_amd64.deb

WORKDIR /phir2framework
