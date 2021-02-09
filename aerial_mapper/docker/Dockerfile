FROM px4io/px4-dev-ros-melodic:2020-08-14

ENV HOME=/root

RUN apt update && apt install -y python3-wstool libgdal-dev autoconf libtool libtool-bin libcurlpp-dev libcurl4-openssl-dev
RUN mkdir -p $HOME/catkin_ws/src;
WORKDIR $HOME/catkin_ws
RUN catkin init
RUN catkin config --extend "/opt/ros/melodic" && catkin config --merge-devel

#Install Dependencies
COPY . $HOME/catkin_ws/src/aerial_mapper/
RUN wstool init src src/aerial_mapper/install/dependencies_https.rosinstall
RUN wstool update -t src -j4
RUN rosdep update && rosdep install --from-paths src --ignore-src -y --rosdistro melodic
RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN catkin build -j$(nproc) -l$(nproc) aerial_mapper -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
