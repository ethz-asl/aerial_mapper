FROM ethzasl/aerial_mapper:base-latest
ENV HOME=/root

#Build Dependencies
COPY . $HOME/catkin_ws/src/aerial_mapper/
RUN catkin build -j$(nproc) -l$(nproc) aerial_mapper -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
