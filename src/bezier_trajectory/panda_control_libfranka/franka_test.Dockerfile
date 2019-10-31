FROM git.isw.uni-stuttgart.de:5000/projekte/forschung/2017_dfg_irtg_softtissuerobotics/panda_control_libfranka/libfranka:0.6.0
WORKDIR /example_franka_control_project
COPY . .
RUN mkdir -p build && cd build && \
    cmake .. && make -j4

