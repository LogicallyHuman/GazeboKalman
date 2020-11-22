export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(readlink -f models)
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(readlink -f plugins/car_plugin/build)
gazebo kalman.world --verbose 
