This file holds the world and model files for the Multi Drone Sim, as well as scripts to control them.

Run gazebo sim using: gz sim -v4 -r multiDrone.sdf

Run Drone One SITL using: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14550

Run Drone Two SITL using: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14551 -I1

Run Drone Three SITL using: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14552 -I2

Run Drone Four SITL using: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14553 -I3

Run Drone Five SITL using: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14554 -I4

Run Drone Six SITL using: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=127.0.0.1:14555 -I5

