# CARLA_training_data_gen

## An efficient way to generate training data for AUtonomous Driving tasks


All the description about sensors in a carla simulator can be added into yaml file, 
1. vechicles and walkers are added to world
2. sensors are attached to the ego vehcile 
3. data from all the sensors are recorded in sync and saved in respective folders

This work has been implemented on Carla v0.9.9.

To execute run command: 

Just download this repository in tht PythonAPI/example of your Carla simulator setup
and execute command 

    $ python3 record_data_sync.py


## Use cases:

1. To connect all types of sensor available in carla:
    
    $ python3 record_data_sync.py --yaml=sensors.yaml

2. Stereo vision setup for ego vehicle:
    
    $ python3 record_data_sync.py --yaml=stereo_vision.yaml
