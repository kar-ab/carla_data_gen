# CARLA_training_data_gen


## Project description
	This project is intended to use training data for Autonomous Driving reserach using open source simulator i.e. Carla Simulator for simulation. 
    In this project, an ego vehicle (vehicle.audi.tt) and other actors (vehicles / wlaking persons) are added to the simualtion. As the There are basically the ego vehicle and other actors in the simulator scene. Sensors are attached to the ego vehicle and as the ego vehicle travels around in the environment, data from sensors are synchronously saved in folders. 
    
Following is the highlights of the work provided: 
    
1. Attach sensors easily using a yaml file: Refer [stereo_vision.yaml](https://github.com/kar-ab/CARLA_training_data_gen/blob/main/stereo_vision.yaml) and [sensors.yaml](https://github.com/kar-ab/CARLA_training_data_gen/blob/main/sensors.yaml)
2. Gather and save data from sensors and then the ego vehicle proceeds to next waypoint
3. Ego vehicle movements is generated using the BehaviroAgent. You can configure the ego vehicle to act upon and tweak with the car control movements as per the incoming data.


## Run:

1. Install [Carla Simulator](https://carla.org/2020/04/22/release-0.9.9/) .This work has been implemented on Carla v0.9.9 .
2. Download this github repository in you folder
3. Run `record_data_sync.py` script

To connect all types of sensor available in carla:
    
    `$ python3 record_data_sync.py --yaml=sensors.yaml`

### OR

2. Stereo vision setup for ego vehicle:
    
    `$ python3 record_data_sync.py --yaml=stereo_vision.yaml`


## Note: 

1. After running above, it will take around 10-15 seconds for data to generate and the ego vehicle to move



## TODO list

1. Extract 2D and 3D bounding boxes for respective cameras
2. Integrating openscenario to add scenarios
3. Convert sensor data in KITTI / Cityscapes dataset format


## Credits: 

[Carla Simulator Team](https://carla.org/)
