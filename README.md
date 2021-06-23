# carla_data_gen


## Project description

This project is intended to generate training data for Autonomous Driving reserach using open source simulator i.e. Carla Simulator. 
In this project, an ego vehicle (vehicle.audi.tt) and other actors (vehicles / wlaking persons) are added to the simualtion. Sensors are attached to the ego vehicle and as the ego vehicle travels around in the environment, data from sensors are synchronously saved in respective folders. 
    
Following are the highlights of this project: 
    
1. Attach sensors easily using a yaml file: Sample yaml files are provided for reference: 
    
    a. [stereo_vision.yaml](../blob/main/stereo_vision.yaml) 
        
    Two cameras, i.e. left and right are attached to vehicle. At the same position semantic segmentation and depth cameras are also attached to generate ground truth data for training. A sample of sensor data can be viewed [here](..blob/main/episodes/2021-05-14_00-06-47)
        
    b. [sensors.yaml](../blob/main/sensors.yaml)
    
    All types of sensor which can be attached to a vehicle in a Carla Simulator. A sample of sensor data can be viewed [here](..blob/main/episodes/2021-05-14_00-08-23)
    
3. Gather and save data synchronously from sensors and then the ego vehicle proceeds to next waypoint
4. By default, Ego vehicle movements are generated using the [BehaviorAgent](../blob/main/configure_agents/navigation/behavior_agent.py).
5. The same repository can be modified to work in online mode also, by just tweaking the Motion planning and Vehicle Control logic as per the incoming sensor data. 

## Setup:

1. Install [Carla Simulator](https://carla.org/2020/04/22/release-0.9.9/) .This work has been implemented on Carla v0.9.9 .
2. `git clone https://github.com/kar-ab/carla_data_gen`
3. Run script record_sync_data.py with the respective yaml file

    `$ python3 record_data_sync.py --yaml=sensors.yaml`
    
    OR     
    
    `$ python3 record_data_sync.py --yaml=stereo_vision.yaml`


## TODO list

1. IT takes 10-15 seconds for ego vehicle to move. possible solution - Maybe initial frames are being used to plan the vehicle path, need to start only sync only when vehicle has moved form its original location. 
2. Extract 2D and 3D bounding boxes for respective cameras
3. Integrating openscenario to add scenarios
4. Convert sensor data in KITTI / Cityscapes dataset format


## Credits: 

[Carla Simulator Team](https://carla.org/)
