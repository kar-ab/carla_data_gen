from __future__ import print_function

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

import glob
import os
import sys
try:
    sys.path.append(glob.glob('../exercises/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
from queue import Queue
import carla
from carla import ColorConverter as cc
import yaml



class World(object):
    def __init__(self, carla_world, vehicle):
        self.world = carla_world
        self.player = vehicle


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors.
    Synchronous mode is enabled as long as we are inside this
    context with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)
    """

    def __init__(self, world, sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None


    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(
            carla.WorldSettings(
                no_rendering_mode=False,
                synchronous_mode=True,
                fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout, world, agent, vehicle):

        # move the vehicle to next waypoint using the
        # agent.update_information(world, vehicle)
        agent.update_information(World(world, vehicle))
        self.frame = self.world.tick()
        speed_limit = vehicle.get_speed_limit()
        agent.get_local_planner().set_speed(speed_limit)
        control = agent.run_step()
        vehicle.apply_control(control)

        data = [self._retrieve_data(q, timeout) \
                for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data



def draw_image(surface, image, blend=False):
    array = np.frombuffer(image.raw_data,
                          dtype=np.dtype("uint8"))
    array = np.reshape(array,
                       (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(
        array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))



class SensorManager(object):
    def __init__(self, world):
        self.file = None
        self.world = world
        self.parent = None
        self.attached = []
        self.mini_view = []
        self.unattached = []
        self.bp = None
        self.transform =None
        self.instance = None
        self.converter_list = []

    def bp_attr(self):
        if 'Setting' in self.instance:
            for setting, value in self.instance['Setting'].items():
                self.bp.set_attribute(setting, str(value))


    def get_transform(self):
        # spawning: Location
        location = carla.Location()
        if self.check_key(self.instance['Spawning'], 'Location'):
            if self.check_key(self.instance['Spawning']['Location'], 'x'):
                location.x = self.instance['Spawning']['Location']['x']
            if self.check_key(self.instance['Spawning']['Location'], 'y'):
                location.y = self.instance['Spawning']['Location']['y']
            if self.check_key(self.instance['Spawning']['Location'], 'z'):
                location.z = self.instance['Spawning']['Location']['z']

        # spawning: Rotation
        rotation = carla.Rotation()
        if self.check_key(self.instance['Spawning'], 'Rotation'):
            if self.check_key(self.instance['Spawning']['Rotation'], 'roll'):
                rotation.roll = self.instance['Spawning']['Rotation']['roll']
            if self.check_key(self.instance['Spawning']['Rotation'], 'pitch'):
                rotation.pitch = self.instance['Spawning']['Rotation']['pitch']
            if self.check_key(self.instance['Spawning']['Rotation'], 'yaw'):
                rotation.yaw = self.instance['Spawning']['Rotation']['yaw']

        # spawning: transform
        self.transform = carla.Transform(location,rotation)

        return self.transform


    def get_color_converter(self):
        if self.check_key(self.instance, 'Listening'):
            if self.check_key(self.instance['Listening'], 'colorConverter'):
                col_value = self.instance['Listening']['colorConverter']
                if col_value == "CityScapesPalette":
                    return cc.CityScapesPalette
                elif col_value == "Raw":
                    return cc.Raw
                elif col_value == "LogarithmicDepth":
                    return cc.LogarithmicDepth
                elif col_value == "Depth":
                    return cc.Depth
            else:
                return None
        else:
            return None


    def check_key(self, event, key):
        if key in event and event[key] is not None:
            return True
        else:
            return False


    def attach_sensor(self):
        '''
        Attach sensor to ego vehicle
        '''
        if self.check_key(self.instance['Spawning'], 'attach_to'):
            #  with attachment type
            if self.check_key(self.instance['Spawning'], 'attachment_type'):
                if self.instance['Spawning']['attachment_type'] == 'SpringArm':
                        attachtype = carla.AttachmentType.SpringArm
                else:
                        attachtype = carla.AttachmentType.Rigid
                # attaching sensor with its type
                sensor_attached = self.world.spawn_actor(self.bp,
                                                        self.transform,
                                                        attach_to = self.parent,
                                                        attachment_type= attachtype)

            # without attachment
            else:
                sensor_attached = self.world.spawn_actor(self.bp,
                                                    self.transform,
                                                    attach_to = self.parent)

            self.attached.append(sensor_attached)

        else: # cannot attach
            self.unattached.append(self.instance)


    def parse_and_attach(self, blueprint_library, file, parent):
        '''
        parsing each sensor dictionary in yaml and attaching it to ego vehicle
        '''
        cc_list = []
        self.parent = parent
        parsed = yaml.load(file, Loader=yaml.FullLoader)
        data_dir = parsed["data_dir"]
        sensors = (x for x in parsed.keys() if x not in ( 'mini_view', 'data_dir') )

        for sensor in sensors:
            self.instance = parsed[sensor]
            self.bp = blueprint_library.find(self.instance['blueprint'])
            self.bp.set_attribute('role_name', str(sensor))

            # setting: attributes to sensor
            self.bp_attr()

            # attach to parent & create data path for each sensor
            self.get_transform()
            self.attach_sensor()
            cc_list.append(self.get_color_converter())

        return self.attached, cc_list, self.unattached, data_dir
