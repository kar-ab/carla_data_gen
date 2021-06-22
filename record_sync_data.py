from __future__ import print_function



# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import argparse
from datetime import datetime
import logging
import random
import numpy as np

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    import carla
except ImportError:
    raise RuntimeError('cannot import carla, please check path of carl library')

import utils
from configure_agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from configure_agents.navigation.roaming_agent import RoamingAgent  # pylint: disable=import-error
from configure_agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error


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


def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():

    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '--yaml',
        metavar='NAME',
        default='stereo_vision.yaml',
        help='actor role name (default: "sensors.yaml")')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=50,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '-tm_p', '--tm_port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '-l', '--loop',
        action='store_true',
        dest='loop',
        help='Sets a new random destination upon reaching the previous one (default: False)')

    args = argparser.parse_args()

    try:

        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)
        actor_list = []
        sensors = []
        sensors_folder = []
        pygame.init()

        display = pygame.display.set_mode(
                    (800, 600),
                    pygame.HWSURFACE | pygame.DOUBLEBUF)
        font = get_font()
        clock = pygame.time.Clock()

        world = client.get_world()

        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(2.0)
        world = client.get_world()


        print('\nRUNNING in synchronous mode\n')
        settings = world.get_settings()
        traffic_manager.set_synchronous_mode(True)
        if not settings.synchronous_mode:
            synchronous_master = True
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.01
            world.apply_settings(settings)
        else:
            synchronous_master = False

        blueprints = world.get_blueprint_library().filter('vehicle.*')

        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
        blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
        blueprints = [x for x in blueprints if not x.id.endswith('t2')]

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if args.number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif args.number_of_vehicles > number_of_spawn_points:
            msg = 'Requested %d vehicles, but could only find %d spawn points'
            logging.warning(msg, args.number_of_vehicles, number_of_spawn_points)
            args.number_of_vehicles = number_of_spawn_points

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor

        # --------------
        # Spawn vehicles
        # --------------
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')
            batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))
            spawn_points.pop(0)

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                actor_list.append(response.actor_id)

        print('\nCreated %d npc vehicles \n' % len(actor_list))



        # -----------------------------
        # Spawn ego vehicle and sensors
        # -----------------------------


        m = world.get_map()
        start_pose = random.choice(m.get_spawn_points())
        waypoint = m.get_waypoint(start_pose.location)

        blueprint_library = world.get_blueprint_library()

        # adding vehicle
        car_bp = random.choice(blueprints)
        car_bp.set_attribute('role_name', str("hero"))

        vehicle = world.spawn_actor(car_bp, start_pose)
        vehicle.set_simulate_physics(False)
        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)
        vehicle.set_autopilot(True)


        # loading yaml file
        sensor_manger = utils.SensorManager(world)
        sensors, cc_list, unattached, data_dir = \
        sensor_manger.parse_and_attach(
            blueprint_library,
            open(args.yaml),
            vehicle)


        #  creating data path folder for each attached sensors
        if (len(sensors) > 0):
            root_folder = os.path.join(
                data_dir,
                datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
            os.makedirs(root_folder)

        print("Following sensors have from yaml are attached: ")
        for sensor in sensors:
            create_folder = root_folder + "/" +\
                            sensor.attributes["role_name"]
            os.makedirs(create_folder)
            sensors_folder.append(create_folder)
            print(sensor.attributes["role_name"])

        if( len(unattached) > 0):
            print("Below sensors from yaml are not attached: ", unattached)


        # additional sensors to draw on pygame window
        camera_rgb = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            carla.Transform(
                carla.Location(x=-5.5, z=2.8),
                carla.Rotation(pitch=-15)),
                attach_to=vehicle)
        sensors.append(camera_rgb)

        camera_semseg = world.spawn_actor(
            blueprint_library.find('sensor.camera.semantic_segmentation'),
            carla.Transform(
                carla.Location(x=-5.5, z=2.8),
                carla.Rotation(pitch=-15)),
                attach_to=vehicle)
        sensors.append(camera_semseg)


        # Initializing the agent....
        world.tick()
        agent = BehaviorAgent(vehicle)
        spawn_points = m.get_spawn_points()
        random.shuffle(spawn_points)

        tot_target_reached = 0
        num_min_waypoints = 21
        if spawn_points[0].location != agent.vehicle.get_location():
            destination = spawn_points[0].location
        else:
            destination = spawn_points[1].location

        agent.set_destination(agent.vehicle.get_location(), destination, clean=True)

        # setting red time of all traffic lights to 0.2 seconds instead of default of 1/2 seconds
        # simply to avoid too many images recorded at a traffic light
        for light in world.get_actors().filter("*traffic_light*"):
            light.set_red_time(0.2)


        frame_count = 0 # to dispaly current frame count
        # Create a synchronous mode context.
        sync_context = utils.CarlaSyncMode(world, sensors, fps=30, timeout = 2)
        with sync_context as sync_mode:
            while True:
                if should_quit():
                    return
                clock.tick()
                frame_count += 1

                # Set new destination when target has been reached
                if len(agent.get_local_planner().waypoints_queue) < num_min_waypoints and args.loop:
                    agent.reroute(spawn_points)
                    tot_target_reached += 1
                    world.hud.notification("The target has been reached " +
                                           str(tot_target_reached) + " times.", seconds=4.0)

                elif len(agent.get_local_planner().waypoints_queue) == 0 and not args.loop:
                    print("Target reached, mission accomplished...")
                    break

                data = sync_mode.tick(1, world, agent, vehicle)

                fps = round(1.0 / data[0].timestamp.delta_seconds)
                data[-1].convert(carla.ColorConverter.CityScapesPalette)
                if frame_count >= 40:
                    # TODO: OPEN_ISSUE: not saving first 40 frames as ego vehicke is not moving
                    print(data[0], "; Number of frames saved: " ,frame_count-40, end ="\r")

                    sys.stdout.flush()

                    #  data collection part
                    for n, item in enumerate(data[1:-2]):

                        # for radar only
                        if not hasattr(item, 'save_to_disk'):
                            # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
                            points = np.frombuffer(item.raw_data, dtype=np.dtype('f4'))
                            points = np.reshape(points, (len(item), 4))
                            np.save('%s/%08d' % (sensors_folder[n], sync_mode.frame), points)

                        else:
                          # for camera
                            if (cc_list[n] is not None):
                                item.save_to_disk('%s/%08d' % (sensors_folder[n], sync_mode.frame),
                                cc_list[n])
                          # for lidar
                            else:
                                item.save_to_disk('%s/%08d' % (sensors_folder[n], sync_mode.frame))

                # Draw the display.
                draw_image(display, data[-2])
                draw_image(display, data[-1], blend=True)
                display.blit(
                    font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                    (8, 10))
                display.blit(
                    font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)),
                    (8, 28))
                pygame.display.flip()


    finally:

        print('\ndestroying actors')
        client.apply_batch([carla.command.DestroyActor(y) for y in actor_list])

        if (len(sensors)>0):

            # this is necessary if listen method is added
            print('stopped listening to attached actors')

            for stop_sensor in sensors:
                stop_sensor.stop()
            print('destroying attached sensors')
            client.apply_batch([carla.command.DestroyActor(x) for x in sensors])

        pygame.quit()


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

