import math
import time
import random

import numpy as np
import carla
from carla import ColorConverter as cc
import pygame
import py_trees

from scenarios.lane_cut_in import LaneCutIn
from scenarios.test_tm_collision import TestTMCollision
from scenarios.sun_glare_junction_demo import SunGlareJunction
from scenarios.distance_to_leading_vehicle import DistanceToLeadingVehicle

ip = "localhost"
port = 2000
display_scale = 0.9 # x1080p
display_x = int(1920*display_scale)
display_y = int(1080*display_scale)

client = carla.Client(ip, port)
world = client.get_world()
world_map = world.get_map()

pygame.font.init()
myfont = pygame.font.SysFont('Comic Sans MS', 30)

tm = None
vehicle = None
traffic_vehicle = None
camera_sensor = None


class surface_holder:
    def __init__(self):
        self.surface = None
camera_holder = surface_holder()

town_05_lane_cut_in = {
    "location1": {
        "ego_vehicle": {
            "model": "vehicle.lincoln.mkz2017",
            "position": [78.1, 202.0, 5]
        },
        "traffic_vehicle": {
            "model": "vehicle.lincoln.mkz2017",
            "position": [48.8, 205.3, 5]
        }
    }
}

town_05_tm_collision = {
    "location1": {
        "ego_vehicle": {
            "model": "vehicle.lincoln.mkz2017",
            "position": [-51.5, -74.3, 5]
        },
        "traffic_vehicle": {
            "model": "walker.pedestrian.0001",
            "position": [-51.1, -16.7, 5]
        }
    }
}

town_03_sun_glare = {
    "location1": {
        "ego_vehicle": {
            "model": "vehicle.tesla.model3",
            "position": [19.0, -134.4, 0.5]
        },
        "traffic_vehicle": {
            "model": "vehicle.tesla.model3",
            "position": [83.7, -86.7, 8.5]
        }
    }
}

town_05_distance_to_leading_vehicle = {
    "location1": {
        "ego_vehicle": {
            "model": "vehicle.lincoln.mkz2017",
            "position": [-128.5, -75.0, 0.5]
        },
        "traffic_vehicle": {
            "model": "vehicle.lincoln.mkz2017",
            "position": [-128.5, -50.0, 0.5]
        }
    }
}

scenario_data = town_05_distance_to_leading_vehicle

def spawn_vehicle(data):
    model = data["model"]
    loc = data["position"]
    location = carla.Location(loc[0], loc[1], loc[2])
    wp = world_map.get_waypoint(location)
    transform = carla.Transform(location, wp.transform.rotation)
    blueprint = random.choice(
        world.get_blueprint_library().filter(model))
    actor = None
    while actor is None:
        actor = world.try_spawn_actor(blueprint, transform)
        if actor is None:
            print("Actor is still none !")
        print("Trying to spawn vehicle with blueprint : ", blueprint)
    print("Vehicle id : ", actor.id, " spawned")
    return actor


def print_tree(tree):
    # print(py_trees.display.print_ascii_tree(root=tree.root, show_status=True))
    pass


def magnitude(vector):
    return math.sqrt(vector.x**2+vector.y**2+vector.z**2)

try:

    traffic_manager = carla.GetTrafficManager(client)
    tm = traffic_manager

    # Spawn vehicle
    vehicle = spawn_vehicle(
        scenario_data["location1"]["ego_vehicle"]
    )
    traffic_vehicle = spawn_vehicle(
        scenario_data["location1"]["traffic_vehicle"]
    )
    if vehicle is None or traffic_vehicle is None:
        print("Couldn't spawn vehicles!")
        exit()

    # Register vehicles with traffic manager
    vehicle_vec = carla.TM_ActorList()
    vehicle_vec.extend([vehicle, traffic_vehicle])

    traffic_manager.register_vehicles(vehicle_vec)
    print("Registered vehicles with TM")

    # Create behaviour tree

    # scenario = LaneCutIn(world, "LaneCutInDemo", traffic_manager, vehicle, [traffic_vehicle])
    # scenario = TestTMCollision(world, "TestTMCollision", traffic_manager, vehicle, [traffic_vehicle])
    # scenario = SunGlareJunction(world, "SunGlareJunction", traffic_manager, vehicle, [traffic_vehicle])
    scenario = DistanceToLeadingVehicle(world, "DistanceToLeadingVehicle", traffic_manager, vehicle, [traffic_vehicle])

    root = scenario.create_tree()
    behaviour_tree = py_trees.trees.BehaviourTree(root=root)
    behaviour_tree.tick()
    print("Started behaviour tree!")

    # Set sensor

    bp_library = world.get_blueprint_library()
    bp = bp_library.find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', str(display_x))
    bp.set_attribute('image_size_y', str(display_y))

    camera_transform = carla.Transform(carla.Location(x=0.0, y=-0.4, z=1.2), carla.Rotation(pitch=-5))
    camera_sensor = world.spawn_actor(
        bp,
        camera_transform,
        attach_to=vehicle)

    # Camera callback funtion

    def parse_image(image):
        image.convert(cc.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        camera_holder.surface = surface

    camera_sensor.listen(lambda image: parse_image(image))

    print("Attached sensors")

    # Project camera image on pygame

    display = pygame.display.set_mode((display_x, display_y), pygame.HWSURFACE | pygame.DOUBLEBUF)

    clock = pygame.time.Clock()
    behaviour_tree.tick()

    print("Running behaviour tree")
    while behaviour_tree.root.status == py_trees.common.Status.RUNNING:
        # clock.tick_busy_loop(100)
        while camera_holder.surface is None:
            # print 'waiting for surface'
            pass
        behaviour_tree.tick(post_tick_handler=print_tree)
        display.blit(camera_holder.surface, (0, 0))
        ev_velocity = vehicle.get_velocity()
        tv_velocity = traffic_vehicle.get_velocity()
        textsurface = myfont.render(
            "EV Speed: " +
            str(round(magnitude(ev_velocity)*3.6)) +
            " TV Speed:" +
            str(round(magnitude(tv_velocity)*3.6)),
            True, (255, 255, 255)
        )
        display.blit(textsurface, (display_x*0.1, display_y*0.9))
        pygame.display.flip()

except Exception as e:
    print("Encountered an exception!")
    print(e)

finally:
    if vehicle is not None:
        vehicle.destroy()
    if traffic_vehicle is not None:
        traffic_vehicle.destroy()
    if camera_sensor is not None:
        camera_sensor.destroy()
    if tm is not None:
        tm.stop()
