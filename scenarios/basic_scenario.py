
class BasicScenario (object):
    def __init__(self, world, name, traffic_manager, ego_vehicle, other_vehicles):
        self._world = world
        self._name = name
        self._traffic_manager = traffic_manager
        self._ego_vehicle = ego_vehicle
        self._other_vehicles = other_vehicles

    def setup(self):
        pass

    def create_tree(self):
        pass
