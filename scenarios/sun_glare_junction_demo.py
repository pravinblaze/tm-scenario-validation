import carla
import py_trees

from utils.atomic_behaviours import SetVelocity
from utils.atomic_behaviours import TriggerAfterDuration
from scenarios.basic_scenario import BasicScenario


class SunGlareJunction(BasicScenario):
    def __init__(self, world, name, traffic_manager, ego_vehicle, other_vehicles):
        super(SunGlareJunction, self).__init__(
            world, name, traffic_manager, ego_vehicle, other_vehicles
        )
        self._traffic_vehicle = other_vehicles[0]
        self._setup()

    def _setup(self):
        wp = carla.WeatherParameters(0, 0, 100, 0, 180, 10)
        self._world.set_weather(wp)

    def create_tree(self):

        urban_speed = 20/3.6

        scenario_sequence = py_trees.composites.Sequence()
        scenario_sequence.add_child(SetVelocity(self._traffic_manager, self._ego_vehicle, urban_speed))
        scenario_sequence.add_child(SetVelocity(self._traffic_manager, self._traffic_vehicle, urban_speed))
        scenario_sequence.add_child(TriggerAfterDuration(60.0))

        return scenario_sequence
