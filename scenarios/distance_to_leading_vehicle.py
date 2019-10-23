import carla
import py_trees

from utils.atomic_behaviours import SetVelocity
from utils.atomic_behaviours import TriggerAfterDuration
from scenarios.basic_scenario import BasicScenario


class DistanceToLeadingVehicle(BasicScenario):
    def __init__(self, world, name, traffic_manager, ego_vehicle, other_vehicles):
        super(DistanceToLeadingVehicle, self).__init__(
            world, name, traffic_manager, ego_vehicle, other_vehicles
        )
        self._traffic_vehicle = other_vehicles[0]
        self._setup()

    def _setup(self):
        self._traffic_manager.set_auto_lane_change(self._ego_vehicle, False)
        self._traffic_manager.set_distance_to_leading_vehicle(self._ego_vehicle, 10)

    def create_tree(self):

        scenario_sequence = py_trees.composites.Sequence()
        scenario_sequence.add_child(SetVelocity(self._traffic_manager, self._ego_vehicle, 20/3.6))
        scenario_sequence.add_child(SetVelocity(self._traffic_manager, self._traffic_vehicle, 0))
        scenario_sequence.add_child(TriggerAfterDuration(15.0))

        return scenario_sequence
