import carla
import py_trees

from utils.atomic_behaviours import SetVelocity
from utils.atomic_behaviours import DriveDistance
from utils.atomic_behaviours import LaneChange
from utils.atomic_behaviours import StopVehicle
from utils.atomic_behaviours import TriggerDistanceToVehicle
from utils.atomic_behaviours import TriggerAfterDuration
from scenarios.basic_scenario import BasicScenario


class TestTMCollision(BasicScenario):
    def __init__(self, world, name, traffic_manager, ego_vehicle, other_vehicles):
        super(TestTMCollision, self).__init__(
            world, name, traffic_manager, ego_vehicle, other_vehicles
        )
        self._traffic_vehicle = other_vehicles[0]
        self._setup()

    def _setup(self):
        unregister_list = carla.TM_ActorList()
        unregister_list.extend([self._traffic_vehicle])
        self._traffic_manager.unregister_vehicles(unregister_list)

    def create_tree(self):

        ego_speed = 20/3.6

        scenario_sequence = py_trees.composites.Sequence()
        scenario_sequence.add_child(SetVelocity(self._traffic_manager, self._ego_vehicle, ego_speed))
        scenario_sequence.add_child(TriggerDistanceToVehicle(self._ego_vehicle, self._traffic_vehicle, 10.0, 1))
        scenario_sequence.add_child(TriggerAfterDuration(15.0))

        return scenario_sequence
