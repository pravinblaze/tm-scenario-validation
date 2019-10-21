import carla
import py_trees

from utils.atomic_behaviours import SetVelocity
from utils.atomic_behaviours import DriveDistance
from utils.atomic_behaviours import LaneChange
from utils.atomic_behaviours import StopVehicle
from utils.atomic_behaviours import TriggerDistanceToVehicle
from scenarios.basic_scenario import BasicScenario


class LaneCutIn(BasicScenario):
    def __init__(self, world, name, traffic_manager, ego_vehicle, other_vehicles):
        super(LaneCutIn, self).__init__(
            world, name, traffic_manager, ego_vehicle, other_vehicles
        )
        self._traffic_vehicle = other_vehicles[0]
        print("Instantiated LaneCutIn")
        self._setup()

    def _setup(self):
        # self._traffic_manager.set_auto_lane_change(self._ego_vehicle, False)
        self._traffic_manager.set_collision_detection(self._traffic_vehicle, self._ego_vehicle, False)

    def create_tree(self):

        tv_speed = 70.0/3.6
        ego_speed = 60.0/3.6
        ego_drive_distance = 300

        scenario_sequence = py_trees.composites.Sequence()
        scenario_sequence.add_child(SetVelocity(self._traffic_manager, self._ego_vehicle, ego_speed))
        scenario_sequence.add_child(SetVelocity(self._traffic_manager, self._traffic_vehicle, tv_speed))

        drive_past = py_trees.composites.Sequence()
        drive_past.add_child(
            TriggerDistanceToVehicle(
                self._ego_vehicle, self._traffic_vehicle,
                5, comparision=1
            )
        )
        drive_past.add_child(
            TriggerDistanceToVehicle(
                self._ego_vehicle, self._traffic_vehicle,
                5, comparision=-1
            )
        )

        scenario_sequence.add_child(drive_past)
        scenario_sequence.add_child(LaneChange(self._traffic_manager, self._traffic_vehicle, True))
        scenario_sequence.add_child(DriveDistance(self._ego_vehicle, ego_drive_distance))

        return scenario_sequence
