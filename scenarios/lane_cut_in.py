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

    def create_tree(self):

        tv_speed = 30
        ego_speed = 20
        ego_drive_distance = 600

        tv_seq = py_trees.composites.Sequence()

        drive_tv = py_trees.composites.Parallel(name="root",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne)
        drive_tv.add_child(
            SetVelocity(self._traffic_manager, self._traffic_vehicle, tv_speed))

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
                2, comparision=-1
            )
        )

        drive_tv.add_child(drive_past)

        tv_seq.add_child(drive_tv)
        tv_seq.add_child(LaneChange(self._traffic_manager, self._traffic_vehicle, False))

        tv_seq.add_child(
            SetVelocity(self._traffic_manager, self._traffic_vehicle, tv_speed)
        )

        drive_ev = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SuccessOnOne
        )
        follow_lane_ev = SetVelocity(self._traffic_manager, self._ego_vehicle, ego_speed)

        drive_distance_ev = DriveDistance(
            self._ego_vehicle, ego_drive_distance
        )
        drive_ev.add_child(follow_lane_ev)
        drive_ev.add_child(drive_distance_ev)

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SuccessOnOne
        )
        root.add_child(tv_seq)
        root.add_child(drive_ev)

        return root
