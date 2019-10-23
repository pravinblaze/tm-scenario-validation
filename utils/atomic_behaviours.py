import math

import carla
import py_trees
import time

# ======================= Actions ============================= #


class SetVelocity(py_trees.behaviour.Behaviour):
    def __init__(self, traffic_manager, vehicle, target_velocity, name="FollowLane"):
        super(SetVelocity, self).__init__(name)
        self._vehicle = vehicle
        self._target_velocity = target_velocity
        self._tm = traffic_manager

    def update(self):
        self._tm.set_vehicle_target_velocity(self._vehicle, self._target_velocity)
        return py_trees.common.Status.SUCCESS


class StopVehicle(py_trees.behaviour.Behaviour):
    def __init__(self, traffic_manager, vehicle, name="StopVehicle"):
        super(StopVehicle, self).__init__(name)
        self._vehicle = vehicle
        self._tm = traffic_manager

    def update(self):
        self._tm.set_vehicle_target_velocity(self._vehicle, 0.0)
        return py_trees.common.Status.SUCCESS


class LaneChange(py_trees.behaviour.Behaviour):
    def __init__(self, traffic_manager, vehicle, direction, name="LaneChange"):
        super(LaneChange, self).__init__(name)
        self._vehicle = vehicle
        self._tm = traffic_manager
        self._direction = direction

    def update(self):
        self._tm.force_lane_change(self._vehicle, self._direction)
        return py_trees.common.Status.SUCCESS


# ======================= Checks ============================= #


class DriveDistance(py_trees.behaviour.Behaviour):
    def __init__(self, vehicle, distance, name="DriveDistance"):
        super(DriveDistance, self).__init__(name)
        self._vehicle = vehicle
        self._distance = distance
        self._initial_location = None

    def setup(self):
        self._initial_location = self._vehicle.get_location()

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if self._initial_location is not None:
            current_distance = self._vehicle.get_location().distance(self._initial_location)
            if current_distance > self._distance:
                new_status = py_trees.common.Status.SUCCESS
        else:
            self.setup()
        return new_status


class TriggerDistanceToVehicle(py_trees.behaviour.Behaviour):
    def __init__(
        self, reference_vehicle, other_vehicle,
        distance, comparision=1, name="TriggerDistanceToVehicle"
    ):
        super(TriggerDistanceToVehicle, self).__init__(name)
        self._reference_vehicle = reference_vehicle
        self._other_vehicle = other_vehicle
        self._distance = distance
        self._comparision = comparision

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if self._comparision == 1 and \
                self._reference_vehicle.get_location().distance(
                    self._other_vehicle.get_location()
                ) < self._distance:
            new_status = py_trees.common.Status.SUCCESS
            print("Approached !")
        elif self._comparision == -1 and \
                self._reference_vehicle.get_location().distance(
                    self._other_vehicle.get_location()
                ) > self._distance:
            new_status = py_trees.common.Status.SUCCESS
        return new_status


class TriggerAfterDuration(py_trees.behaviour.Behaviour):
    def __init__(self, duration, name="TriggerAfterDuration"):
        super(TriggerAfterDuration, self).__init__(name)
        self._duration = duration
        self._initial_time = None

    def setup(self):
        self._initial_time = time.time()

    def update(self):
        new_status = py_trees.common.Status.RUNNING
        if self._initial_time is not None:
            current_time = time.time()
            if (current_time - self._initial_time) > self._duration:
                new_status = py_trees.common.Status.SUCCESS
        else:
            self.setup()
        return new_status