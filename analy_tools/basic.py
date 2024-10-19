import os

class Header:
    def __init__(self, items) -> None:
        assert len(items) == 3
        self.seq = int(items[0])
        self.stamp = float(items[1])
        self.frame_id = items[2]

    def to_dict(self):
        return {"seq": self.seq, "stamp": self.stamp, "frame_id": self.frame_id}

    @property
    def attr_num(self):
        return 3

class State:
    def __init__(self, items) -> None:
        assert len(items) == 12
        self.header = Header(items[0:3])
        self.stamp = float(items[3])
        self.x = float(items[4])
        self.y = float(items[5])
        self.z = float(items[6])
        self.angle = float(items[7])
        self.curvature = float(items[8])
        self.velocity = float(items[9])
        self.acceleration = float(items[10])
        self.steer = float(items[11])

    def to_dict(self):
        return {"header": self.header.to_dict(),
                "stamp": self.stamp,
                "x": self.x,
                "y": self.y,
                "z": self.z,
                "angle": self.angle,
                "curvature": self.curvature,
                "velocity": self.velocity,
                "acceleration": self.acceleration,
                "steer": self.steer}

    @property
    def attr_num(self):
        return 12

class VehicleParam:
    def __init__(self, items) -> None:
        assert len(items) == 9
        self.width = float(items[0])
        self.length = float(items[1])
        self.wheel_base = float(items[2])
        self.front_suspension = float(items[3])
        self.rear_suspension = float(items[4])
        self.max_steering_angle = float(items[5])
        self.d_cr = float(items[6])
        self.max_longitudinal_acc = float(items[7])
        self.max_lateral_acc = float(items[8])
    
    def to_dict(self):
        return {"width": self.width, "length": self.length, 
                "wheel_base":self.wheel_base, 
                "front_suspension": self.front_suspension,
                "rear_suspension": self.rear_suspension,
                "max_steering_angle": self.max_steering_angle,
                "d_cr": self.d_cr,
                "max_longitudinal_acc": self.max_longitudinal_acc,
                "max_lateral_acc": self.max_lateral_acc}

    @property
    def attr_num(self):
        return 9

class Vehicle:
    def __init__(self, items) -> None:
        assert len(items) == 30
        self.header = Header(items[0:3])
        self.id = int(items[3])
        self.subclass = items[4]
        self.type = items[5]
        self.name = items[6]
        self.param = VehicleParam(items[7:16])
        self.state = State(items[16:28])
        self.ready_change_lane = bool(items[28])
        self.intent = int(items[29])

    def to_dict(self):
        return {"header": self.header.to_dict(),
                "id": self.id,
                "subclass": self.subclass,
                "type": self.type,
                "name": self.name,
                "param": self.param.to_dict(),
                "state": self.state.to_dict(),
                "ready_change_lane": self.ready_change_lane,
                "intent": self.intent}

    @property
    def attr_num(self):
        return 30

class VehicleSet:
    def __init__(self, items) -> None:
        assert (len(items) - 3) % 30 == 0
        self.header = Header(items[0:3])
        start_id = 3
        self.vehicles = []
        while start_id < len(items):
            self.vehicles.append(Vehicle(items[start_id:(start_id+30)]))
            start_id += 30
    
    def to_dict(self):
        return {"header": self.header.to_dict(),
                "vehicles": [v.to_dict() for v in self.vehicles]}

class ArenaInfoDynamic:
    def __init__(self, items) -> None:
        self.time = float(items[0])
        self.header = Header(items[1:4])
        self.vehicle_set = VehicleSet(items[4:])

    def to_dict(self):
        return {"time": self.time,
                "header": self.header.to_dict(),
                "vehicle_set": self.vehicle_set.to_dict()}