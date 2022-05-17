from morse.builder import *


class DummyCar2(GroundRobot):
    def __init__(self, name=None):
        GroundRobot.__init__(self, 'fourwd/robots/dummy_car_2.blend', name)
        self.properties(classpath="fourwd.robots.dummy_car.DummyCar")
