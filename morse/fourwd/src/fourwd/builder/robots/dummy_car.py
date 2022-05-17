from morse.builder import *


class DummyCar(GroundRobot):
    def __init__(self, name=None):
        GroundRobot.__init__(self, 'fourwd/robots/dummy_car_low.blend', name)
        self.properties(classpath="fourwd.robots.dummy_car.DummyCar")
