import logging; logger = logging.getLogger("morsebuilder." + __name__)
from morse.builder.creator import ComponentCreator, ActuatorCreator
from morse.builder.blenderobjects import *
from morse.core.exceptions import MorseBuilderError

class DummyKeyboard(ActuatorCreator):
    _classpath = "fourwd.actuators.dummy_car_keyboard.DummyKeyboard"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)
        self.mark_unexportable()
