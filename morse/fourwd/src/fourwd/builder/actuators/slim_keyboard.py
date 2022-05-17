import logging; logger = logging.getLogger("morsebuilder." + __name__)
from morse.builder.creator import ComponentCreator, ActuatorCreator
from morse.builder.blenderobjects import *
from morse.core.exceptions import MorseBuilderError

class SlimKeyboard(ActuatorCreator):
    _classpath = "fourwd.actuators.slim_keyboard.SlimKeyboard"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)
        self.mark_unexportable()
