import logging

logger = logging.getLogger("morse." + __name__)
import morse.core.robot

class DummyCar2(morse.core.robot.Robot):
    """ 
    Class definition for the pole robot.
    """

    _name = 'dummy_car_2'

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main loop of the robot
        """

        # This is usually not used (responsibility of the actuators
        # and sensors). But you can add here robot-level actions.
        pass



