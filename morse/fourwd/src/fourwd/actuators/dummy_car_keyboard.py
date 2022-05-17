import logging;

logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.core.actuator import Actuator
from morse.helpers.components import add_data, add_property


class DummyKeyboard(Actuator):
    """
    This actuator does not require a connection with external data. It
    simply responds to the keyboard arrows to generate movement
    instructions for the robot attached.

    When parented to a robot, the user can press the arrow keys to modify the
    linear and angular velocities (V, W) of the robot.

    :kbd:`I` forward
    :kbd:`K` backwards
    :kbd:`J` turn/yaw left
    :kbd:`L` turn/yaw right
    :kbd:`O` strafe left
    :kbd:`U` strafe right
    """

    _name = "Dummy car Keyboard Actuator"
    _short_desc = "A 'fake' actuator that allows to move a robot from the keyboard."

    add_property('_type', 'Position', 'ControlType', 'string',
                 "Kind of control to move the parent robot, in ['Position', "
                 "'Velocity', 'Differential']")
    add_property('_speed', 1.0, 'Speed', 'float',
                 "Movement speed of the parent robot, in m/s")
    add_property('_id', 0, 'Id', 'int',
                 "can be 0 or 1, determines if the keyboard is enabled when pressing N")

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Actuator.__init__(self, obj, parent)

        # Correct the speed considering the Blender clock
        if self._type == 'Position':
            self._speed = self._speed / self.frequency

        self.zero_motion = True


    def default_action(self):
        """ Interpret keyboard presses and assign them to movement
            for the robot."""
        keyboard = blenderapi.keyboard()
        is_actived = blenderapi.input_active()

        # Reset movement variables
        vx, vy, vz = 0.0, 0.0, 0.0
        rx, ry, rz = 0.0, 0.0, 0.0
	# move forward
        if keyboard.events[blenderapi.KKEY] == is_actived:
            vx = self._speed

        # move backward
        if keyboard.events[blenderapi.IKEY] == is_actived:
            vx = -self._speed

        # turn left
        if keyboard.events[blenderapi.JKEY] == is_actived:
            rz = self._speed

        # turn right
        if keyboard.events[blenderapi.LKEY] == is_actived:
            rz = -self._speed

        # strafe left
        if keyboard.events[blenderapi.OKEY] == is_actived:
            vy = self._speed

        # strafe right
        if keyboard.events[blenderapi.UKEY] == is_actived:
            vy = -self._speed

        # Send a 'zero motion' only once in a row.
        if self.zero_motion and (vx, vy, vz, rx, ry, rz) == (0, 0, 0, 0, 0, 0):
            return

        if self._type == 'Position' or self._type == 'Velocity':
            self.robot_parent.apply_speed(self._type, [vx, vy, vz], [rx, ry, rz / 2.0])
        elif self._type == 'Differential':
            self.robot_parent.apply_vw_wheels(vx, rz)

        if (vx, vy, vz, rx, ry, rz) == (0, 0, 0, 0, 0, 0):
            self.zero_motion = True
        else:
            self.zero_motion = False
