#!/usr/bin/python
""" states.py
State Machine for the Bebop Follow package
2 Big States w/ sub states
- Follow Mode
-- AprilTag Lost -> Spin around
-- Following April Tag
-- Low on Battery -> switching to Manual Mode
- Manual Mode
-- Landed -> wait for takeoff command. Disable all other movements
-- In air

To be used in conjucntion with operator.py and potential_path.py
"""
import rospy

from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_msgs.msg import Ardrone3PilotingStateAlertStateChanged

#TODO ensure that follow does not write while operator is writing and vice versa.

class State(object):
    def __init__(self, rotation_controller, location_controller):
        self.rotation_controller = rotation_controller
        self.location_controller = location_controller
        #print("Processing current state: {0}".format(self.__name__))

    def run(self):
        pass
        #print("State {0} is running".format(self.__name__))

    def next(self, event):
        pass
        #print("State {0} is interrupted by {1}".format(self.__name__, event.__name__))
# Search State
# Mode: Follow
# Conditions: Lost April Tag
# Actions: Turn in circles until April Tag is found.
class SearchState(State):
    def __init__(self, rotation_controller, location_controller):
        super(SearchState, self).__init__(rotation_controller,location_controller)
        self.rotation_controller.publish('search')
        self.locaiton_controller.publish('hover')

    def run(self):
        super(SearchState, self).run()

    def next(self, event):
        if event == 'tag found':
            rospy.loginfo("April Tag found, switching to follow state")
            return FollowState(self.rotation_controller, self.location_controller)
        elif event == 'manual':
            rospy.loginfo("Entering Manual Mode, switching to flying state")
            return FlyingState(self.rotation_controller, self.location_controller)
        elif event == 'low battery':
            rospy.logwarn("Low Battery, switching to critical state")
            return CriticalState(self.rotation_controller, self.location_controller)
        else:
            return self

# Follow State
# Mode: Follow
# Conditions: April Tag in sight
# Actions: Keep April Tag at the center of the camera && Keep bebop a fixed distance away from the robot
class FollowState(State):
    def __init__(self, rotation_controller, location_controller):
        super(SearchState, self).__init__(rotation_controller,location_controller)
        self.rotation_controller.publish('follow')
        self.locaiton_controller.publish('follow')

    def run(self):
       super(SearchState, self).run()

    def next(self, event):
        if event == 'tag lost':
            rospy.loginfo("April Tag lost, switching to search state")
            return SearchState(self.rotation_controller, self.location_controller)
        elif event == 'manual':
            rospy.loginfo("Entering Manual Mode, switching to flying state")
            return FlyingState(self.rotation_controller, self.location_controller)
        elif event == 'low battery':
            rospy.logwarn("Low Battery, switching to critical state")
            return CriticalState(self.rotation_controller, self.location_controller)
        else:
            return self

# Critical State
# Mode: Follow, Manual
# Conditions: Low Battery
# Actions: Land
class CriticalState(State):
    def __init__(self, rotation_controller, location_controller):
        rospy.logwarn("Landing")
        self.land = rospy.Publisher("land", Empty, queue_size=1)
        self.land.publish()
        super(CriticalState, self).__init__(rotation_controller, location_controller)

    def run(self):
        super(CriticalState, self).run()

    def next(self, event):
        if event == 'grounded':
            rospy.loginfo("Successfully landed, switching to Grounded State")
            return GroundedState(self.rotation_controller, self.location_controller)
        elif event == 'low battery':
            self.land.publish() #TODO consider switching to emergency landing when low battery is warned a second time.
            return self
        else:
            return self

# Grounded State
# Mode: Follow, Manual
# Conditions: Motors off
# Actions: Nothing
class GroundedState(State):
    def __init__(self, rotation_controller, location_controller):
        super(GroundedState, self).__init__(rotation_controller, location_controller)

    def run(self):
        super(GroundedState, self).run()

    def next(self, event):
        if event == 'flying':
            rospy.loginfo("Successfully took off, switching to Flying State")
            return FlyingState(self.rotation_controller, self.location_controller)
        else:
            return self

# Flying State
# Mode: Follow
# Conditions: Motors on with operator control
# Actions: Obey Operator Control
def FlyingState(State):
    def __init__(self, rotation_controller, location_controller):
        super(FlyingState,self).__init__(rotation_controller, location_controller)
        #TODO give controls to the operator

    def run(self):
        super(FlyingState, self).run()

    def next(self, event):
        if event == 'grounded':
            rospy.loginfo("Landed, switching to grounded state")
            return GroundedState(self.rotation_controller, self.location_controller)
        elif event == 'follow':
            rospy.loginfo("switching to follow mode, switching to search state")
            return SearchState(self.rotation_controller, self.location_controller)
        elif event == 'low battery':
            rospy.logwarn("Low Battery, switching to critical state")
            return CriticalState(self.rotation_controller, self.location_controller)
        else:
            return self

#State Machine Itself. Subscribes to various topics and listens for an event update at which point it fires off the next method in the current state. Also runs the run method of the current state at 100ms intervals
class StateMachine():
    def __init__(self):
        rospy.init_node('state_machine')
        self.rate = rospy.Rate(10) #10 Hz cycle
        rospy.Subscriber("tag_found", Bool, self.update_tag_found)
        rospy.Subscriber("operation_mode", String, self.update_operation_mode)
        rospy.Subscriber("states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, self.update_flying)
        rospy.Subscriber("states/ardron3/PilotingState/AlertStateChanged", Ardrone3PilotingStateAlertStateChanged, self.update_alert)
        self.rotation_controller = rospy.Publisher("rotation_controller", String, queue_size=1)
        self.location_controller = rospy.Publisher("location_controller", String, queue_size=1)
        self.cmd_vel_topic = rospy.Publisher("cmd_vel_topic", String, queue_size=1)
        self.state = GroundedState(self.rotation_controller,self.location_controller)
        self.tag_found = False

    def run(self):
        while not rospy.is_shutdown():
            self.state.run()
            self.rate.sleep()

    def update_tag_found(self, data):
        if not (self.tag_found == data.data):
            self.tag_found == data.data
            if self.tag_found == True:
                self.state = self.state.next('tag found')
            else:
                self.state = self.state.next('tag lost')

    def update_operation_mode(self, data):
        if data.data == 'manual':
            self.cmd_vel_topic.publish(rospy.get_param("operator_topic"))
            self.state = self.state.next('manual')
        elif data.data == 'follow':
            self.cmd_vel_topic.publish(rospy.get_param("controller_topic"))
            self.state = self.state.next('follow')
        else:
            rospy.logwarn("Unknown mode " + data.data)

    def update_alert(self, data):
        if data.state == data.state_low_battery or data.state == data.state_critical_battery:
            self.state = self.state.next('low battery')
        else:
            rospy.logwarn("Other warning: " + str(data.state))

    def update_flying(self, data):
        if data.state == data.state_landed:
            self.state_landed = self.state.next("grounded")
        elif data.state == data.state_hovering or data.state == data.state_flying:
            self.state = self.state.next("flying")
        elif data.state == data.state_emergency_landing:
            rospy.logwarn("Emergency Landing State Entered")
        else:
            rospy.loginfo(data)
        


if __name__ == '__main__':
    sm = StateMachine()
    sm.run()
