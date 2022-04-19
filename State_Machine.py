import time
import rospy


class StateMachine:

    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []
        self.transition = None

    def add_state(self, name, handler, end_state=0):
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name.upper()
        try:
            test = self.handlers[self.startState]
        except KeyError:
            rospy.loginfo("Not a state:", self.startState)
            exit()

    def run(self, cargo, c_time=0):
        try:
            if not self.endStates:
                raise RuntimeError("At least one state must be an end_state")
            if self.startState is None:
                raise RuntimeError("Must call .set_start() before .run()")
            handler = self.handlers[self.startState]
            (newState, cargo, self.transition) = handler(cargo)

        except Exception as e:
            rospy.loginfo(e)
            exit()

        while True:
            if self.transition is not None:
                rospy.loginfo(self.transition)
                self.transition = None
            if c_time > 0:
                time.sleep(c_time)
            if newState.upper() in self.endStates:
                handler = self.handlers[newState.upper()]
                if handler is not None:
                    handler(cargo)
                rospy.loginfo("Finished in", newState)
                break
            else:
                try:
                    handler = self.handlers[newState.upper()]
                    (newState, cargo, self.transition) = handler(cargo)
                except KeyError:
                    rospy.loginfo("Not a state:", newState)
                    exit()
