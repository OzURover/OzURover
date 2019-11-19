import timeit

class Driver:

    def __init__(self):
        self.d = 0
        self.a = 0

    def start(self):
        pass

    def is_waiting(self, val=False):
        if val is True or val is False:
            return val
        else:
            return val > self.d

    def is_target_angle_reached(self):
        return (timeit.default_timer() - self.start_t) > 10.0

    def new_target(self, d, a):
        self.start_t = timeit.default_timer()
        self.d = d
        self.a = a
        print("Driving to: " + str(d) + " " + str(a))

    def move(self, pause=0):
        print("PAUSED" if pause == 1 else "RESUMED")

    def get_travelled(self):
        return 0

    def halt(self):
        print("HALT!")
        pass
