class Driver:

    def __init__(self):
        pass

    def start(self):
        pass

    def is_waiting(self):
        _ = raw_input("Waiting (Press Enter to release)")
        return True

    def new_target(self, d, a):
        print("Driving to: " + str(d) + " " + str(a))
