from pypdevs.DEVS import AtomicDEVS

# Define the state of the collector as a structured object
class CollectorState(object):
    def __init__(self):
        # Contains received events and simulation time
        self.events = []
        self.current_time = 0.0

class Collector(AtomicDEVS):
    def __init__(self, name="Collector", filename="output.csv"):
        AtomicDEVS.__init__(self, name)
        self.state = CollectorState()
        self.filename = filename
        # Has two input ports
        self.in1_event = self.addInPort("in1_event")
        self.in2_event = self.addInPort("in2_event")

    def extTransition(self, inputs):
        # Update simulation time
        self.state.current_time += self.elapsed

        # Write data to file
        with open(self.filename, 'a') as f:
            if self.in1_event in inputs:
                q1 = inputs[self.in1_event][0]
                current_time = self.state.current_time
                self.state.events.append([current_time,q1,float('nan')])
                f.write("%f, %f,\n" %(current_time,q1))
            elif self.in2_event in inputs:
                q2 = inputs[self.in2_event][0]
                current_time = self.state.current_time
                self.state.events.append([current_time,float('nan'),q2])
                f.write("%f, , %f\n" %(current_time,q2))
        return self.state

    # Don't define anything else, as we only store events.
    # Collector has no behaviour of its own.
