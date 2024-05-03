#
from pypdevs.DEVS import CoupledDEVS

# Import all models to couple
# from generators import PeriodicGenerator, SinusoidalGenerator 
from atomics.token_handler import TokenHandler, TokenGenerator

class TokenHandlerSystem(CoupledDEVS):
    def __init__(self, name='TokenHandlerSystem'):
        """
        A simple oscillator composed of two integrators (equivalent to a mass-spring system with m=1 and k=1).
        """
        # Always call parent class' constructor FIRST:
        CoupledDEVS.__init__(self, name)

        #if __name__ == '__main__':
        #action_token_0 = Token(
        #	creator='2',
        #	kind='action',
        #	order=127,
        #	data={'3': np.array([1.0, 2.0])},
        #	hops_to_target=4,
        #	hops_travelled=1
        #)
        #action_token_1 = Token(
        #	creator='1',
        #	kind='action',
        #	order=127,
        #	data={'2': np.array([1.0, 2.0])},
        #	hops_to_target=4,
        #	hops_travelled=2
        #)
        #action_token_2 = Token(
        #	creator='1',
        #	kind='action',
        #	order=128,
        #	data={'2': np.array([1.0, 2.0])},
        #	hops_to_target=4,
        #	hops_travelled=2
        #)
        #state_token = Token(
        #	creator='3',
        #	kind='state',
        #	order=132,
        #	data={'1': np.array([3.0, 4.0])},
        #	hops_to_target=4,
        #	hops_travelled=2
        #)

        # Declare the coupled model's sub-models:
        self.token_handler   = self.addSubModel(TokenHandler(robot_id='2',name='Token_Han'))
        self.token_generator = self.addSubModel(TokenGenerator(period=1,name='Token_Gen'))
        self.connectPorts(self.token_generator.OUT_token, self.token_handler.IN_token)

        # token_handler = TokenHandler(robot_id='2')

        # test do nothing if robot is the creator
        #self. token_handler.handle_received_token(action_token_0)
        #assert action_token_0.hops_travelled == 1
        #assert token_handler.received == {'action': {}, 'state': {}}

        # test update hops travelled
        #token_handler.handle_received_token(action_token_1)
        #assert action_token_1.hops_travelled == 3
        # test update received registry
        #assert token_handler.received == {'action': {'1': 127}, 'state': {}}

        # test update received registry
        #token_handler.handle_received_token(action_token_2)
        #assert token_handler.received == {'action': {'1': 128}, 'state': {}}
        # test not update received registry
        #token_handler.handle_received_token(action_token_1)
        #assert token_handler.received == {'action': {'1': 128}, 'state': {}}

        # test update hops travelled
        #token_handler.handle_received_token(state_token)
        #assert state_token.hops_travelled == 3
        # test update received registry
        #assert token_handler.received == {'action': {'1': 128}, 'state': {'3': 132}}
