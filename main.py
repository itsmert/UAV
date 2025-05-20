import simpy
from utils import config
from simulator.simulator import Simulator

"""
  _______  ___       ___  ___  _____  ___    _______  ___________  
 /"     "||"  |     |"  \/"  |(\"   \|"  \  /"     "|("     _   ") 
(: ______)||  |      \   \  / |.\\   \    |(: ______) )__/  \\__/  
 \/    |  |:  |       \\  \/  |: \.   \\  | \/    |      \\_ /     
 // ___)   \  |___    /   /   |.  \    \. | // ___)_     |.  |     
(:  (     ( \_|:  \  /   /    |    \    \ |(:      "|    \:  |     
 \__/      \_______)|___/      \___|\____\) \_______)     \__|     
                                                                                                                                                                                                                                      
"""

"""
-- Credits --

This simulation framework is developed by Zihao Zhou eezihaozhou@gmail.com https://github.com/ZihaoZhouSCUT.
Some components were added and further extended by Hamzeh Abu Ali (MSc student at METU NCC).

"""


"""

-- Description of the folder structure --

energy
  energy_model.py-> Implementation of the UAV energy model (No need for changes there).

entities
  node.py-> The core logic for drone implementation. It includes the functionalities for generating and receiving
                      packets. More details are mentioned in the file.
  packet.py-> Packet implementation. Here, you will extend the attributes for the packets and generate them based on
                      the probability distributions mentioned in the assignment.
mac
  csma_ca.py-> An implmentation of CSMA/CA. Generally, no changes should be needed here, but that is up to your implementation.
  pure_aloha.py-> An implementation of pure ALOHA. We advise you to you CSMA/CA, but you could use both and compare between the results.

mobility
  gauss_markov_3d.py-> Gauss-Markov mobility model. A random outdoor mobility model with future steps depending on previous steps based on a parameter.
                                More details are found in the file. You can use it in your simulation.
  random_walk_3d.py-> A random memoryless mobility model. Future steps are independent of previous ones, and are totally random.
  random_waypoint.py-> A drone visits a set of generate way-points will pausing at every point for a while. If waypoint finish
                       before the simulaiton finishes, drone stops at the last waypoint.

phy
  channel.py-> channel formation (no changes are needed here).
  large_scale_fading.py-> defines physical layer properties such as Signal-to-interference plus noise ratio, pathloss,...
                              (no changes are needed here).
  phy.py-> implementation of the physical layer through message formation (no changes are needed here).

routing
  Defines the routing protocols (more details can be found in each directory). You can use the Greedy Perimeter Stateless Routing (GPSR) protocol.

simulator
  metrics.py-> includes the evaluation metrics in the simulation framework. More details can be found in the file.
  simulator.py-> the simulator class is responsible for calling the functions and classes responsible for...
                           assigning starting points for the sensors and UAVs, creating drone and sensor objects, and displaying
                           visuals for initial node positions (more details can be found there).

topology
  There is no need to describe this folder as it is out of the scope. You can leave it unchanged.

utils
  config.py-> this file includes all the configuration parameters. You can find details inside it as to which parameters to tune
                    and which to leave.
  ieee_802_11.py-> includes the definition of the IEEE 802.11 Wi-Fi standard. You can leave it unchanged, but you can choose whichever
                   version (a/b/c/g) you like. You can modify your protocol from utils/config.py
  util_function.py-> Includes some special functions used across the framework. You can leave it unchanged.

"""

"""

Some notes:

* Download the related libaries to be able to run the code (SimPy, Numpy, matplotlib, scipy, seaborn, ...).
* The main processes in the simulation are logged in the running_log.log file. You can check the file while/after the simulation runs.
* Try to include as much print and log statements as you can in your implementation to track the simulation progress, and to check
  if you are working correctly. Once sure if your implementation , you can keep the important print and log statements.
* Start small. Run your simulation with one drone and few sensors on a small map within a small timeframe (<=100s). 
  Check you results and logic, and increase in complexity if everything is working correctly.
* Don't attempt to run the code with a large number of nodes. 
  Simulation might take a long time to finish (>1day). Try to obtain results with (10-50 nodes).
* Try to familiarise yourself with the framework early on so you know how to approach the problems.
* Ask me questions if there are any unclear parts.


"""


if __name__ == "__main__":
    env = simpy.Environment()
    channel_states = {i: simpy.Resource(env, capacity=1) for i in range(config.NUMBER_OF_NODES)}
    sim = Simulator(seed=2024, env=env, channel_states=channel_states, n_nodes=config.NUMBER_OF_NODES)

    env.run(until=config.SIM_TIME)
