import simpy
import logging
import numpy as np
import random
import math
import queue
from entities.packet import DataPacket
from routing.dsdv.dsdv import Dsdv
from routing.gpsr.gpsr import Gpsr
from routing.grad.grad import Grad
from routing.opar.opar import Opar
from routing.parrot.parrot import Parrot
from routing.q_routing.q_routing import QRouting
from mac.csma_ca import CsmaCa
from mac.pure_aloha import PureAloha
from mobility.gauss_markov_3d import GaussMarkov3D
from mobility.random_walk_3d import RandomWalk3D
from mobility.random_waypoint_3d import RandomWaypoint3D
from topology.virtual_force.vf_motion_control import VfMotionController
from energy.energy_model import EnergyModel
from utils import config
from utils.util_function import has_intersection
from phy.large_scale_fading import sinr_calculator

# config logging
logging.basicConfig(filename='running_log.log',
                    filemode='w',  # there are two modes: 'a' and 'w'
                    format='%(asctime)s - %(levelname)s - %(message)s',
                    level=config.LOGGING_LEVEL
                    )

GLOBAL_DATA_PACKET_ID = 0


class Node:
    transmitting_queue = queue.Queue()
    """
    Node implementation

    Nodes in the simulation are served as routers. Each node can be selected as a potential source node, destination
    and relaying node. Each drone needs to install the corresponding routing module, MAC module, mobility module and
    energy module, etc. However, sensors will not have any mobility model. In addition, drone nodes will not generate
    any packets At the same time, each node also has its own queue and can only send one packet at a time, so
    subsequent data packets need queuing for queue resources, which is used to reflect the queue delay in the node
    network.

    Attributes:
        simulator: the simulation platform that contains everything
        env: simulation environment created by simpy
        identifier: used to uniquely represent a drone
        coords: the 3-D position of the drone
        start_coords: the initial position of drone
        direction: current direction of the drone
        pitch: current pitch of the drone
        speed: current speed of the drone
        velocity: velocity components in three directions
        direction_mean: mean direction
        pitch_mean: mean pitch
        velocity_mean: mean velocity
        inbox: a "Store" in simpy, used to receive the packets from other drones (calculate SINR)
        buffer: used to describe the queuing delay of sending packet
        transmitting_queue: when the next hop node receives the packet, it should first temporarily store the packet in
                    "transmitting_queue" instead of immediately yield "packet_coming" process. It can prevent the buffer
                    resource of the previous hop node from being occupied all the time
        waiting_list: for reactive routing protocol, if there is no available next hop, it will put the data packet into
                      "waiting_list". Once the routing information bound for a destination is obtained, drone will get
                      the data packets related to this destination, and put them into "transmitting_queue"
        mac_protocol: installed mac protocol (CSMA/CA, ALOHA, etc.)
        mac_process_dict: a dictionary, used to store the mac_process that is launched each time
        mac_process_finish: a dictionary, used to indicate the completion of the process
        mac_process_count: used to distinguish between different "mac_send" processes
        routing_protocol: routing protocol installed (GPSR, DSDV, etc.)
        mobility_model: mobility model installed (3-D Gauss-markov, 3-D random waypoint, etc.)
        motion_controller: used to control the cooperative movement of drones
        energy_model: energy consumption model installed
        residual_energy: the residual energy of drone in Joule
        sleep: if the drone is in a "sleep" state, it cannot perform packet sending and receiving operations.

    """

    
    """
    The main logic for priority queuing and task offloading is implemented here, mainly in the generate datapacket and 
    feedpacket functions
    
    """

    def __init__(self,
                 env,
                 node_id,
                 coords,
                 speed,
                 inbox,
                 simulator,
                 sensor_drone_flag):
        self.simulator = simulator
        self.env = env
        self.identifier = node_id
        self.coords = coords
        self.start_coords = coords

        random.seed(2024 + self.identifier)
        self.direction = random.uniform(0, 2 * np.pi)

        random.seed(2025 + self.identifier)
        self.pitch = random.uniform(-0.05, 0.05)
        self.speed = speed
        self.velocity = [self.speed * math.cos(self.direction) * math.cos(self.pitch),
                         self.speed * math.sin(self.direction) * math.cos(self.pitch),
                         self.speed * math.sin(self.pitch)]

        self.direction_mean = self.direction
        self.pitch_mean = self.pitch
        self.velocity_mean = self.speed

        self.inbox = inbox

        # ✅ NEW: Initialize individual priority queues for Task 2
        self.priority_queues = {
            "high": queue.Queue(),
            "medium": queue.Queue(),
            "low": queue.Queue()
        }

        # ✅ NEW: Processing queue for Task 4
        self.compute_queue = queue.Queue()

        self.buffer = simpy.Resource(env, capacity=1)
        self.max_queue_size = config.MAX_QUEUE_SIZE

        self.waiting_list = []
        # ✅ NEW: Set UAV processing capacity if this is a drone
        if sensor_drone_flag == 1:
            self.processing_capacity = round(np.random.uniform(2.0, 5.0), 2)
            self.simulator.metrics.uav_capacities.append(self.processing_capacity)

        self.mac_protocol = CsmaCa(self)
        self.mac_process_dict = dict()
        self.mac_process_finish = dict()
        self.mac_process_count = 0
        self.enable_blocking = 1
        self.sensor_drone_flag=sensor_drone_flag

        """
        You need to set the condition in the if-statement below manually depending on how many sensor nodes and drones you want.
        for example, currently, in the utils/config.py file, NUMBER_OF_NODES=10, and I want 7 sensors and 3 drones in the
        implementation, so I set self.identifier<8, meaning nodes from 0-7 are sensor nodes (since they generate the packets),
        and in the else-statement (drones with ids 8-9) are assigned (you can understand that they are drones since they have
        a mobility model (self.mobility_model = GaussMarkov3D(self)))
        """

        if(self.sensor_drone_flag==0): # sensor node features, set the value here based on your implementation

            self.routing_protocol = Gpsr(self.simulator, self) # You can keep the routing protocol as is
            self.energy_model = EnergyModel()
            self.residual_energy = config.INITIAL_ENERGY
            self.sleep = False

            self.env.process(self.generate_data_packet()) # sensors generate and route packets to drones
            self.env.process(self.feed_packet())
            self.env.process(self.energy_monitor())
            self.env.process(self.receive())
        else:  # drone features
            self.routing_protocol = Gpsr(self.simulator, self)
            self.mobility_model = GaussMarkov3D(self)
            self.energy_model = EnergyModel()
            self.residual_energy = config.INITIAL_ENERGY
            self.sleep = False

            self.env.process(self.feed_packet())
            self.env.process(self.energy_monitor())
            self.env.process(self.receive())


    def generate_data_packet(self, traffic_pattern='Poisson'):

        """
        Generate one data packet, it should be noted that only when the current packet has been sent can the next
        packet be started. When the drone generates a data packet, it will first put it into the "transmitting_queue",
        the drone reads a data packet from the head of the queue every very short time through "feed_packet()" function.
        :param traffic_pattern: characterize the time interval between generating data packets
        :return: none
        """

        global GLOBAL_DATA_PACKET_ID

        while True:
            if not self.sleep:
                if traffic_pattern == 'Uniform':
                    # the drone generates a data packet every 0.5s with jitter
                    yield self.env.timeout(random.randint(5000000, 5050000))
                    #yield self.env.timeout(random.randint(500000, 505000))
                elif traffic_pattern == 'Poisson':
                    """
                    the process of generating data packets by nodes follows Poisson distribution, thus the generation 
                    interval of data packets follows exponential distribution
                    """

                    # Set lambda (arrival rate) based on simulation time
                    current_time_s = self.env.now / 1e6  # convert ns to seconds

                    if current_time_s < 50:
                        rate = 0.5  # low load
                    elif current_time_s < 100:
                        rate = 2.0  # medium load
                    else:
                        rate = 5.0  # high load
                    yield self.env.timeout(round(random.expovariate(rate) * 1e6))

                GLOBAL_DATA_PACKET_ID += 1  # data packet id

                # choosing the destination drone for this packet randomly
                # build a list of all other drones that have sensor_drone_flag == 1, meaning drones
                drone_candidates = [
                    drone
                    for drone in self.simulator.drones
                    if drone.sensor_drone_flag == 1
                    and drone.identifier != self.identifier
                ]
                destination = random.choice(drone_candidates)
                dst_id = destination.identifier
                destination = self.simulator.drones[dst_id]

                pkd = DataPacket(self,
                                 dst_drone=destination,
                                 creation_time=self.env.now,
                                 data_packet_id=GLOBAL_DATA_PACKET_ID,
                                 data_packet_length=config.DATA_PACKET_LENGTH,
                                 simulator=self.simulator)
                pkd.transmission_mode = 0  # the default transmission mode of data packet is "unicast" (0)
                self.simulator.metrics.packet_sizes.append(pkd.size_mb)
                self.simulator.metrics.priority_distribution[pkd.priority] += 1
                self.simulator.metrics.datapacket_generated_num += 1

                logging.info('------> Sensor: %s generates a data packet (id: %s, dst: %s) at: %s, qsize is: %s',
                             self.identifier, pkd.packet_id, destination.identifier, self.env.now,
                             Node.transmitting_queue.qsize())

                pkd.waiting_start_time = self.env.now
                print(f"[GEN] Sensor {self.identifier} -> DataPacket {pkd.packet_id} -> Dest {dst_id}")

                # 7. Try to put it into the appropriate priority queue
                queue_ref = self.priority_queues.get(pkd.priority)
                if queue_ref and queue_ref.qsize() < self.max_queue_size:
                    queue_ref.put(pkd)
                    Node.transmitting_queue.put(pkd)

                else:
                    logging.warning('Queue FULL! Dropping packet %s at Sensor %s', pkd.packet_id, self.identifier)

            else:
                break  # node is asleep

    # The process of waiting for an ACK will block subsequent incoming data packets
    # head-of-line blocking problem
    def blocking(self):
        if self.enable_blocking:
            if not self.mac_protocol.wait_ack_process_finish:
                flag = False  # there is currently no waiting process for ACK
            else:
                # get the latest process status
                final_indicator = list(self.mac_protocol.wait_ack_process_finish.items())[-1]

                if final_indicator[1] == 0:
                    flag = True  # indicates that the drone is still waiting
                else:
                    flag = False  # there is currently no waiting process for ACK
        else:
            flag = False

        return flag

    def feed_packet(self):

        """
        It should be noted that this function is designed for those packets which need to compete for wireless channel

        Firstly, all packets received or generated will be put into the "transmitting_queue", every very short
        time, the drone will read the packet in the head of the "transmitting_queue". Then the drone will check
        if the packet is expired (exceed its maximum lifetime in the network), check the type of packet:
        1) data packet: check if the data packet exceeds its maximum re-transmission attempts. If the above inspection
           passes, routing protocol is executed to determine the next hop drone. If next hop is found, then this data
           packet is ready to transmit, otherwise, it will be put into the "waiting_queue".
        2) control packet: no need to determine next hop, so it will directly start waiting for buffer

        :return: none
        """

        

        while True:
            if not self.sleep:  # if drone still has enough energy to relay packets
                
                yield self.env.timeout(10)  # for speed up the simulation

                if not self.blocking():

                    
                    
                    if not Node.transmitting_queue.empty():
                        
                        packet = Node.transmitting_queue.get()  # get the packet at the head of the queue

                        if self.env.now < packet.creation_time + packet.deadline:  # this packet has not expired
                            if isinstance(packet, DataPacket):
                                # 🔴 DROP POLICY: if required_rate > processing_capacity
                                required_rate = packet.size_mb / packet.processing_time
                                if hasattr(self, 'processing_capacity') and required_rate > self.processing_capacity:
                                    logging.warning(
                                        '[DROP] Packet %s dropped at Node %s due to insufficient processing rate (%.2f > %.2f MB/s)',
                                        packet.packet_id, self.identifier, required_rate, self.processing_capacity)
                                    self.simulator.metrics.packets_dropped_due_to_capacity += 1  # ← metrik olarak say (metrics.py’ye eklenecek)
                                    continue  # skip
                                if packet.number_retransmission_attempt[self.identifier] < config.MAX_RETRANSMISSION_ATTEMPT:
                                    print(f"[FEED] Node {self.identifier} reading packet {packet.packet_id}")
                                    # UAV bu paketi işleyebilir mi?


                                    #print("here noww\n")
                                    # it should be noted that "final_packet" may be the data packet itself or a control
                                    # packet, depending on whether the routing protocol can find an appropriate next hop
                                    has_route, final_packet, enquire = self.routing_protocol.next_hop_selection(packet)

                                    if has_route:
                                        logging.info('Sensor: %s obtain the next hop: %s of data packet (id: %s)',
                                                     self.identifier, packet.next_hop_id, packet.packet_id)

                                        # in this case, the "final_packet" is actually the data packet
                                        yield self.env.process(self.packet_coming(final_packet))
                                    else:
                                        self.waiting_list.append(packet)

                                        if enquire:
                                            # in this case, the "final_packet" is actually the control packet
                                            yield self.env.process(self.packet_coming(final_packet))

                            else:  # control packet but not ack
                                yield self.env.process(self.packet_coming(packet))
                        else:
                            pass  # means dropping this data packet for expiration
            else:  # this drone runs out of energy
                break  # it is important to break the while loop

    def packet_coming(self, pkd):
        """
        When drone has a packet ready to transmit, yield it.

        The requirement of "ready" is:
            1) this packet is a control packet, or
            2) the valid next hop of this data packet is obtained
        :param pkd: packet that waits to enter the buffer of drone
        :return: none
        """

        if not self.sleep:
            arrival_time = self.env.now
            logging.info('Packet: %s waiting for Sensor: %s buffer resource at: %s',
                         pkd.packet_id, self.identifier, arrival_time)

            with self.buffer.request() as request:
                yield request  # wait to enter to buffer

                logging.info('Packet: %s has been added to the buffer at: %s of Sensor: %s, waiting time is: %s',
                             pkd.packet_id, self.env.now, self.identifier, self.env.now - arrival_time)

                pkd.number_retransmission_attempt[self.identifier] += 1

                if pkd.number_retransmission_attempt[self.identifier] == 1:
                    pkd.time_transmitted_at_last_hop = self.env.now

                logging.info('Re-transmission times of pkd: %s at Sensor: %s is: %s',
                             pkd.packet_id, self.identifier, pkd.number_retransmission_attempt[self.identifier])

                # every time the drone initiates a data packet transmission, "mac_process_count" will be increased by 1
                self.mac_process_count += 1
                key = str(self.identifier) + '_' + str(self.mac_process_count)  # used to uniquely refer to a process
                mac_process = self.env.process(self.mac_protocol.mac_send(pkd))
                self.mac_process_dict[key] = mac_process
                self.mac_process_finish[key] = 0

                yield mac_process
        else:
            pass

    def energy_monitor(self):
        while True:
            yield self.env.timeout(1 * 1e5)  # report residual energy every 0.1s
            if self.residual_energy <= config.ENERGY_THRESHOLD:
                self.sleep = True
                logging.info('UAV: %s run out of energy at: %s',self.identifier, self.env.now)
                print('UAV: ', self.identifier, ' run out of energy at: ', self.env.now)

    def remove_from_queue(self, data_pkd):
        """
        After receiving the ack packet, drone should remove the data packet that has been acked from its queue
        :param data_pkd: the acked data packet
        :return: none
        """
        temp_queue = queue.Queue()

        while not Node.transmitting_queue.empty():
            pkd_entry = Node.transmitting_queue.get()
            if pkd_entry != data_pkd:
                temp_queue.put(pkd_entry)

        while not temp_queue.empty():
            Node.transmitting_queue.put(temp_queue.get())

    def receive(self):
        """
        Core receiving function of drone
        1. the drone checks its "inbox" to see if there is incoming packet every 5 units (in us) from the time it is
           instantiated to the end of the simulation
        2. update the "inbox" by deleting the inconsequential data packet
        3. then the drone will detect if it receives a (or multiple) complete data packet(s)
        4. SINR calculation
        :return: none
        """

        while True:
            if not self.sleep:
                # delete packets that have been processed and do not interfere with
                # the transmission and reception of all current packets
                self.update_inbox()

                flag, all_drones_send_to_me, time_span, potential_packet = self.trigger()

                if len(all_drones_send_to_me) > 1:
                    self.simulator.metrics.collision_num += 1

                if flag:
                    # find the transmitters of all packets currently transmitted on the channel
                    transmitting_node_list = []
                    for drone in self.simulator.drones:
                        for item in drone.inbox:
                            packet = item[0]
                            insertion_time = item[1]
                            transmitter = item[2]
                            transmitting_time = packet.packet_length / config.BIT_RATE * 1e6
                            interval = [insertion_time, insertion_time + transmitting_time]

                            for interval2 in time_span:
                                if has_intersection(interval, interval2):
                                    transmitting_node_list.append(transmitter)

                    transmitting_node_list = list(set(transmitting_node_list))  # remove duplicates

                    sinr_list = sinr_calculator(self, all_drones_send_to_me, transmitting_node_list)

                    # receive the packet of the transmitting node corresponding to the maximum SINR
                    max_sinr = max(sinr_list)
                    if max_sinr >= config.SNR_THRESHOLD:
                        which_one = sinr_list.index(max_sinr)

                        pkd = potential_packet[which_one]

                        if pkd.get_current_ttl() < config.MAX_TTL:
                            sender = all_drones_send_to_me[which_one]
                            print(f"[RECEIVE] Node {self.identifier} receiving packet {pkd.packet_id} from {sender}")

                            logging.info('Packet %s from Sensor: %s is received by Sensor: %s at time: %s, sinr is: %s',
                                         pkd.packet_id, sender, self.identifier, self.simulator.env.now, max_sinr)

                            yield self.env.process(self.routing_protocol.packet_reception(pkd, sender))
                        else:
                            logging.info('Packet %s is dropped due to exceeding max TTL', pkd.packet_id)
                    else:  # sinr is lower than threshold
                        pass

                yield self.env.timeout(5)
            else:
                break

    def update_inbox(self):
        """
        Clear the packets that have been processed.
                                           ↓ (current time step)
                              |==========|←- (current incoming packet p1)
                       |==========|←- (packet p2 that has been processed, but also can affect p1, so reserve it)
        |==========|←- (packet p3 that has been processed, no impact on p1, can be deleted)
        --------------------------------------------------------> time
        :return:
        """

        max_transmission_time = (config.DATA_PACKET_LENGTH / config.BIT_RATE) * 1e6  # for a single data packet
        for item in self.inbox:
            insertion_time = item[1]  # the moment that this packet begins to be sent to the channel
            received = item[3]  # used to indicate if this packet has been processed (1: processed, 0: unprocessed)
            if insertion_time + 2 * max_transmission_time < self.env.now:  # no impact on the current packet
                if received:
                    self.inbox.remove(item)

    def trigger(self):
        """
        Detects whether the drone has received a complete data packet
        :return:
        1. flag: bool variable, "1" means a complete data packet has been received by this drone and vice versa
        2. all_drones_send_to_me: a list, including all the sender of the complete data packets received
        3. time_span, a list, the element inside is the time interval in which the received complete data packet is
           transmitted in the channel
        4. potential_packet, a list, including all the instances of the received complete data packet
        """

        flag = 0  # used to indicate if I receive a complete packet
        all_drones_send_to_me = []
        time_span = []
        potential_packet = []

        for item in self.inbox:
            packet = item[0]  # not sure yet whether it has been completely transmitted
            insertion_time = item[1]  # transmission start time
            transmitter = item[2]
            processed = item[3]  # indicate if this packet has been processed
            transmitting_time = packet.packet_length / config.BIT_RATE * 1e6  # expected transmission time

            if not processed:  # this packet has not been processed yet
                if self.env.now >= insertion_time + transmitting_time:  # it has been transmitted completely
                    flag = 1
                    all_drones_send_to_me.append(transmitter)
                    time_span.append([insertion_time, insertion_time + transmitting_time])
                    potential_packet.append(packet)
                    item[3] = 1
                else:
                    pass
            else:
                pass

        return flag, all_drones_send_to_me, time_span, potential_packet
