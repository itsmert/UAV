import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
from datetime import datetime


class Metrics:
    """
    Tools for statistics of network performance

    1. Packet Delivery Ratio (PDR): is the ratio of number of packets received at the destinations to the number
       of packets sent from the sources
    2. Average end-to-end (E2E) delay: is the time a packet takes to route from a source to its destination through
       the network. It is the time the data packet reaches the destination minus the time the data packet was generated
       in the source node
    3. Routing Load: is calculated as the ratio between the numbers of control Packets transmitted
       to the number of packets actually received. NRL can reflect the average number of control packets required to
       successfully transmit a data packet and reflect the efficiency of the routing protocol
    4. Throughput: it can be defined as a measure of how fast the data is sent from its source to its intended
       destination without loss. In our simulation, each time the destination receives a data packet, the throughput is
       calculated and finally averaged
    5. Hop count: used to record the number of router output ports through which the packet should pass.

    References:
        [1] Rani. N, Sharma. P, Sharma. P., "Performance Comparison of Various Routing Protocols in Different Mobility
            Models," in arXiv preprint arXiv:1209.5507, 2012.
        [2] Gulati M K, Kumar K. "Performance Comparison of Mobile Ad Hoc Network Routing Protocols," International
            Journal of Computer Networks & Communications. vol. 6, no. 2, pp. 127, 2014.

    """

    def __init__(self, simulator):
        self.simulator = simulator

        self.control_packet_num = 0

        self.datapacket_generated = set()  # all data packets generated
        self.datapacket_arrived = set()  # all data packets that arrives the destination
        self.datapacket_generated_num = 0

        self.delivery_time = []
        self.deliver_time_dict = defaultdict()

        self.throughput = []
        self.throughput_dict = defaultdict()

        self.hop_cnt = []
        self.hop_cnt_dict = defaultdict()

        self.mac_delay = []

        self.packet_sizes = []
        self.priority_distribution = {"high": 0, "medium": 0, "low": 0}
        self.uav_capacities = []
        self.collision_num = 0
        self.dropped_due_to_capacity = 0  # Task 2 için: UAV'in kapasitesini aşan paketler
        self.packets_dropped_due_to_capacity = 0

    def print_metrics(self):
        # calculate the average end-to-end delay
        for key in self.deliver_time_dict.keys():
            self.delivery_time.append(self.deliver_time_dict[key])

        for key2 in self.throughput_dict.keys():
            self.throughput.append(self.throughput_dict[key2])

        for key3 in self.hop_cnt_dict.keys():
            self.hop_cnt.append(self.hop_cnt_dict[key3])

            # Safe metrics (prevent zero division)
        if len(self.datapacket_arrived) > 0:
            pdr = len(self.datapacket_arrived) / self.datapacket_generated_num * 100
            rl = self.control_packet_num / len(self.datapacket_arrived)
            throughput = np.mean(self.throughput) / 1e3
            hop_cnt = np.mean(self.hop_cnt)
            e2e_delay = np.mean(self.delivery_time) / 1e3
            average_mac_delay = np.mean(self.mac_delay)
        else:
            pdr = rl = throughput = hop_cnt = e2e_delay = average_mac_delay = 0

            # Safe average packet size and UAV capacity
        avg_packet_size = round(np.mean(self.packet_sizes), 2) if self.packet_sizes else 0
        avg_uav_capacity = round(np.mean(self.uav_capacities), 2) if self.uav_capacities else 0

        # Timestamp for results
        now = datetime.now()
        timestamp = now.strftime("%H-%M-%S_%d-%m-%Y")

        # Log results to file
        with open("simulator/network_results.txt", "a") as file:
            file.write(f"{timestamp}\n")
            file.write(f"Total Arrived is: {len(self.datapacket_arrived)}\n")
            file.write(f"Totally sent: {self.datapacket_generated_num} data packets\n")
            file.write(f"Packet delivery ratio is: {pdr}%\n")
            file.write(f"Average end-to-end delay is: {e2e_delay} ms\n")
            file.write(f"Routing load is: {rl}\n")
            file.write(f"Average throughput is: {throughput} Kbps\n")
            file.write(f"Average hop count is: {hop_cnt}\n")
            file.write(f"Collision num is: {self.collision_num}\n")
            file.write(f"Average MAC delay is: {average_mac_delay} ms\n")
            file.write(f"\n--- Packet-Level Statistics (Task 1) ---\n")
            file.write(f"Average Packet Size (MB): {avg_packet_size}\n")
            file.write(f"Priority Distribution: {self.priority_distribution}\n")
            file.write(f"UAV Capacities: {self.uav_capacities}\n")
            file.write(f"Average UAV Capacity (MB/s): {avg_uav_capacity}\n")
            file.write(f"Dropped Packets (Capacity Constraint): {self.dropped_due_to_capacity}\n")
            file.write(f"Packets Dropped due to Capacity Limits: {self.packets_dropped_due_to_capacity}\n")
            file.write(f"--------------------------------------------\n\n")

        # Print to console
        print('Totally send: ', self.datapacket_generated_num, ' data packets')
        print('Packet delivery ratio is: ', pdr, '%')
        print('Average end-to-end delay is: ', e2e_delay, 'ms')
        print('Routing load is: ', rl)
        print('Average throughput is: ', throughput, 'Kbps')
        print('Average hop count is: ', hop_cnt)
        print('Collision num is: ', self.collision_num)
        print('Average mac delay is: ', average_mac_delay, 'ms')
        print('\n--- Packet-Level Statistics (Task 1) ---')
        print('Average Packet Size (MB):', avg_packet_size)
        print('Priority Distribution:', self.priority_distribution)
        print('UAV Capacities:', self.uav_capacities)
        print('Average UAV Capacity (MB/s):', avg_uav_capacity)
        print('Packets Dropped due to Capacity Limits:', self.packets_dropped_due_to_capacity)


