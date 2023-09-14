"""Library for controlling multiple DJI Ryze Tello drones.
"""

from threading import Thread, Barrier
from queue import Queue
from typing import List, Callable

from tello import Tello, TelloException
from enforce_types import enforce_types
import time
import socket

@enforce_types
class TelloSwarm:
    """Swarm library for controlling multiple Tellos simultaneously
    """

    tellos: List[Tello]
    barrier: Barrier
    funcBarier: Barrier
    funcQueues: List[Queue]
    threads: List[Thread]
    
    @staticmethod

    def find_tello(number_tello,timeout=0.15): 
    # Python Program to Get IP Address

        hostname = socket.gethostname()
        IPAddr = socket.gethostbyname(hostname)
        octets = IPAddr.split('.')
        ip_list = []

        for i in range(1, 255):
            if i%25 == 0:
                print(f"searching, {int(100*i/255)}%")
            if i == int(octets[3]):
                continue

            tello_ip = octets[0] + '.' + octets[1] + '.' + octets[2] + '.' + str(i)

            tello_port = 8889

            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Bind the socket to a local port (you can choose any available port)
            local_port = 9000
            sock.bind(('0.0.0.0', local_port))

            # Send a command to the Tello drone
            command = "command"  # You can send other Tello commands as needed
            sock.sendto(command.encode(), (tello_ip, tello_port))

            # Set a timeout of 0.5 seconds for receiving a response
            sock.settimeout(timeout)

            try:
                # Receive and print the response
                response, _ = sock.recvfrom(1024)
                if response.decode() == "ok":
                    print("Response:", response.decode(), "from IP:", tello_ip)
                    ip_list.append(tello_ip)

            except socket.timeout:
                pass

            # Close the socket
            sock.close()
        if len(ip_list) == number_tello:
            return TelloSwarm.fromIps(ip_list)
        else:
            print(f"find {len(ip_list)} tello, not {number_tello} tello")
            return None


    @staticmethod
    def fromFile(path: str):
        """Create TelloSwarm from file. The file should contain one IP address per line.

        Arguments:
            path: path to the file
        """
        with open(path, 'r') as fd:
            ips = fd.readlines()

        return TelloSwarm.fromIps(ips)

    @staticmethod
    def fromIps(ips: list):
        """Create TelloSwarm from a list of IP addresses.

        Arguments:
            ips: list of IP Addresses
        """
        if not ips:
            raise TelloException("No ips provided")

        tellos = []
        for ip in ips:
            tellos.append(Tello(ip.strip()))

        return TelloSwarm(tellos,ips)

    def __init__(self, tellos: List[Tello],ips: list):
        """Initialize a TelloSwarm instance

        Arguments:
            tellos: list of [Tello][tello] instances
        """
        self.ips=ips
        self.tellos = tellos
        self.barrier = Barrier(len(tellos))
        self.funcBarrier = Barrier(len(tellos) + 1)
        self.funcQueues = [Queue() for tello in tellos]
        self.get_comment=[]

        def stayConnected(self):
            while True:
                for i, tello in enumerate (self.tellos):
                    if time.time()-self.get_comment[i]>10:
                        tello.connect(wait_for_state=False)
                        self.get_comment[i]=time.time()
                time.sleep(2)

        def worker(i):
            queue = self.funcQueues[i]
            tello = self.tellos[i]

            while True:
                func = queue.get()
                self.get_comment[i]=time.time()
                self.funcBarrier.wait()
                try:func(i, tello)

                except:
                    print(f"Error with tello{i} {self.ips[i]}")    

                self.funcBarrier.wait()
                #print(f"Done with {i} tello")

        self.threads = []
        for i, _ in enumerate(tellos):
            self.get_comment.append(time.time()+5)
            thread = Thread(target=worker, daemon=True, args=(i,))
            thread.start()
            self.threads.append(thread)

        self.stay_connect= Thread(target=stayConnected, daemon=True, args=(self,))
        #self.stay_connect.start()

    def sequential(self, func: Callable[[int, Tello], None]):
        """Call `func` for each tello sequentially. The function retrieves
        two arguments: The index `i` of the current drone and `tello` the
        current [Tello][tello] instance.

        ```python
        swarm.parallel(lambda i, tello: tello.land())
        ```
        """

        for i, tello in enumerate(self.tellos):
            func(i, tello)

    def parallel(self, func: Callable[[int, Tello], None]):
        """Call `func` for each tello in parallel. The function retrieves
        two arguments: The index `i` of the current drone and `tello` the
        current [Tello][tello] instance.

        You can use `swarm.sync()` for syncing between threads.

        ```python
        swarm.parallel(lambda i, tello: tello.move_up(50 + i * 10))
        ```
        """

        for queue in self.funcQueues:
            queue.put(func)

        self.funcBarrier.wait()
        self.funcBarrier.wait()

    def sync(self, timeout: float = None):
        """Sync parallel tello threads. The code continues when all threads
        have called `swarm.sync`.

        ```python
        def doStuff(i, tello):
            tello.move_up(50 + i * 10)
            swarm.sync()

            if i == 2:
                tello.flip_back()
            # make all other drones wait for one to complete its flip
            swarm.sync()

        swarm.parallel(doStuff)
        ```
        """
        return self.barrier.wait(timeout)

    def __getattr__(self, attr):
        """Call a standard tello function in parallel on all tellos.

        ```python
        swarm.command()
        swarm.takeoff()
        swarm.move_up(50)
        ```
        """
        def callAll(*args, **kwargs):
            self.parallel(lambda i, tello: getattr(tello, attr)(*args, **kwargs))

        return callAll

    def __iter__(self):
        """Iterate over all drones in the swarm.

        ```python
        for tello in swarm:
            print(tello.get_battery())
        ```
        """
        return iter(self.tellos)

    def __len__(self):
        """Return the amount of tellos in the swarm

        ```python
        print("Tello count: {}".format(len(swarm)))
        ```
        """
        return len(self.tellos)
