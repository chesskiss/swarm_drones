# Swarm_drones
My partner and I developed a tool that received an image of a closed shape as an input, e.g. a triangle, converts it into a pattern, and controls a swarm of drones (as many as you want) to fly in that pattern. We had to deal with all levels of abstraction, starting by turning the drones into stations to establish a TCP connection (since by default they were using a direct WiFi connection, i.e. peer-to-peer), and going all the way up to managing cloud operations. 
Stack: Firebase, Vpython, Packet sender (to communicate with the drones, and Robo-phone (an abstraction that served as a distributed OS)
Video (it's in Hebrew but I added English subtitles)- https://www.youtube.com/watch?v=ld3mvVBgMNQ
