# binpicking


Installing packages on Ubuntu 16.04

# Realsense package 
Ga naar de website https://github.com/intel-ros/realsense en volg de instructies voor het installeren van de realsense package.

# Pointcloud library 1.8 
Ga naar de website https://larrylisky.com/2016/11/03/point-cloud-library-on-ubuntu-16-04-lts/ en volg de instructies voor het downloaden en installeren van PCL versie 1.8.

# Universal_robot 
Ga naar de website http://wiki.ros.org/universal_robot en volg de instructies voor het installeren van de package. Let op dat de pagina staat ingesteld op Kinetic omdat er gebruik gemaakt wordt van ROS Kinetic.

# Ur_moder_driver 
Ga naar de website https://github.com/ros-industrial/ur_modern_driver en volg de installation instructies.
Na installatie "Script" folder toevoegen.

# Python-urx 
Ga naar de website https://www.rosehosting.com/blog/how-to-install-pip-on-ubuntu-16-04/ en download PIP. Ga vervolgens naar de website https://github.com/SintefManufacturing/python-urx en volg de installatie instructies.

# opstarten 
Na installatie van al deze packages kun je de huidige mappen vervangen door de mappen die op github staan. De map Universal_Robot hoeft niet vervangen te worden.
Handleiding voor het opstarten van het programma ‘Bin Picking' 

1. Zet Universal Robot UR5 aan.  
2. Zorg dat de camera aangesloten is aan de computer. (De usb versterker ook tussen de kabel) 
3. Zorg ervoor dat er een ethernet kabel tussen de robot en de computer zit. 
4. Zet het robot IP-adres op 192.168.1.102 
5. Zet het subnetmasker op 255.255.255.0 
6. Zet de standaard gateway op 192.168.1.30 
7. Zet de gewenste DNS-server op 192.168.1.102 
8. Start de computer op met Ubuntu. 
9. Zet het IP-adres van je computer op 192.168.1.30 
10. Start 3 Terminals. 
11. Voer bij alle terminals in: Cd workspace  
12. Voer bij alle terminals in: source devel/setup.bash 
13. Voer in een terminal in: roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.1.102 [reverse_port:=REVERSE_PORT] (Reverse_port kun je zo laten staan) 
14. Voer bij een andere terminal in: rosrun pointcloud vision_node (deze start de vision applicatie op) 
15. Voer bij de laatste terminal in: rosrun ur_modern_driver project_bin_picking.py 

