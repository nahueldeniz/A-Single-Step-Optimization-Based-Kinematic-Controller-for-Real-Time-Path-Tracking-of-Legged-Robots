% VERY USEFUL INFORMATION ABOUT TROUBLES RELATED THE UNIRTEE GO1:
% https://githubplus.com/unitreerobotics/unitree_ros_to_real/issues
%
% to connect through wifi to the unitree go1:
% To control the robot via Wi-Fi, the following has to be run on the robots Pi (ssh pi@192.168.12.1 password 123):
% 
%     uncomment the line net.ipv4.ip_forward=1 in the file /etc/sysctl.conf if not already done
%     Run (on the Pi):
% 
% sudo sysctl -p
% sudo iptables -F
% sudo iptables -t nat -F
% sudo iptables -t nat -A POSTROUTING -o wlan1 -j MASQUERADE
% sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
% sudo iptables -A FORWARD -i wlan1 -o eth0 -j ACCEPT
% sudo iptables -A FORWARD -i eth0 -o wlan1 -j ACCEPT
% 
%     On your host machine:
% 
% sudo route add -net 192.168.123.0/24 gw 192.168.12.1
%
% very good forum with useful info: 
% https://forum.mybotshop.de/t/unitree-go1-high-state-topic-receives-only-zeros/443


rosshutdown;
rosinit;

% launch the topic "keyboard_control.lanch" in a terminal
% Once it is laucnhed, suscribe to it.
% For publishing the velocities:
go1_vel     = rossubscriber('/cmd_vel');
[pub,msg]   = rospublisher('/cmd_vel_aux','geometry_msgs/Twist');
% For reading the states:
go1_status     = rossubscriber('/parche_go1');
Status = init_Status();
%%
% read go1 status
Status = read_go1_status(go1_status, Status, 0);
%
speedX = 0.5;
%
msg.Linear.X    = speedX;
msg.Linear.Y    = 0;
msg.Linear.Z    = 0;
msg.Angular.X   = 0;
msg.Angular.Y   = 0;
msg.Angular.Z   = 0;
% Send the command
send(pub, msg)
pause(1.5);
%
msg.Linear.X    = 0;
send(pub, msg)
pause(0.5);
% 
msg.Linear.X    = -speedX;
send(pub, msg)
pause(1.5);
%
msg.Linear.X    = 0;
send(pub, msg)