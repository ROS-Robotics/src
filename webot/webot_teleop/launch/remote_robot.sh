#!/usr/bin/expect -f
set robot_ip [lindex $argv 0]
set pc_ip [lindex $argv 1]
set user [lindex $argv 2]
set pass [lindex $argv 3]
set launch_file [lindex $argv 4]
spawn ssh -o StrictHostKeyChecking=no $user@$robot_ip -o ServerAliveInterval=30

expect {
        "*?assword:*" {
         send "$pass\r"
    }
    "No route to host" {
        puts  "No route to host $robot_ip, check that the robot IP is correct"
        exit
    }
}

expect "$user@"
send "echo '#!/bin/sh' > ~/catkin_ws/src/ric/ric_base_station/config/remote_env.sh\r"
send "echo 'export ROS_HOSTNAME=\"$robot_ip\"' >> ~/catkin_ws/src/ric/ric_base_station/config/remote_env.sh\r"
send "echo 'export ROS_MASTER_URI=\"http://localhost:11311\"' >> ~/catkin_ws/src/ric/ric_base_station/config/remote_env.sh\r"
send "echo '. ~/catkin_ws/devel/setup.sh' >> ~/catkin_ws/src/ric/ric_base_station/config/remote_env.sh\r"
send "echo 'exec \"\$@\"' >> ~/catkin_ws/src/ric/ric_base_station/config/remote_env.sh\r"

send "roscore\r"
expect "started core service"
puts "Launching remote robot..."
system "~/catkin_ws/src/ric/ric_base_station/launch/base_station.sh $robot_ip $pc_ip $user $pass $launch_file"
while {true } {
#	puts ".\n";
	sleep 5
}

