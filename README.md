# script-for-PID-tuner

To run script, run following lines in 3 different terminals

roslaunch pid_calc posix_sitl.launch
roslaunch pid_calc px4.launch fcu_url:="udp://:14550@127.0.0.1:14557"
roslaunch pid_calc pid_calc.launch
