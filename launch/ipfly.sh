hostname=${1:-Bebop2_A}
ip=$(host $hostname | egrep -o '[0-9.]{7,15}')
echo hostname:$hostname resolved to $ip
roslaunch bebop_follow fly.launch ip:=$ip
