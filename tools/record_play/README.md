###########################################
1. Run build.sh to obtain the latest protobuf

######## recorder_path_cyber.py #######
This tool records trajectory info into a csv file defined in filename_path. (cybertron environment)
for example:
python recorder_path_cyber.py
python recorder_path_cyber.py -b byd            # choose vehicle branch as byd (default is lincoln)
python recorder_path_cyber.py -s 5              # set as constant speed 5 (default is chassis speed)

######## recorder_path_ros.py #######
This tool records trajectory info into a csv file defined in filename_path.txt. (ros environment)
for example:
python recorder_path_ros.py
python recorder_path_ros.py -b byd              # choose vehicle branch as byd (default is lincoln)
python recorder_path_ros.py -s 5                # set as constant speed 5 (default is chassis speed)

######## player_path_cyber.py #######
This tool publishes planning trajectory from a csv file. (cybertron environment)
for example:
python player_path_cyber.py
python player_path_cyber.py -c t                # publish complete planning data (default is f)
python player_path_cyber.py -s 90               # set speed as 90% (default is 100%)

######## player_path_ros.py #######
This tool publishes planning trajectory from a csv file. (ros environment)
for example:
python player_path_ros.py
python player_path_ros.py -c t                  # publish complete planning data (default is f)
python player_path_ros.py -s 90                 # set speed as 90% (default is 100%)

######## player_path_cyber_single.py #######
This tool publishes complete planning trajectory only for one time. (cybertron environment)
for example:
python player_path_cyber_single.py -p t         # publish when get pad message (default is f)
python player_path_cyber_single.py -g 10000     # publish when gps time reaches and set gps
                                                # timestamp as 10000 (enter automode first)

######## player_path_ros_loop.py #######
This tool publishes planning loop trajectory. (ros environment)
for example:
python player_path_ros_loop.py -m t             # mode: publish data by time (default is t)
python player_path_ros_loop.py -m d             # mode: publish data by position data
python player_path_ros_loop.py -l t             # publish light data (default is f)