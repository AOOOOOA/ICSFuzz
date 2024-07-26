#!/bin/bash

#!/bin/bash

#TODO: tips： the followleading vehicle ori is the origial all scope one, in order to test the inconsistency, we modify it

# scenario_name="FollowLeadingVehicle"
# scenario_name="IntersectionCollisionAvoidance"

# run in the correct direction, but since I accidently kill it, I need to re-run it from the middle
scenario_name="LaneChangeSimple" 
#scenario_config="FollowLeadingVehicle1_dist5_speed2_10"
scenario_config="LaneChangeSimple"
temp_folder="all_temp_data/temp_data1"
result_txt="result_txt/FollowLeadingVehicle/dist5/result_FollowLeadingVehicle1_dist5_speed2_10.txt"
config_folder_path="FollowLeadingVehicle/dist5"
port=2025
tm_port=8025
graphics_num=1




count=0
launch_carla(){
        sh ../carla/CarlaUE4.sh -windowed -carla-port=$port -graphicsadapter=$graphics_num
    }

launch_sr_init(){
        sleep 20
        #TODO: change the temp_data folder path 
        python scenario_runner.py --openscenario srunner/examples/$scenario_name.xosc --record $temp_folder --port=$port --trafficManagerPort=$tm_port
        # pkill -9 python
    }
    
launch_ego_vehicle_init(){
        sleep 40
        python ego_vehicle.py $scenario_name $init_speed $init_direction_x $init_direction_y $distance $speed $d_x $d_y $count $temp_folder $result_txt $port
}


launch_sr(){
        sleep 10
        #TODO: change the temp_data folder path 
        python scenario_runner.py --openscenario srunner/examples/$scenario_name.xosc --record $temp_folder --port=$port --trafficManagerPort=$tm_port
        # pkill -9 python
    }
    
launch_ego_vehicle(){
        sleep 20
        python ego_vehicle.py $scenario_name $init_speed $init_direction_x $init_direction_y $distance $speed $d_x $d_y $count $temp_folder $result_txt $port
}



json=$(cat srunner/fuzzing_config/$scenario_config.json)
init_speed=$(echo $json | jq -r '.'$scenario_name'.maneuver_parameter.init_speed')
init_direction_x=$(echo $json | jq -r '.'$scenario_name'.maneuver_parameter.init_direction_x')
init_direction_y=$(echo $json | jq -r '.'$scenario_name'.maneuver_parameter.init_direction_y')

new_speed_scope=$(echo $json | jq -r '.'$scenario_name'.testing_scope.new_speed | map(tostring) | join(" ")')
change_distance_scope=$(echo $json | jq -r '.'$scenario_name'.testing_scope.change_distance | map(tostring) | join(" ")')
new_direction_x_scope=$(echo $json | jq -r '.'$scenario_name'.testing_scope.new_direction_x | map(tostring) | join(" ")')
new_direction_y_scope=$(echo $json | jq -r '.'$scenario_name'.testing_scope.new_direction_y | map(tostring) | join(" ")')



for distance in $(seq $change_distance_scope)
do
    for speed in $(seq $new_speed_scope)
    do
        for d_x in $(seq $new_direction_x_scope)
        do
            for d_y in $(seq $new_direction_y_scope)
            do
            #先启动carla  # 再启动scenario runner #最后启动自己的程序并且将参数传进去
            # echo "Running iteration i=$i, j=$j"
            if [ $(($count % 10)) -eq 0 ]  
            then
                # pkill -9 CarlaUE4
                # pkill -9 scenario_runner.py
                # pkill -9 ego_vehicle.py

                kill -9 $(lsof -i:$port -t) 2> /dev/null
                kill -9 $(lsof -i:$tm_port -t) 2> /dev/null
                launch_carla & launch_sr_init & launch_ego_vehicle_init $scenario_name $init_speed $init_direction_x $init_direction_y $distance $speed $d_x $d_y $count $temp_folder $result_txt $port
                count=$((count+1))
                echo "distance: $distance, speed: $speed, d_x: $d_x, d_y: $d_y"  
                echo " iterationRunning number: $count"
            else
                # pkill -9 scenario_runner.py
                # pkill -9 ego_vehicle.py
                launch_sr & launch_ego_vehicle $scenario_name $init_speed $init_direction_x $init_direction_y $distance $speed $d_x $d_y $count $temp_folder $result_txt $port
                count=$((count+1))
                echo "distance: $distance, speed: $speed, d_x: $d_x, d_y: $d_y"  
                echo " iterationRunning number: $count"
            fi
            done
        done
    done
done


