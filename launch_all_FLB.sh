#!/bin/bash

scenario_name="FollowLeadingVehicle" 
scenario_config="FollowLeadingVehicle"
temp_folder="all_temp_data/temp_data1"
result_txt="result_txt/FLV.txt"
port=2000
tm_port=8000
graphics_num=0




count=0
launch_carla(){
        sh carla/CarlaUE4.sh -windowed -carla-port=$port -graphicsadapter=$graphics_num
    }

launch_sr_init(){
        sleep 20
        python scenario_runner.py --openscenario srunner/examples/$scenario_name.xosc --record $temp_folder --port=$port --trafficManagerPort=$tm_port
    }
    
launch_ego_vehicle_init(){
        sleep 30
        sh ego_vehicle.py $scenario_name $init_speed $init_direction_x $init_direction_y $distance $speed $d_x $d_y $count $temp_folder $result_txt $port
}


launch_sr(){
        sleep 10
        python scenario_runner.py --openscenario srunner/examples/$scenario_name.xosc --record $temp_folder --port=$port --trafficManagerPort=$tm_port
    }
    
launch_ego_vehicle(){
        sleep 20
        sh ego_vehicle.py $scenario_name $init_speed $init_direction_x $init_direction_y $distance $speed $d_x $d_y $count $temp_folder $result_txt $port
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
            if [ $(($count % 10)) -eq 0 ]  
            then

                kill -9 $(lsof -i:$port -t) 2> /dev/null
                kill -9 $(lsof -i:$tm_port -t) 2> /dev/null
                echo "-------------------------------------------------------------"
                launch_carla & launch_sr_init & launch_ego_vehicle_init $scenario_name $init_speed $init_direction_x $init_direction_y $distance $speed $d_x $d_y $count $temp_folder $result_txt $port
                count=$((count+1))
            else
                # pkill -9 scenario_runner.py
                # pkill -9 ego_vehicle.py
                echo "-------------------------------------------------------------"
                launch_sr & launch_ego_vehicle $scenario_name $init_speed $init_direction_x $init_direction_y $distance $speed $d_x $d_y $count $temp_folder $result_txt $port
                count=$((count+1))
            fi
            done
        done
    done
done


