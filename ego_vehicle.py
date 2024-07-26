from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import weakref
import time
import config as config_ori
import json
import shutil
import numpy as np 
import torch 

from multiprocessing import Process, Manager
import subprocess

from pytorch3d.ops import box3d_overlap
import carla
from carla import ColorConverter as cc

from agents.navigation.behavior_agent import BehaviorAgent  
from agents.navigation.basic_agent import BasicAgent  


ego_vehicle_collision_indicator=False

def get_carla_transform(loc_rot_tuples):
    """
    Convert loc_rot_tuples = ((x, y, z), (roll, pitch, yaw)) to
    carla.Transform object
    """

    if loc_rot_tuples is None:
        return None

    loc = loc_rot_tuples[0]
    rot = loc_rot_tuples[1]

    t = carla.Transform(
        carla.Location(loc[0], loc[1], loc[2]),
        carla.Rotation(roll=rot[0], pitch=rot[1], yaw=rot[2])
    )

    return t


def dst_folder_confirm(path):
    if not os.path.exists(path):
        os.makedirs(path)

def precise_detector(ego_vehicle,npc):
    incon_collision_vehicle=False
    
    
    ego_bbox=ego_vehicle.bounding_box.get_world_vertices(ego_vehicle.get_transform())
    ego_bbox_coor=[]
    for i in range(0,8):
        ego_bbox_coor.append([ego_bbox[i].x,ego_bbox[i].y,ego_bbox[i].z])
    ego_bbox_coor=np.array(ego_bbox_coor)        


    bb_coor=[]
    bb = npc.bounding_box
    bb_world_coor=bb.get_world_vertices(npc.get_transform())                
    for item in bb_world_coor:
        bb_coor.append([item.x, item.y, item.z])        
    bb_coor=np.array(bb_coor)
    intersection_vol,iou3d=box3d_overlap((torch.from_numpy(bb_coor.astype(np.float32))).unsqueeze(0),(torch.from_numpy(ego_bbox_coor.astype(np.float32))).unsqueeze(0))
    # iou3d=box3d_iou(ego_bbox_coor,bb_coor) 
    ev_loc=ego_vehicle.get_transform().location
    npc_loc=npc.get_transform().location
    if ev_loc.x == 0 and ev_loc.y == 0 and ev_loc.z == 0 and npc_loc.x ==0 and npc_loc.y == 0 and npc_loc.z ==0:
        incon_collision_vehicle=False
        return incon_collision_vehicle  
    else:
        if iou3d>0 or intersection_vol > 0:
            incon_collision_vehicle=True
        
    return incon_collision_vehicle        


def _on_collision(event,args,config,ego_vehicle_collision):
    global ego_vehicle_collision_indicator
    if event.other_actor.type_id != "static.road":
        if len(event.other_actor.attributes) != 0 and (event.other_actor.type_id == "vehicle" or event.other_actor.type_id=="pedestrian"):
            ego_vehicle_collision_indicator=True
            
            ego_vehicle_collision={
                "collision:":True,
                "collision with:": event.other_actor.type_id,
                "collision with 2:":event.other_actor.id}
        with open(os.path.join(args.temp_folder,"ego_vehicle_collision.json"),'w', encoding='utf-8') as fp:
            json.dump(ego_vehicle_collision, fp, sort_keys=False, indent=4)

def _on_collision_other(event,args,config,other_vehicle_collision):
    if event.other_actor.type_id != "static.road":
        if len(event.other_actor.attributes) != 0:
            other_vehicle_collision={
                "collision:":True,
                "collision with:": event.other_actor.type_id,
                "collision with 2:":event.other_actor.id}
        with open(os.path.join(args.temp_folder,"other_vehicle_collision.json"),'w', encoding='utf-8') as fp:
            json.dump(other_vehicle_collision, fp, sort_keys=False, indent=4)



def save_scenario(config,args, scenario_time,all_fail_dir=None, ego_fail_dir=None, other_fail_dir=None, inconsistency_dir=None, all_fail=None,ego_fail=None, other_fail=None,collision_inconsistency=None):
    """_summary_

    Args:
        all_fail (_type_, optional): both ego vehicle and other vehicle detect the collision. Defaults to None.
        ego_fail (_type_, optional): collision, only ego vehicle detected. Defaults to None.
        other_fail (_type_, optional): collision, only other vehicle detected. Defaults to None.
    """    
    if all_fail:
        dst_folder=os.path.join(all_fail_dir, scenario_time)
    if ego_fail:
        dst_folder=os.path.join(ego_fail_dir, scenario_time)
    if other_fail:
        dst_folder=os.path.join(other_fail_dir, scenario_time)
    if collision_inconsistency:
        dst_folder=os.path.join(inconsistency_dir, scenario_time)
    
    os.makedirs(dst_folder)
    for dirpath, dirnames,filenames in os.walk(args.temp_folder):
        for op_file in filenames:
            src_op_path=os.path.join(args.temp_folder,op_file)
            dst_op_path=os.path.join(dst_folder,op_file)
            shutil.move(src_op_path, dst_op_path)
            
    return



def collision_confirm(args,config,scenario_name, scenario_time,inconsistency_dir,precise_collision):
    with open(os.path.join(args.temp_folder,scenario_name+".json")) as f: 
        data = json.load(f)
        ego_vehicle_collision = data["CollisionTest"]
        other_vehicle_collision= data["NpcTest"]

        
        if (other_vehicle_collision['collision_with_other_actor']==False and ego_vehicle_collision_indicator==False) and precise_collision==True:
            save_scenario(config, args,scenario_time, inconsistency_dir=inconsistency_dir, collision_inconsistency=True)
            print("*******************************Inconsistency found ********************************")
            return "[collision inconsistency]"
        
        if precise_collision==True and other_vehicle_collision['collision_with_other_actor']==True and ego_vehicle_collision["collision_with_other_actor"] == True:
            # save_scenario(config, scenario_time, inconsistency_dir=inconsistency_dir, collision_inconsistency=True)
            print("*******************************collision and all detected********************************")
            return "collision and all detected"
        
        if precise_collision==False and other_vehicle_collision['collision_with_other_actor']==False and ego_vehicle_collision["collision_with_other_actor"] == False:
            return "no collision"
        

def ego_vehicle(args,config):
    actor_list =  []
    player = None
    other_actor=None
    ego_vehicle_collision={}
    other_vehicle_collision={}
    precise_collision_indicator=False
    try:
        try:
            #create client
            client = carla.Client('localhost', args.port)
            client.set_timeout(2.0)
            #world connection
            world = client.get_world() 

            
            blueprint_library = world.get_blueprint_library()
            bp = blueprint_library.filter("model3")[0]

            
            # actual monitoring of the driving simulation begins here
            snapshot0 = world.get_snapshot()
            first_frame_id = snapshot0.frame
            first_sim_time = snapshot0.timestamp.elapsed_seconds

            
            spawn_point = random.choice(world.get_map().get_spawn_points())
            
            print("waiting for ego vehicle ....")
            possible_vehicles = world.get_actors().filter('vehicle.*')
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == 'hero':
                    print("Ego vehicle found")
                    player = vehicle
                    continue
                if vehicle.attributes['role_name'] == 'adversary':
                    print("Other vehicle found")
                    other_actor=vehicle
                    continue
            if other_actor==None:
                possible_walkers=world.get_actors().filter('walker.*')
                for walker in possible_walkers:
                    if walker.attributes['role_name'] == 'adversary':
                        print("Other walker found")
                        other_actor=walker
                        continue

            if player is not None:       
                if config.manual:
                    agent=player
                    agent.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
                else:
                    if config.agent=="Basic": 
                        agent=BasicAgent(player) 
                    elif config.agent=="Behavior": 
                        agent=BehaviorAgent(player, behavior=config.behavior)
                    
                    elif config.agent=="Maneuver":
                        player=player
                        
                spectator=world.get_spectator()
                actor_list.append(player)
                
                
                collision_bp = blueprint_library.find('sensor.other.collision')
                sensor_collision = world.spawn_actor(collision_bp, carla.Transform(),attach_to=player)
                                                    
                sensor_collision.listen(lambda event: _on_collision(event,args,config,ego_vehicle_collision))
                actor_list.append(sensor_collision)


                collision_bp1 = blueprint_library.find('sensor.other.collision')
                sensor_collision1 = world.spawn_actor(collision_bp1, carla.Transform(),attach_to=other_actor)
                sensor_collision1.listen(lambda event: _on_collision_other(event,args,config,other_vehicle_collision))
                actor_list.append(sensor_collision1)
                
                
                
                        
                while True:
                    #maybe need to set the frame rate
                    snapshot=world.get_snapshot()
                    cur_frame_id=snapshot.frame
                    cur_sim_time = snapshot.timestamp.elapsed_seconds

                    #Change it to a public variable
                    num_frames=cur_frame_id-first_frame_id
                    
                    
                    if config.agent == 'Basic' or config.agent == 'Behavior':
                        player.apply_control(agent.run_step())
                    elif config.agent=="Maneuver":
                        
                        ego_location = player.get_location()
                        other_location = other_actor.get_location()

                        # Calculate the distance
                        distance = math.sqrt((ego_location.x - other_location.x)**2 + 
                                            (ego_location.y - other_location.y)**2 + 
                                            (ego_location.z - other_location.z)**2)
                        
                        transform=player.get_transform()
                        init_direction= carla.Vector3D(x=args.init_direction_x ,y=args.init_direction_y ,z=0)  #set the init direction, init speed, new_speed, new_direction as mutated parameters
                        new_direction=carla.Vector3D(x=args.d_x,y=args.d_y,z=0.0)
                        
                        
                        
                        player.set_target_velocity(init_direction*args.init_speed)
                        
                        if distance < args.distance:
                            
                            player.set_target_velocity(new_direction*args.speed) 
                            precise_collision=precise_detector(player,other_actor)
                            if precise_collision==True:
                                precise_collision_indicator=True
                                precise_collision_dict={
                                    "yes":True,
                                    "precise_collision":precise_collision
                                } 
                                
                                
                                with open(os.path.join(args.temp_folder,"precise_collision.json"),'w', encoding='utf-8') as fp:
                                    json.dump(precise_collision_dict, fp, sort_keys=False, indent=4)
                    transform=player.get_transform() 
                    spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),carla.Rotation(pitch=-90)))
                    if num_frames >2000:
                        return actor_list,"end",precise_collision_indicator
                    if config.sync:
                        world.tick()

                        
                    else:
                        world.wait_for_tick()
            
            else:
                print("Cannot find the ego vehicle")
                
        finally:
            pass
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

    
def main():
    """
    main function
    """
 
    argparser = argparse.ArgumentParser(
        description='CARLA Ego Vehicle Controller')
    
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Synchronous mode execution')
    argparser.add_argument(
        '-m',"--manual",type=bool,
        default=False,
        help='manually allocate the running command')
    
    argparser.add_argument(
        "-a", "--agent", type=str,
        choices=["Behavior", "Basic","Maneuver"],
        help="select which agent to run",
        default="Basic")
    
    argparser.add_argument(
        '-b', '--behavior', type=str,
        choices=["cautious", "normal", "aggressive"],
        help='Choose one of the possible agent behaviors (default: normal) ',
        default='normal')

    argparser.add_argument(
        'scenario_name', type= str,
        help='scenario name in scenario runner',
        default='FollowLeadingVehicle')
    
    argparser.add_argument(
        'init_speed', type=float,
        help='init speed of the ego vehicle',
        default='5')
    
    argparser.add_argument(
        'init_direction_x', type=float,
        help='the initial direction x of the ego vehicle',
        default='1')
    
    argparser.add_argument(
        'init_direction_y', type=float,
        help='the initial direction y of the ego vehicle',
        default='0')
    
    argparser.add_argument(
        'distance', type= float,
        help='distance when the ego vehicle begin to turn',
        default='5')
    
    argparser.add_argument(
        'speed', type=float,
        help='the turn speed of the ego vehicle',
        default='5')
    
    argparser.add_argument(
        'd_x', type=float,
        help='the turn direction x of the ego vehicle',
        default='1')
    
    argparser.add_argument(
        'd_y', type=float,
        help='the turn direction y of the ego vehicle',
        default='0')
    
    argparser.add_argument(
        'count', type=int,
        help='the whole running times of testing',
        default='0')

    argparser.add_argument(
        'temp_folder', type=str,
        help='the temp folder path to store the data for each generation',
        default='all_temp_data/temp_data')
    


    argparser.add_argument(
        'result_txt', type=str,
        help='the name of result txt file to store the final running result',
        default='result_txt/fuzzing_result_precise_detector.txt')

    argparser.add_argument(
        'port', type=int,
        default='2000',
        help='TCP port to listen to (default: 2000)')
    
    args=argparser.parse_args()
    config=config_ori.Config()


    print("testing times:",args.count)
    print("Collision Distance:",args.distance,"Collision Speed:",args.speed,"Collision Angle x:",args.d_x,"Collision Angle y:",args.d_y)
    
    
    scenario_name=args.scenario_name
    
    all_fail_dir=os.path.join(os.path.join(config.final_out_dir,scenario_name),"all_collision")
    ego_fail_dir=os.path.join(os.path.join(config.final_out_dir,scenario_name),"ego_collision")
    other_fail_dir=os.path.join(os.path.join(config.final_out_dir,scenario_name),"other_actor_collision")
    inconsistency_dir=os.path.join(os.path.join(config.final_out_dir,scenario_name),"collision_inconsistency")

    
    dst_folder_confirm(all_fail_dir)
    dst_folder_confirm(ego_fail_dir)
    dst_folder_confirm(other_fail_dir)
    dst_folder_confirm(inconsistency_dir)
    
    
    actor_list, state, precise_collision_indicator=ego_vehicle(args,config)
    if state=="end":
        for actor in actor_list:
            try:
                actor.destroy()
            except:
                print("All cleaned up!")
        
    
    
    scenario_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    config_save = {
        "scenario_name":scenario_name,
        "init_speed":args.init_speed,
        "new_speed":args.speed, 
        "change_distance":args.distance,
        "init_direction_x":args.init_direction_x,
        "init_direction_y":args.init_direction_y,
        "new_direction_x":args.d_x,
        "new_direction_y":args.d_y,
        "weather":config.weather 
            }
    with open(os.path.join(args.temp_folder,"config.json"), "w") as f:
        json.dump(config_save,f, indent=4)
    
    time.sleep(3)
    
    collision_result=collision_confirm(args,config, scenario_name, scenario_time, all_fail_dir,ego_fail_dir, other_fail_dir,inconsistency_dir, precise_collision_indicator)
    print("Collision Result:",collision_result)
    
    
    
    with open(args.result_txt,'a') as f:
        f.write(f"running time{scenario_time}, \ntesting times: {args.count}, init speed is: {args.init_speed},init_x is:{args.init_direction_x}, init_y_is: {args.init_direction_y}, change distance is: {args.distance}, change_speed is: {args.speed}, direction x is: {args.d_x}, direction y is: {args.d_y},collision result is: {collision_result}")
        f.write('\n')
        f.write('\n')
    

if __name__ == '__main__':
    main()

