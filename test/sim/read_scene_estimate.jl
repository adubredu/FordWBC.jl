using Revise
using FordWBC
using FordWBC.DigitInterface

ip = robot_ip
host=:real

publisher_address = ip
llapi_init(publisher_address)    
observation = llapi_observation_t()
command = llapi_command_t()  
observation = llapi_observation_t() 
connect_to_robot(observation, command) 

q, _, _ = get_generalized_coordinates(observation) 
scene_estimate = read_scene_estimate(q)
top_object_pose = get_top_object_pose(scene_estimate) 
com_z, pitch = compute_height_and_pitch(top_object_pose)
