std_msgs/Header header

cob_articulation_controller/TrackMsg[] parts    # observed trajectories for each object part 
cob_articulation_controller/ParamMsg[] params   # global parameters
cob_articulation_controller/ModelMsg[] models   # models, describing relationships between parts
visualization_msgs/MarkerArray markers          # marker visualization of models/object 
