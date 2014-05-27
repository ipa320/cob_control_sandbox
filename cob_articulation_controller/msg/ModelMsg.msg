# Single kinematic model
#
# A kinematic model is defined by its model class name, and a set of parameters. 
# The client may additionally specify a model id, that can be used to colorize the
# model when visualized using the RVIZ model display.
# 
# For a list of currently implemented models, see the documetation at
# http://www.ros.org/wiki/articulation_models
#
#

std_msgs/Header header                         # frame and timestamp

int32 id                                       # user specified model id
string name                                    # name of the model family (e.g. "rotational",
                                               # "prismatic", "pca-gp", "rigid")
cob_articulation_controller/ParamMsg[] params  # model parameters
cob_articulation_controller/TrackMsg track     # trajectory from which the model is estimated, or
                                               # that is evaluated using the model
