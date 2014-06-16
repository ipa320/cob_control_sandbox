#!/usr/bin/env python

import roslib; roslib.load_manifest('cob_articulation_controller')
import rospy
import actionlib

import StringIO
import sys
import os
import tf
import tf_conversions
import PyKDL


from cob_articulation_controller.msg import *
from cob_articulation_controller.srv import *
from cob_srvs.srv import *

class cob_articulation_models_prior(object):
    def __init__(self):
        try:
            # wait for services
            rospy.wait_for_service('collector_toggle', 5)
            rospy.wait_for_service('model_select_eval', 5)
            rospy.wait_for_service('model_store', 5)
            rospy.wait_for_service('model_prior_set', 5)
            rospy.wait_for_service('model_prior_get', 5)
            rospy.wait_for_service('/mm/start', 5)
            rospy.loginfo("Services OK")
        except:
            rospy.logerr("Service(s) not found")
            rospy.signal_shutdown("Missing services")

        # action clients
        self.moveModel_ac = actionlib.SimpleActionClient('moveModel', ArticulationModelAction)
        self.moveModel_ac.wait_for_server()

        # service clients
        self.toggle_collector = rospy.ServiceProxy('collector_toggle', CartCollectorPrior) #?? change into action!?
        self.eval_model = rospy.ServiceProxy('model_select_eval', TrackModelSrv)
        self.store_model = rospy.ServiceProxy('model_store', TrackModelSrv)
        self.set_prior = rospy.ServiceProxy('model_prior_set', SetModelPriorSrv)
        self.get_prior = rospy.ServiceProxy('model_prior_get', GetModelPriorSrv)
        self.start_mm = rospy.ServiceProxy('/mm/start', Trigger)

        # variables
        self.prior_changed = False

        #tf listener
        self.listener = tf.TransformListener()


    ######################################################
    # output methods
    def print_parameter(self, model_id, name, value):
        print ("ID: " + str(model_id)).ljust(7), ("NAME: " + name).ljust(30), ("VALUE: " + str(value)).ljust(30)


    def filter_parameters(self, models, param_name):
        for n in range(len(models[0].params)):
            for param in param_name:
                if param in models[0].params[n].name:
                    for model in models:
                        if model.name == models[-1].name:
                            self.print_parameter(model.id, model.params[n].name, model.params[n].value)
                    print 75*"-"


    def print_model(self, model):
        print ("ID: " + str(model.id)).ljust(7), ("NAME: " + model.name).ljust(20), ("POSES: " + str(len(model.track.pose))).ljust(30)


    def print_prior_models(self):
        # print prior models
        for model in self.get_prior_models().model:
            self.print_model(model)


    def print_models_verbose(self, models):
        # filter out relevant models
        if models[-1].id != -1: # if learned model updades a prior model
            keep = []
            # keep prior model with same id as learned one
            for k in range(len(models)):
                if models[k].id == models[-1].id:
                    keep.append(models[k])
            models = keep

        # prints all evaluation parameters of all given models
        for n in range(len(models[0].params)):
            if models[0].params[n].type == 2:
                for model in models:
                    self.print_parameter(model.id, model.params[n].name, model.params[n].value)
                print 75*"-"

        # prints informative articulation parameter 
        if models[-1].name == "rotational": # for rotational articulations
            self.filter_parameters(models, ["rot_center", "rot_radius"])
        elif models[-1].name == "prismatic": # for prismatic articulations
            self.filter_parameters(models, ["rigid_position", "rigid_orientation", "prismatic_dir"])
        else: # for rigid articulation
            self.filter_parameters(models, ["rigid_position", "rigid_orientation"])


    ######################################################
    # methods for user interaction
    def query(self, question, choises):
        # let user choose 
        while True:
            choise = raw_input(question + ' [' + '/'.join(choises) + ']: ').lower()
            if choise not in choises:
                print "Invalid choise!"
            else:
                return choise


    def query_database(self, load=False):
        # let user enter prior database file
        valid_file = False
        while not valid_file:
            database = raw_input("Please enter database file name: ")
            if load:
                if os.path.isfile(database):
                    valid_file = True
                else:
                    print "File not found, please enter name of existing file to load prior from"
            else:
                if os.path.isfile(database):
                    print "%s is an existing file and will be overwritten"%database
                else:
                    print "File %s not found. Will create new file"%database
                valid_file = True
        return database


    def query_articulation_parameters(self):
        goal = ArticulationModelGoal()
        goal.model.id = -1
        if self.query("What kind of articulation should be generated? Rotational or prismatic?", ['r', 'p']) == 'r':
            goal.model.name = 'rotational'
            if self.query("Do you want to enter the parameters in a simple or complex way?", ['s', 'c']) == 's':
                # initialize F_articulation
                F_articulation_EE = PyKDL.Frame.Identity()
                print "It is believed that the robot is grasping the articulated objects handle right now!"
                # get transform from /map to /sdh_tip_link
                try:
                    F_sdh_tip = tf_conversions.posemath.fromTf(self.listener.lookupTransform('/map', '/sdh_tip_link', rospy.Time(0)))
                except (tf.LookupException, tf.ConnectivityException):
                    rospy.logerr("Could not look up transformation")
                    raise Exception("Could not look up transformation")

                print "Please enter the distance between the handle and the rotational axis in x- and y-direction (in sdh_tip_link coordinates):"
                dist_x = self.query_parameter('distance in x', 'm')
                dist_y = self.query_parameter('distance in y', 'm')
                handle_orient = self.query("Is the handle parallel or orthogonal to the rotational axis?", ['p', 'o'])

                F_articulation_EE.p.x(dist_x)
                F_articulation_EE.p.y(dist_y)

                if handle_orient == 'p':
                    F_articulation_EE.M = PyKDL.Rotation.Quaternion(0.7071, 0, 0, 0.7071)
                else:
                    F_articulation_EE.M = PyKDL.Rotation.Quaternion(0, 0.7071, 0, 0.7071)


                F_articulation = F_sdh_tip*F_articulation_EE
                (x, y, z, w) = F_articulation.M.GetQuaternion()

                goal.model.params.append(ParamMsg('rot_center.x', F_articulation.p.x(), 1))
                goal.model.params.append(ParamMsg('rot_center.y', F_articulation.p.y(), 1))
                goal.model.params.append(ParamMsg('rot_center.z', F_articulation.p.z(), 1))
                goal.model.params.append(ParamMsg('rot_axis.x', x, 1))
                goal.model.params.append(ParamMsg('rot_axis.y', y, 1))
                goal.model.params.append(ParamMsg('rot_axis.z', z, 1))
                goal.model.params.append(ParamMsg('rot_axis.w', w, 1))

            else:
                goal.model.params.append(ParamMsg('rot_center.x', self.query_parameter('rot_center.x'), 1))
                goal.model.params.append(ParamMsg('rot_center.y', self.query_parameter('rot_center.y'), 1))
                goal.model.params.append(ParamMsg('rot_center.z', self.query_parameter('rot_center.z'), 1))
                goal.model.params.append(ParamMsg('rot_axis.x', self.query_parameter('rot_axis.x'), 1))
                goal.model.params.append(ParamMsg('rot_axis.y', self.query_parameter('rot_axis.y'), 1))
                goal.model.params.append(ParamMsg('rot_axis.z', self.query_parameter('rot_axis.z'), 1))
                goal.model.params.append(ParamMsg('rot_axis.w', self.query_parameter('rot_axis.w'), 1))

            goal.model.params.append(ParamMsg('action', self.query_parameter('angle', 'rad'), 1))
            goal.target_duration.secs = self.query_parameter('target_duration', 'sec')
        else:
            print "It is assumed that the robot is grasping the articulated objects handle right now!"
            goal.model.name = 'prismatic'
            goal.model.params.append(ParamMsg('action', self.query_parameter('opening length', 'm'), 1))
            goal.model.params.append(ParamMsg('rigid_position.x', 0.0, 1)) #self.query_parameter('rigid_position.x'), 1))
            goal.model.params.append(ParamMsg('rigid_position.y', 0.0, 1)) #self.query_parameter('rigid_position.y'), 1))
            goal.model.params.append(ParamMsg('rigid_position.z', 0.0, 1)) #self.query_parameter('rigid_position.z'), 1))
            goal.model.params.append(ParamMsg('rigid_orientation.x', 0.0, 1)) #self.query_parameter('rigid_orientation.x'), 1))
            goal.model.params.append(ParamMsg('rigid_orientation.y', 0.0, 1)) #self.query_parameter('rigid_orientation.y'), 1))
            goal.model.params.append(ParamMsg('rigid_orientation.z', 0.0, 1)) #self.query_parameter('rigid_orientation.z'), 1))
            goal.model.params.append(ParamMsg('rigid_orientation.w', 1.0, 1)) #self.query_parameter('rigid_orientation.w'), 1))
            print "Enter now the direction in which the prismatic articulation can be moved (in global coordinates)"
            goal.model.params.append(ParamMsg('prismatic_dir.x', self.query_parameter('prismatic_dir.x'), 1))
            goal.model.params.append(ParamMsg('prismatic_dir.y', self.query_parameter('prismatic_dir.y'), 1))
            goal.model.params.append(ParamMsg('prismatic_dir.z', self.query_parameter('prismatic_dir.z'), 1))
            goal.target_duration.secs = self.query_parameter('target_duration')

        return goal


    def query_parameter(self, param_name, param_unit=""):
        prompt = "Enter float value for parameter '%s': "%param_name
        if param_unit != "":
            prompt = "Enter float value for parameter '%s' in [%s]: "%(param_name, param_unit)
        param_value = float(raw_input(prompt))
        return param_value


    ######################################################
    # load, save and store methods
    def get_prior_model_by_id(self, model_id):
        prior_models = self.get_prior_models()
        for prior_model in prior_models.model:
            if prior_model.id == model_id:
                return prior_model
        return None

    def get_prior_models(self):
        # get prior from model_learner_prior
        request = GetModelPriorSrvRequest()

        try:
            response = self.get_prior(request)
        except rospy.ServiceException, e:
            rospy.logerr("Failed to get prior models: %s"%e)
        return response


    def store_model_to_prior(self, model):
        # adds a learned model to the prior models
        try:
            store_request = TrackModelSrvRequest()
            store_request = model
            store_response = self.store_model(store_request)
            self.prior_changed = True
        except rospy.ServiceException, e:
            self.prior_changed = False
            rospy.logerr("Failed to store learned model in prior models: %s"%e)
        return self.prior_changed


    def load_prior_from_database(self, database):
        # load prior from database and set up model_learner_prior node
        request = SetModelPriorSrvRequest()
        try:
            with open(database, 'r') as db_handle:
                saved_prior = db_handle.read()
                request.deserialize(saved_prior)
                response = self.set_prior(request)
                rospy.loginfo("%d prior model(s) loaded from %s", len(request.model), database)
        except rospy.ServiceException, e:
            rospy.logerr("Failed to set prior models: %s"%e)
            raise
        except IOError, e:
            rospy.logerr("Failed to read prior database: %s"%e)
            raise


    def save_prior_to_database(self, database):
        # get prior from model_learner_prior and save into database
        try:
            response = self.get_prior_models()
            output = StringIO.StringIO()
            response.serialize(output)
            with open(database, "w") as dh_handle:
                dh_handle.write(output.getvalue())
            output.close()
        except (IOError, UnicodeError), e:
            rospy.logerr("Failed to write prior models to database: %s"%e)
            


    ######################################################
    # cartcollector methods
    def cartcollector_start(self):
        self.cartcollector_toggle()
        return

    
    def cartcollector_stop(self):
        return self.cartcollector_toggle()


    def cartcollector_toggle(self):
        try:
            cartcoll_request = CartCollectorPriorRequest()
            cartcoll_response = self.toggle_collector(cartcoll_request)
        except rospy.ServiceException, e:
            rospy.logerr("Toggle cartcollector service failed: %s"%e)
            raise
        
        return cartcoll_response



    ######################################################
    # evaluation method
    def evaluate_model(self, model):
        request = TrackModelSrvRequest()
        request.model = model
        try:
            return self.eval_model(request.model)
        except rospy.ServiceException, e:
            rospy.loginfo("Model evaluation service failed, trying service call again")
            return self.eval_model(track_model)


    def compare_evaluation(self, prior_model, learned_model):
        try:
            prior_response = self.evaluate_model(prior_model).model
            learned_response = self.evaluate_model(learned_model).model
            if self.get_parameter_value(learned_response, 'bic') <= self.get_parameter_value(prior_response, 'bic'):
                return learned_response, "learned"
            else:
                return prior_response, "prior"

        except rospy.ServiceException, e:
            rospy.logerr("Model evaluation service failed: %s"%e)
            rospy.signal_shutdown("Evaluation Failed")
        except LookupError, e:
            rospy.logerr(e)
            rospy.signal_shutdown("Evaluation Failed")


    def get_parameter_value(self, model, param_name):
        for parameter in model.params:
            if parameter.name == param_name:
                return parameter.value
        raise LookupError("Parameter not %s found"%param_name)


    def set_parameter_value(self, model, param_name, param_value):
        for parameter in model.params:
            if parameter.name == param_name:
                parameter.value = param_value
        raise LookupError("Parameter not %s found"%param_name)


