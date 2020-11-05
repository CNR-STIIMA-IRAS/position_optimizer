#!/usr/bin/env python
import rospy
import tf.transformations
from geometry_msgs.msg import Pose
import random
import copy
from manipulability_pkg_msgs.srv import pickPlaceManipulability, pickPlaceManipulabilityRequest
import math
import numpy as np


class Individual:

    def __init__(self, id):
        if type(id) == int:
            self.id = str(id)
        elif type(id) == str:
            self.id = id
        else:
            rospy.logerr("id type is : " +type(id) + ", required int or string")
            raise ValueError('wrong id type')

        self.pick_pose_  = self.get_or_error("pick_pose")
        self.place_pose_ = self.get_or_error("place_pose")
        self.chromosome_type = self.get_or_error("chromosome_type")
        self.fitness = float
        self.new_chromosome()
        self.chromosome_to_pose()
        self.rank_probability = 0.0
        self.differonte = 0.0
        self.seed_pick_joints = []
        self.seed_place_joints = []
        self.pick_joints = []
        self.place_joints = []
        self.drift_index = 0.0

        self.manip_service_ns = "pick_place_manipulability_srv"
        rospy.loginfo("waiting for " + rospy.get_namespace() + self.manip_service_ns)
        rospy.wait_for_service(self.manip_service_ns)
        self.pick_place_man_cli = rospy.ServiceProxy(self.manip_service_ns, pickPlaceManipulability)

        self.plan_service_ns = "pick_place_plan_srv"
        rospy.loginfo("waiting for " + rospy.get_namespace() + self.plan_service_ns)
        rospy.wait_for_service(self.plan_service_ns)
        self.pick_place_plan_cli = rospy.ServiceProxy(self.plan_service_ns, pickPlaceManipulability)

    def new_chromosome(self):
        self.chromosome, self.minumum_boud, self.maximum_bound = self.get_chromosome()

    def set_weight(self,weight):
        self.weight = weight

    def discretize(self, param):
        pose_dic = copy.deepcopy(param)
        l_bound = pose_dic["l_bound"]
        u_bound = pose_dic["u_bound"]
        space = pose_dic["linspace"]

        grid = []
        for i in range(0, len(l_bound)):
            grid.append(np.linspace(l_bound[i], u_bound[i], space[i]))

        rtp = []
        for i in range(0, len(l_bound)):
            rtp.append(random.choice(grid[i]))
        return rtp, l_bound, u_bound

    def get_rtp(self, param):
        pose_dic = copy.deepcopy(param)
        l_bound = pose_dic["l_bound"]
        u_bound = pose_dic["u_bound"]

        rtp = []
        for i in range(0, len(l_bound)):
            rtp.append(random.uniform(l_bound[i], u_bound[i]))
        return rtp, l_bound, u_bound

    def get_chromosome(self):
        # crom,  min_crom,  max_crom  = self.get_rtp(self.pick_pose_)
        # crom2, min_crom2, max_crom2 = self.get_rtp(self.place_pose_)
        crom,  min_crom,  max_crom  = self.discretize(self.pick_pose_)
        crom2, min_crom2, max_crom2 = self.discretize(self.place_pose_)
        for i in crom2:
            crom.append(i)
        for i in range(0,len(min_crom2)):
            min_crom.append(min_crom2[i])
            max_crom.append(max_crom2[i])
        return crom, min_crom, max_crom

    def chromosome_to_pose(self):

        if self.chromosome_type == "spherical":

            self.vec_pick_pose = \
                [
                    self.pick_pose_["base"][0] + self.chromosome[0] * math.cos(self.chromosome[1]) * math.cos(self.chromosome[2]),
                    self.pick_pose_["base"][1] + self.chromosome[0] * math.sin(self.chromosome[1]) * math.cos(self.chromosome[2]),
                    self.pick_pose_["base"][2] + self.chromosome[0] * math.sin(self.chromosome[2]),
                    self.pick_pose_["base"][3] + self.chromosome[3],
                    self.pick_pose_["base"][4] + self.chromosome[4],
                    self.pick_pose_["base"][5] + self.chromosome[1] + self.chromosome[5]
                ]

            self.vec_place_pose = \
                [
                    self.place_pose_["base"][0] + self.chromosome[6] * math.cos(self.chromosome[7]) * math.cos(self.chromosome[8]),
                    self.place_pose_["base"][1] + self.chromosome[6] * math.sin(self.chromosome[7]) * math.cos(self.chromosome[8]),
                    self.place_pose_["base"][2] + self.chromosome[6] * math.sin(self.chromosome[8]),
                    self.place_pose_["base"][3] + self.chromosome[9],
                    self.place_pose_["base"][4] + self.chromosome[10],
                    self.place_pose_["base"][5] + self.chromosome[7] + self.chromosome[11]
                ]

        elif self.chromosome_type == "cartesian":
            self.vec_pick_pose = \
                [
                    self.pick_pose_["base"][0] + self.chromosome[0],
                    self.pick_pose_["base"][1] + self.chromosome[1],
                    self.pick_pose_["base"][2] + self.chromosome[2],
                    self.pick_pose_["base"][3] + self.chromosome[3],
                    self.pick_pose_["base"][4] + self.chromosome[4],
                    self.pick_pose_["base"][5] + self.chromosome[5]
                ]

            self.vec_place_pose = \
                [
                    self.place_pose_["base"][0] + self.chromosome[6],
                    self.place_pose_["base"][1] + self.chromosome[7],
                    self.place_pose_["base"][2] + self.chromosome[8],
                    self.place_pose_["base"][3] + self.chromosome[9],
                    self.place_pose_["base"][4] + self.chromosome[10],
                    self.place_pose_["base"][5] + self.chromosome[11]
                ]
        else:
            raise ValueError("Chromosome type is not rtp nor xyz")

        self.vec_pick_pose_offset = \
            [
                self.pick_pose_["offset"][0],
                self.pick_pose_["offset"][1],
                self.pick_pose_["offset"][2],
                self.pick_pose_["offset"][3],
                self.pick_pose_["offset"][4],
                self.pick_pose_["offset"][5]
            ]

        self.vec_place_pose_offset = \
            [
                self.place_pose_["offset"][0],
                self.place_pose_["offset"][1],
                self.place_pose_["offset"][2],
                self.place_pose_["offset"][3],
                self.place_pose_["offset"][4],
                self.place_pose_["offset"][5]
            ]

        self.pick_pose = self.vec_to_pose(self.vec_pick_pose, self.vec_pick_pose_offset)
        self.place_pose = self.vec_to_pose(self.vec_place_pose, self.vec_place_pose_offset)

    def mutate(self):
        i = random.randint (0, len(self.chromosome)-1)
        self.chromosome[i] = random.uniform(self.minumum_boud[i], self.maximum_bound[i])


    def vec_to_pose(self, vector, offset):
        if len(vector) != 6:
            raise ValueError('length required is 6, vector size is : '+str(len(vector)))
        if len(offset) != 6:
            raise ValueError('length required is 6, vector size is : '+str(len(offset)))

        q = tf.transformations.quaternion_from_euler(vector[3] ,vector[4] ,vector[5] )
        trans_mat = tf.transformations.translation_matrix([vector[0], vector[1], vector[2]])
        rot_mat   = tf.transformations.quaternion_matrix(q)
        transform = np.dot(trans_mat,rot_mat)

        # TODO:: gestione dell' offset, se lo metto cosi non appaiono giusti gli oggetti
        q_off = tf.transformations.quaternion_from_euler  (offset[3] ,offset[4] ,offset[5] )
        trans_mat_off = tf.transformations.translation_matrix([offset[0], offset[1], offset[2]])
        rot_mat_off   = tf.transformations.quaternion_matrix(q_off)
        transform_off = np.dot(trans_mat_off,rot_mat_off)

        tf_fin = np.dot(transform,transform_off)
        # return self.tf_to_pose(tf_fin)

        #

        return self.tf_to_pose(transform)

    def tf_to_pose(self,transform):

        p = Pose()
        p.position.x = tf.transformations.translation_from_matrix(transform)[0]
        p.position.y = tf.transformations.translation_from_matrix(transform)[1]
        p.position.z = tf.transformations.translation_from_matrix(transform)[2]

        p.orientation.x = tf.transformations.quaternion_from_matrix(transform)[0]
        p.orientation.y = tf.transformations.quaternion_from_matrix(transform)[1]
        p.orientation.z = tf.transformations.quaternion_from_matrix(transform)[2]
        p.orientation.w = tf.transformations.quaternion_from_matrix(transform)[3]

        return p

    def set_pick_place_fitness(self):

        req = pickPlaceManipulabilityRequest()
        req.pick_pose  = self.pick_pose
        req.pick_name  = self.pick_pose_["id"]
        req.place_pose = self.place_pose
        req.place_name = self.place_pose_["id"]
        if len(self.seed_pick_joints) > 1:
            req.seed_pick_joints = self.seed_pick_joints
        if len(self.seed_place_joints) > 1:
            req.seed_place_joints = self.seed_place_joints

        resp = self.pick_place_man_cli(req)

        if resp.success is not True:
            rospy.logerr("error in checking the pose retrying")

            self.fitness = 0.0001
            self.pick_manipulability  = 0.0001
            self.place_manipulability = 0.0001
            return False
        else:
            self.fitness = self.weight[0] * resp.manipulability[0] + self.weight[1] * resp.manipulability[1]

            self.pick_manipulability  = resp.manipulability[0]
            self.place_manipulability = resp.manipulability[1]

            self.pick_joints = resp.pick_joints
            self.place_joints = resp.place_joints

            self.seed_pick_joints = resp.pick_joints
            self.seed_place_joints = resp.place_joints

            return True

    def planTrajectory(self, precomputed = True):

        req = pickPlaceManipulabilityRequest()
        req.pick_pose  = self.pick_pose
        req.pick_name  = self.pick_pose_["id"]
        req.place_pose = self.place_pose
        req.place_name = self.place_pose_["id"]
        req.precomputed_path = precomputed
        if len(self.seed_pick_joints) > 1:
            req.seed_pick_joints = self.seed_pick_joints
        if len(self.seed_place_joints) > 1:
            req.seed_place_joints = self.seed_place_joints

        resp = self.pick_place_plan_cli(req)

        if resp.success is not True:
            rospy.logerr("error in planning the pose retrying")
            return False
        else:
            self.fitness = self.weight[0] * resp.manipulability[0] + self.weight[1] * resp.manipulability[1]
            self.pick_manipulability  = resp.manipulability[0]
            self.place_manipulability = resp.manipulability[1]

            self.pick_joints = resp.pick_joints
            self.place_joints = resp.place_joints
            ###### manipolabilita fion qui

            ####altrimenti altro criterio da decidere
            # self.drift_index = resp.drift_index
            # print '>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><drift index::' + str(self.drift_index)
            # raw_input()
            ####altrimenti altro criterio da decidere

            return True

    def set_gen_n(self, gen):
        self.gen_n = gen

    def get_fitness(self):
        return self.fitness

    def set_probability(self, prob):
        self.prob = prob

    def get_probability(self):
        return self.prob

    def get_or_error(self,string):
        if rospy.has_param(string):
            return rospy.get_param(string)
        else:
            rospy.logerr("%s/%s not found ! exit", rospy.get_namespace(), string)


if __name__ == "__main__":






    xx = np.linspace(0,10,50)
    yy = np.linspace(0,10,50)

    e = []
    e.append( xx )
    e.append( yy )


    print(random.choice(e[0]))
    print(random.choice(e[1]))