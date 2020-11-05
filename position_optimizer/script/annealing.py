#!/usr/bin/env python
import rospy
import random
import copy
import numpy as np
import math

def random_crom(crom, base_crom, pick = True):
    if pick:
        delta = rospy.get_param("delta_pick")
        crom[0] = base_crom[0] + random.uniform(-delta[0], delta[0])
        crom[1] = base_crom[1] + random.uniform(-delta[1], delta[1])
        crom[2] = base_crom[2] + random.uniform(-delta[2], delta[2])
        crom[3] = base_crom[3] + random.uniform(-delta[3], delta[3])
        crom[4] = base_crom[4] + random.uniform(-delta[4], delta[4])
        crom[5] = base_crom[5] + random.uniform(-delta[5], delta[5])
    else:
        delta = rospy.get_param("delta_place")
        crom[6]  = base_crom[6]  + random.uniform(-delta[0], delta[0])
        crom[7]  = base_crom[7]  + random.uniform(-delta[1], delta[1])
        crom[8]  = base_crom[8]  + random.uniform(-delta[2], delta[2])
        crom[9]  = base_crom[9]  + random.uniform(-delta[3], delta[3])
        crom[10] = base_crom[10] + random.uniform(-delta[4], delta[4])
        crom[11] = base_crom[11] + random.uniform(-delta[5], delta[5])

class annealed():

    def __init__(self, n, m):

        jh = rospy.get_param("joint_home")

        self.joints = np.zeros((n+1,len(jh)))
        self.manip = np.zeros(n+1)
        self.joints = np.zeros((n+1,len(jh)))
        self.crom = np.zeros((n+1,12))
        self.all_crom = np.zeros((n*m+1,12))


def annealing(ind,pick = True):
    n   = rospy.get_param("temperature_decrease_step"     )
    m   = rospy.get_param("trial_each_step"               )
    p1  = rospy.get_param("initial_acceptance_probability")
    p50 = rospy.get_param("final_acceptance_probability"  )

    t1 = -1.0/math.log(p1)
    t50 = -1.0/math.log(p50)
    frac = (t50/t1)**(1.0/(n-1.0))
    na = 1.0

    history = annealed(n, m)

    history.crom[0] = copy.deepcopy(ind.chromosome)
    if pick:
        history.joints[0] = ind.pick_joints
        current_manip = ind.pick_manipulability
    else:
        history.joints[0] = ind.place_joints
        current_manip = ind.place_manipulability

    history.manip[0] = current_manip

    t = t1
    DeltaE_avg = 0.0

    best_crom = copy.deepcopy(ind.chromosome)
    best_joints = copy.deepcopy(ind.pick_joints)

    ann_ind = copy.deepcopy(ind)

    all_man = []

    precomp = True
    # if pick:
    #     precomp = False

    while not ann_ind.planTrajectory(precomp):
        print ("\n\nqualcosa non va con la soluzione del genetico\n\n")
        random_crom(ann_ind.chromosome, history.crom[0], pick)
        ann_ind.chromosome_to_pose()

    ann_ind_prev = copy.deepcopy(ann_ind)
    best_crom_prev = copy.deepcopy(best_crom)
    current_manip_prev = copy.deepcopy(current_manip)

    best_of_all = copy.deepcopy(best_crom)

    for i in range(n):
        print('Cycle: ' + str(i) + ' with Temperature: ' + str(t))
        for j in range(m):

            print ("stage" + str(i) + ", trial: "+ str(j))

            random_crom(ann_ind.chromosome, history.crom[i], pick)

            ann_ind.chromosome_to_pose()

            while not ann_ind.set_pick_place_fitness():
            # while not ann_ind.planTrajectory():
                random_crom(ann_ind.chromosome, history.crom[i], pick)
                ann_ind.chromosome_to_pose()

            if pick:
                manip = ann_ind.pick_manipulability
            else:
                manip = ann_ind.place_manipulability

            all_man.append(manip)

            DeltaE = abs(manip - current_manip)
            if (manip < current_manip):
                if (i==0 and j==0): DeltaE_avg = DeltaE
                p = math.exp(-DeltaE/(DeltaE_avg * t))
                # p = math.exp(DeltaE/t)
                if (random.random()<p):
                    accept = True
                else:
                    accept = False
            else:
                accept = True
            if (accept==True):
                current_manip = manip
                best_crom = copy.deepcopy(ann_ind.chromosome)

                na = na + 1.0
                DeltaE_avg = (DeltaE_avg * (na-1.0) +  DeltaE) / na

        if not ann_ind.planTrajectory():
            ann_ind = copy.deepcopy(ann_ind_prev)
            best_crom = copy.deepcopy(best_crom_prev)
            current_manip = copy.deepcopy(current_manip_prev)
            individual_ok = False
        else:
            ann_ind_prev = copy.deepcopy(ann_ind)
            best_crom_prev = copy.deepcopy(best_crom)
            current_manip_prev = copy.deepcopy(current_manip)
            if pick:
                best_joints = ann_ind.pick_joints
            else:
                best_joints = ann_ind.place_joints
            individual_ok = True

        history.crom[i+1]= best_crom
        history.manip[i + 1] = current_manip
        history.joints[i + 1] = best_joints

        t = frac * t

    ind.chromosome = copy.deepcopy(best_crom)
    # print solution
    print('Best solution: ' + str(ann_ind.chromosome))
    print('Best objective: ' + str(current_manip))

    print ("\n\n\n\n\n\n\n\n\n\n\n"+str(na)+"\n\n\n\n\n\n\n\n\n\n\n\n\n")

    return history, all_man
