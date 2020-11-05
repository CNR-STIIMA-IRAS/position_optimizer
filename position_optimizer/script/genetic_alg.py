#!/usr/bin/env python
import rospy
import rospkg
from generation import Generation
import random
import matplotlib.pyplot as plt
import datetime
import time
import numpy as np
import os
import copy
from annealing import annealing

def get_or_error(string):
    if rospy.has_param(string):
        return rospy.get_param(string)
    else:
        rospy.logerr("%s/%s not found ! exit", rospy.get_namespace(), string)


def mean(list):
    ret = np.array(list)
    return ret.mean()


if __name__ == '__main__':

    rospy.init_node('manipulation_optimizer')

    n_ind           = rospy.get_param("individuals_number"    )
    n_trial         = rospy.get_param("max_generations_number")
    mutation_rate   = rospy.get_param("mutation_rate"         )
    last_elements   = rospy.get_param("last_elements"         )
    pc              = rospy.get_param("probability_rank"      )
    weight          = rospy.get_param("weight"                )
    final_annealing = rospy.get_param("final_annealing"       )
    write_data      = rospy.get_param("save_data_to_txt"      )

    rospack = rospkg.RosPack()
    pat = rospack.get_path('position_optimizer')

    ts = time.time()
    st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')

    if write_data:
        os.mkdir(pat + "/data/"+st)

        file1 = open(pat + "/data/"+st+"/base_info.txt","w+")
        file1.write("n_ind: " + str(n_ind)+", n_trial: "+str(n_trial)+", mutation rate: "+str(mutation_rate)
                    + ", last element: "+str(last_elements)+", Pc: "+str(pc)+", weight: "+str(weight))
        file1.close()

    generation = Generation(n_ind)
    generation.set_pc(pc)

    for ind in generation.population:
        ind.set_weight(weight)

    old_generation = copy.deepcopy(generation)

    best_of_all = []
    fitness_of_all_time = []
    iter_time = []
    mean_of_all_gen = []
    generation_unsuccess = []

    use_precomputed_path = False
    end = False
    n_gen=0
    jj = 0


    while jj < n_trial:
    # while True: #TODO: solo per prove di tempo
        unsuccess = 0
        time_iter_start = time.time()
        n_gen += 1
        jj = jj+1

        if write_data:
            file2 = open(pat + "/data/" + st + "/gen_"+str(n_gen) + ".txt", "w+")
            file2.write("Generation,Fitness,Rank,Pick_x,Pick_y,Pick_theta,Place_x,Place_y,Place_theta,Pick_manip,Place_manip\n")

        rospy.loginfo("\n\n\n\n\n\n---------------------- new generation: "+str(n_gen)+" -----------------------------\n\n\n\n\n")
        rospy.loginfo("\n\n\n\n\n\n---------------------- new population size: "+str(len(generation.population))+" -----------------------------\n\n\n\n\n")

        for ind in generation.population:
            ind.set_gen_n(n_gen)

            if not ind.set_pick_place_fitness():
                unsuccess = unsuccess+1

        if not generation.probability():
            rospy.logfatal("nessuna posa buona, siam nella merda")

        generation.distribution()

        iter = 0
        maxiter = 2
        # maxiter = rospy.get_param("max_planning_trial")
        planned = False
        while iter <= maxiter:
            if not generation.population[-1].planTrajectory(use_precomputed_path):
                iter = iter+1
                use_precomputed_path = False
                rospy.loginfo("iter number: "+str(iter))
            else:
                planned = True
                break

        rospy.loginfo("total iter number: "+str(iter))

        if not planned:
            rospy.loginfo("trying planning with old best solution")
            if old_generation.population[-1].planTrajectory():
                planned = True
                generation.population[-1] = old_generation.population[-1]
            else:
                planned = False

        if not planned:
            rospy.logerr("solution not found, regenerate individuals")
            if jj==1:
                generation = Generation(n_ind)
                generation.set_pc(pc)

                for ind in generation.population:
                    ind.set_weight(weight)
                jj = 0
            else:
                generation = copy.deepcopy(old_generation)
                generation.selection()

                for i in range(0, int(n_ind * mutation_rate)):
                    ind = random.randint(0, len(generation.children) - 1)
                    rospy.loginfo("individuo " + str(ind) + " mutante")
                    generation.mutation(ind)

            if write_data:
                file2.write( str(0)+", "+ str(0) + ", " + str(0) + ", " + str(0) + ", " + str(0) + ", "
                            +str(0)+", "+ str(0) + ", " + str(0)+ ", " + str(0) + ", " + str(0)+ ", " + str(0) + "\n")
                file2.close()
            continue
        else:
            rospy.loginfo("well done")
            use_precomputed_path = True

        generation.rank()

        fitness_of_this_gen = []

        for ind in generation.population:
            print ("id: " + str(ind.id) + ", fitness: " + str(ind.fitness) + ", probability: " + str(ind.prob) \
                  + ", rank: " + str(ind.rank_probability) + ", manip: [ " + str(ind.pick_manipulability) + ", " +str(ind.place_manipulability) + "]")

            if write_data:
                file2.write( str(ind.id)+", "+ str(ind.fitness) + ", " + str(ind.rank_probability)
                             + ", " + str(ind.chromosome[0])
                             + ", " + str(ind.chromosome[1])
                             + ", " + str(ind.chromosome[2])
                             + ", " + str(ind.chromosome[3])
                             + ", " + str(ind.chromosome[4])
                             + ", " + str(ind.chromosome[5])
                             + ", " + str(ind.pick_manipulability)+ ", " + str(ind.place_manipulability) + "\n")

            fitness_of_all_time.append(ind.fitness)
            fitness_of_this_gen.append(ind.fitness)

        mean_of_all_gen.append(mean(fitness_of_this_gen))

        best_of_all.append(copy.deepcopy(generation.population[-1]))

        if jj == n_trial:
            break

        if len(best_of_all) > last_elements:
            count = 0
            for i in range(1, last_elements):
                if best_of_all[-i].chromosome == best_of_all[-i-1].chromosome:
                    count += 1
            if count == last_elements - 1:
                end = True
        if end == True:
            rospy.loginfo("last " + str(last_elements) + " are equal. break")
            break
        generation.selection()

        for i in range(0, int(n_ind*mutation_rate)):
            ind = random.randint(0, len(generation.children) - 1)
            rospy.loginfo("individuo "+str(ind)+" mutante")
            generation.mutation(ind)


        best_of_all[-1].set_pick_place_fitness()
        old_generation = copy.deepcopy(generation)

        iter_time.append(time.time()-time_iter_start)
        generation_unsuccess.append(unsuccess)


    old_best = copy.deepcopy(best_of_all[-1])

    anneal_time = time.time()

    if final_annealing:
        h_pick, man_pi = annealing(best_of_all[-1],True)
        h_place, man_pl = annealing(best_of_all[-1],False)

        best_of_all[-1].chromosome_to_pose()

        best_of_all[-1].seed_pick_joints  = h_pick.joints[-1]
        best_of_all[-1].seed_place_joints = h_place.joints[-1]

        best_of_all[-1].set_pick_place_fitness()

        print "fitness: " + str(old_best.fitness) + " crom: " + str(old_best.chromosome) + " pick: "+ str(
            old_best.pick_manipulability) + " place: " + str(old_best.place_manipulability)

        print "fitness: " + str(best_of_all[-1].fitness) + " crom: " + str(best_of_all[-1].chromosome) + " pick: " + str(
            best_of_all[-1].pick_manipulability) + " place: " + str(best_of_all[-1].place_manipulability)

        print str(h_pick.manip)
        print str(h_pick.crom)
        print str(h_pick.joints)
        print str(h_place.manip)
        print str(h_place.crom)
        print str(h_place.joints)


        if not best_of_all[-1].planTrajectory():
            rospy.logfatal("something is wrong with the final position")

        print "fitness: " + str(best_of_all[-1].fitness) + " crom: " + str(best_of_all[-1].chromosome) + " pick: " + str(
            best_of_all[-1].pick_manipulability) + " place: " + str(best_of_all[-1].place_manipulability)

        if write_data:
            file3 = open(pat + "/data/"+st+"/pick_annealing.txt","w+")

            file3.write("Pick_x,Pick_y,Pick_theta,Place_x,Place_y,Place_theta,manip\n")
            for i in range(1,len(h_pick.manip)):
                file3.write(                  str(h_pick.crom[i][0])
                                     + ", " + str(h_pick.crom[i][1])
                                     + ", " + str(h_pick.crom[i][2])
                                     + ", " + str(h_pick.crom[i][3])
                                     + ", " + str(h_pick.crom[i][4])
                                     + ", " + str(h_pick.crom[i][5])
                                     + ", " + str(h_pick.manip[i]) + "\n")
            file3.close()

            file4 = open(pat + "/data/"+st+"/place_annealing.txt","w+")
            file4.write("Pick_x,Pick_y,Pick_theta,Place_x,Place_y,Place_theta,manip\n")
            for i in range(1,len(h_place.manip)):
                file4.write(                  str(h_place.crom[i][0])
                                     + ", " + str(h_place.crom[i][1])
                                     + ", " + str(h_place.crom[i][2])
                                     + ", " + str(h_place.crom[i][3])
                                     + ", " + str(h_place.crom[i][4])
                                     + ", " + str(h_place.crom[i][5])
                                     + ", " + str(h_place.manip[i]) + "\n")
            file4.close()



    end_time = time.time()

    if write_data:
        file5 = open(pat + "/data/"+st+"/time.txt","w+")
        file5.write("total_time: " + str(end_time-ts)+", GA_time: "+str(anneal_time-ts)+", SA_time: "+str(end_time-anneal_time))

        for t in iter_time:
            file5.write("\niter time: " + str(t))

        for u in generation_unsuccess:
            file5.write("\nunsuccesfull individuals: " + str(u))

        file5.close()

    rospy.loginfo("best pick pose: \n "    + str(best_of_all[-1].pick_pose          ))
    rospy.loginfo("\nbest place pose: \n " + str(best_of_all[-1].place_pose         ))
    rospy.loginfo("\nseed pick pose: \n "  + str(best_of_all[-1].seed_pick_joints   ))
    rospy.loginfo("\nseed place pose: \n " + str(best_of_all[-1].seed_place_joints  ))

    file10 = open(pat + "/data/optimized_poses.txt", "w+")
    file10.write("best pick pose: \n "   + str(best_of_all[-1].pick_pose)
                  + "\n\nbest place pose: \n " + str(best_of_all[-1].place_pose)
                  + "\n\nseed pick pose: \n "  + str(best_of_all[-1].seed_pick_joints)
                  + "\n\nseed place pose: \n " + str(best_of_all[-1].seed_place_joints))
    file10.close()
    #
    # n=0
    # plt.figure()
    # for i in range(0,len(mean_of_all_gen)):
    #     n+=1
    #     print "gen "+str(n)+" max fit: "+str(best_of_all[i].fitness) + " mean: " + str(mean_of_all_gen[i])+ " crom: " + str(best_of_all[i].chromosome)
    #     plt.plot(n, mean_of_all_gen[i], "ro")
    #     plt.plot(n, best_of_all[i].fitness, "bo")
    #     plt.xlabel("x generation")
    #     plt.ylabel("y mean, fitness")
    #     plt.legend()
    # plt.grid()
    # plt.show()
    #
    #
    # plt.figure()
    # n=0
    # for i in range(0,len(h_pick.manip)):
    #     n+=1
    #     plt.plot(n, h_pick.manip[i], "ro")
    #     plt.plot(n, h_place.manip[i], "bo")
    #     plt.xlabel("x generation")
    #     plt.ylabel("y maip pick")
    #     plt.legend()
    # plt.grid()
    # plt.show()
