from enum import Flag
import numpy as np 
import math
from scipy.spatial.transform import Rotation as R



clusterization_factor = 2   #max number of null forces to classify a cluster as such


def handle_forces(force_dict, mode, grouping_model, force_threshold,verbosity):
    
    acting_forces = []
    for i in range (len(force_dict)):
        elm = force_dict[i]
        
        # for each node estract the unique resulting force
        node_force = np.array([0.0,0.0,0.0])
        for subelm in elm:
            if (subelm['value']>= force_threshold):
                subelm_comp = (np.array(subelm['components']))
                node_force += subelm_comp
        if (verbosity):
            print("---")
            print("Node",i, ":",node_force)

        acting_forces.append(node_force)
    

    

    
    # First you cluster out all forces acting on the catheter
    # For each cluster:
    # 1. estract the highest force (and its reference node), 
    # 2. get the final force acting in that node as the weighted (depending on the distance) average on the other force acting
    # 3. save the final forces as a tuple (node,force) 
   
    print("------ PROCESSED FORCES------")
    clusters = []
    cnt = 0
    zeros = 0
    cluster_nodes = []
    for i in range (len(acting_forces)):
        force = acting_forces[i]
        if (np.linalg.norm(force) != 0.0):
            zeros=0
            if (cnt == 0):
                clusters.append([force])
                cluster_nodes.append([i])
            else:
                clusters[len(clusters)-1].append(force)
                cluster_nodes[len(clusters)-1].append(i)
            cnt +=1
        else:
            if(zeros==clusterization_factor):
                cnt = 0
            else:
                zeros+=1
    print('Have been found', len(clusters), 'clusters of forces in the nodes:')

    weights=[]
    approx_forces = []
    for i in range (len(clusters)):
        best_force = np.array([0.0,0.0,0.0])
        best_force_node = 0
        
        if(grouping_model == False): # the best force would be the highest in abs value
            for j in range (len(clusters[i])):
                magnitude_j = np.linalg.norm(clusters[i][j])
                if (magnitude_j > np.linalg.norm(best_force)):
                    best_force = clusters[i][j]
                    best_force_node = cluster_nodes[i][j]

            weights = gaussian_weights(cluster_nodes[i],best_force_node,1.0)
            final_force = average(clusters[i],weights)
            approx_forces.append([best_force_node, final_force])

        else:       # the best force would be the one in the mass centre
            app_node = 0
            for j in range (len(clusters[i])): app_node += cluster_nodes[i][j]
            app_node = int(round(app_node/len(cluster_nodes[i])))
            weights = gaussian_weights(cluster_nodes[i],app_node,1.0)
            final_force = average(clusters[i],weights)
            approx_forces.append([app_node, final_force])

    print(approx_forces)
    print("----------------------------------")

    if (mode == 0): # Returns all the (clustered) forces 
        print('Mode 0 activated: all the interaction foreces are mapped.')
        return approx_forces

    elif (mode == 1):  # Returns the forces acting on the last node
        return_array = []
        node_array = []
        node_array.append(99)
        node_array.append(acting_forces[99])
        return_array.append(node_array)
        
        return return_array

    elif (mode == 2):  # Returns the highest (clustered) force acting
        max_force = np.array([0.0,0.0,0.0])
        res = []
        for tuple in approx_forces:
            if np.linalg.norm(tuple[1]) >= np.linalg.norm(max_force):
                max_force = np.linalg.norm(tuple[1]) 
                res.append(tuple)
        
        return res

    elif (mode == 3):  # Returns the highest (clustered) force acting
        res = [[0,np.array([0.0,0.0,0.0])]]
        for tuple in approx_forces: 
            res[0][1] = res[0][1] + np.array(tuple[1])
        
        return res

    else:
        return('NO compatible mode has been chosen')



def gaussian_weights(list, mu, sig):
    weights = []
    for i in range(len(list)):
        x = list[i]
        weights.append(np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.))))
    return weights

def average(x,y):
    force = np.array([0.0,0.0,0.0])
    for i in range(len(x)):
        force += x[i]*y[i]
    return force/len(x)



def get_angles(vec1, vec2):
    #Mrot = R.align_vectors([np.array(vec1)],[np.array(vec2)]) 
    #theta = np.arctan2(np.sqrt(Mrot[1][2]^2 + Mrot[1][3]^2), Mrot[1][1]) + np.arctan2(-np.sqrt(Mrot[1][2]^2 + Mrot[1][3]^2), 0)
    #psi = np.arctan2(Mrot[1][3]/np.sin(theta), -Mrot[1][2]/np.sin(theta))
    #phi = np.arctan2(Mrot[3][1]/np.sin(theta), Mrot[2][1]/np.sin(theta))
    #angles = Mrot[0].as_euler('xyz', degrees=True)
    #return angles

    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    r11, r12, r13 = rotation_matrix[0]
    r21, r22, r23 = rotation_matrix[1]
    r31, r32, r33 = rotation_matrix[2]

    theta1 = np.arctan(-r23 / r33)
    theta2 = np.arctan(r13 * np.cos(theta1) / r33)
    theta3 = np.arctan(-r12 / r11)
    return np.round(np.array([theta1,theta2,theta3]),5)