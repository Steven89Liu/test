import pybullet as p
import math
import pybullet_data

def single_test(urdf_file, isDIRECT=True):
    p.connect(p.DIRECT if isDIRECT else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    print(pybullet_data.getDataPath())
    p.loadURDF(urdf_file, [3, 3, 1])

    numRays = 1024; rayLen = 13

    info_lst = []; rayFrom_lst = []; rayTo_lst = []

    for i in range(numRays):
        rayFrom = [0, 0, 1]
        rayTo = [rayLen * math.sin(2. * math.pi * float(i) / numRays),
                rayLen * math.cos(2. * math.pi * float(i) / numRays), 1]

        info = p.rayTest(rayFrom, rayTo)
        info_lst.append(info[0])

        #p.addUserDebugLine(rayFrom, rayTo, [1, 0, 0])
        rayFrom_lst.append(rayFrom)
        rayTo_lst.append(rayTo)

    info_lst_batch = p.rayTestBatch(rayFrom_lst, rayTo_lst)
    #print("info_list=", info_lst)
    #print("-----0-----------------------------------------------------")
    #print("info_batch=", info_lst_batch)
    collide_predicate = lambda info:info[0]!=-1
    #print("----------------------------------------------------------")
    #print("collide=", collide_predicate)
    info_filtered = list(filter(collide_predicate, info_lst))
    info_batch_filtered = list(filter(collide_predicate, info_lst_batch))
    #print("-----1-----------------------------------------------------")
    #print("info_filter=", info_filtered)
    #print("-----2-----------------------------------------------------")
    
    #print("info_batch_filter=", info_batch_filtered)

    p.disconnect()

    return info_filtered, info_batch_filtered

def isConsistent(model):
    info_filtered, info_batch_filtered = single_test(model)
    # number of collide ray must be equal 
    return (len(info_filtered) == len(info_batch_filtered)) 

models = ["sphere_small.urdf", "sphere2.urdf", "block.urdf",  # composed of single object
        "r2d2.urdf" #, "humanoid/humanoid.urdf" # composed of multiple objects 
        ]
'''
isConsistent("sphere_small.urdf")
isConsistent("sphere2.urdf")
isConsistent("block.urdf")
isConsistent("r2d2.urdf")
isConsistent("humanoid/humanoid.urdf")
'''
lst_consistensy = [isConsistent(model) for model in models]
print(lst_consistensy)