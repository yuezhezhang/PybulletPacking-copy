# -*- coding: utf-8 -*-
"""
Created on Mon Jan 17 01:10:07 2022

@author: Chiba
"""

import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
import time
import pybullet_data
import os
# from pybullet_object_models import ycb_objects

def load_items(numbers):
    flags = p.URDF_USE_INERTIA_FROM_FILE
    model_list = []
    item_ids = []
    for root,dirs,files in os.walk('./pybullet-URDF-models/urdf_models/models'):
        for file in files:
            if file == "model.urdf":
                model_list.append(os.path.join(root,file))
    for count in range(len(numbers)):
        item_id = p.loadURDF(model_list[numbers[count]-1], 
                             [(count//8)/3+1.2, (count%8)/3+0.2, 0.05], flags=flags)
        item_ids.append(item_id)
    return item_ids

def grid_scan(xminmax, yminmax, z_start, z_end, sep):
    '''
    p.rayTest(ray_from, ray_to)
    specify:
    the start and end coordinates of the ray and the function 
    return:
    the intersection information (object id, hit position, etc) for the first object that the ray passes through.
    rayTestBatch()
    provides the same functionality but instead with an array of rays
    see https://medium.com/@chand.shelvin/sensors-in-pybullet-ac9b3c01460f
    '''
    xpos = np.arange(xminmax[0]+sep/2,xminmax[1]+sep/2,sep)
    ypos = np.arange(yminmax[0]+sep/2,yminmax[1]+sep/2,sep)
    xscan, yscan = np.meshgrid(xpos, ypos)
    ScanArray = np.array([xscan.reshape(-1), yscan.reshape(-1)])
    Start = np.insert(ScanArray, 2, z_start,0).T
    # print("start shape", Start.shape) # [961, 3]
    # print("start", Start[10:13, :])             # [[x, y, z], ]
    End = np.insert(ScanArray, 2, z_end, 0).T
    # print("End", End[10:13, :])
    RayScan = np.array(p.rayTestBatch(Start, End), dtype="object")
    # print("ray scan shape", RayScan.shape) # [961, 3]
    # print("ray scan", RayScan[10:13, :])             # [[item_id, -1, scaler, (0, 0, 0), (0, 0, 0)]]
    # print("ray scan 2", RayScan[10:13, 2]) # scaler
    # time.sleep(7.)
    Height = RayScan[:,2].astype('float64')*(z_end-z_start)+z_start
    HeightMap = Height.reshape(ypos.shape[0],xpos.shape[0]).T
    print("heightmap shape", HeightMap.shape) # [31, 31]
    # print("heightmap", HeightMap)
    return HeightMap

## real world, get volume from point cloud??
def item_volume(item):
    # cm^3 
    scan_sep = 0.005
    old_pos, old_quater = p.getBasePositionAndOrientation(item)
    volume = np.inf # a big number
    for row in np.arange(0, np.pi/4, np.pi/4):
        for pitch in np.arange(0, np.pi/4, np.pi/4):
            quater = p.getQuaternionFromEuler([row, pitch, 0])
            p.resetBasePositionAndOrientation(item,[1,1,1],quater)
            AABB = p.getAABB(item)
            # print("AABB", AABB)
            # time.sleep(3.)
            TopDown = grid_scan([AABB[0][0], AABB[1][0]], [AABB[0][1],AABB[1][1]],
                                AABB[1][2], AABB[0][2], scan_sep)
            # print("shape", TopDown.shape)
            # print("TopDown", TopDown)
            DownTop = grid_scan([AABB[0][0], AABB[1][0]], [AABB[0][1],AABB[1][1]],
                                AABB[0][2], AABB[1][2], scan_sep)
            # print("DownTop", DownTop)
            HeightDiff = TopDown-DownTop
            HeightDiff[HeightDiff<0] = 0 # empty part no object in this array
            temp_v = np.sum(HeightDiff)*(scan_sep/0.01)**2
            volume = min(volume, temp_v)
    p.resetBasePositionAndOrientation(item,old_pos,old_quater)
    print("item" ,item)
    print("volume", volume)
    return volume

def Compactness(item_in_box, item_volumes, box_hm):
    total_volume = 0
    for item in item_in_box:
        total_volume += item_volumes[item]
    box_volume = np.max(box_hm)*box_hm.size
    return total_volume/box_volume

def Pyramidality(item_in_box, item_volumes, box_hm):
    total_volume = 0
    for item in item_in_box:
        total_volume += item_volumes[item]
    used_volume = np.sum(box_hm)
    return total_volume/used_volume

'''
getAabb returns the axis aligned bounding box
'''
def drawAABB(aabb,width=1):
  aabbMin = aabb[0]
  aabbMax = aabb[1]
  f = [aabbMin[0], aabbMin[1], aabbMin[2]]
  t = [aabbMax[0], aabbMin[1], aabbMin[2]]
  p.addUserDebugLine(f, t, [1, 0, 0], width)
  f = [aabbMin[0], aabbMin[1], aabbMin[2]]
  t = [aabbMin[0], aabbMax[1], aabbMin[2]]
  p.addUserDebugLine(f, t, [0, 1, 0], width)
  f = [aabbMin[0], aabbMin[1], aabbMin[2]]
  t = [aabbMin[0], aabbMin[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [0, 0, 1], width)

  f = [aabbMin[0], aabbMin[1], aabbMax[2]]
  t = [aabbMin[0], aabbMax[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1, 1, 1], width)

  f = [aabbMin[0], aabbMin[1], aabbMax[2]]
  t = [aabbMax[0], aabbMin[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1, 1, 1], width)

  f = [aabbMax[0], aabbMin[1], aabbMin[2]]
  t = [aabbMax[0], aabbMin[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1, 1, 1], width)

  f = [aabbMax[0], aabbMin[1], aabbMin[2]]
  t = [aabbMax[0], aabbMax[1], aabbMin[2]]
  p.addUserDebugLine(f, t, [1, 1, 1], width)

  f = [aabbMax[0], aabbMax[1], aabbMin[2]]
  t = [aabbMin[0], aabbMax[1], aabbMin[2]]
  p.addUserDebugLine(f, t, [1, 1, 1], width)

  f = [aabbMin[0], aabbMax[1], aabbMin[2]]
  t = [aabbMin[0], aabbMax[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1, 1, 1], width)

  f = [aabbMax[0], aabbMax[1], aabbMax[2]]
  t = [aabbMin[0], aabbMax[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1, 1, 1], width)
  f = [aabbMax[0], aabbMax[1], aabbMax[2]]
  t = [aabbMax[0], aabbMin[1], aabbMax[2]]
  p.addUserDebugLine(f, t, [1, 1, 1], width)
  f = [aabbMax[0], aabbMax[1], aabbMax[2]]
  t = [aabbMax[0], aabbMax[1], aabbMin[2]]
  p.addUserDebugLine(f, t, [1, 1, 1], width)


if __name__ == '__main__':
    if p.getConnectionInfo()['isConnected']:
        p.disconnect()
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")
    item_numbers = np.arange(18,21)
    item_ids = load_items(item_numbers)
    v = []
    for item in item_ids:
        AABB = p.getAABB(item)
        drawAABB(AABB)
        volume = item_volume(item)
        v.append(volume)
        
        
    for i in range(3000):
        p.stepSimulation()
        time.sleep(1./240.)