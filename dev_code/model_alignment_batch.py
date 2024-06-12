"""
Version: 1.5

Summary: alignment 3d model to Z axis and translate it to its center.

Author: suxing liu

Author-email: suxingliu@gmail.com

USAGE

    python3 model_alignment_batch.py -p ~/example/ -ft ply -o ~/example/result/


argument:
("-p", "--path", dest = "path", type = str, required = True, help = "path to *.ply model file")
("-m", "--model", dest = "model", type = str, required = False, help = "model file name")
("-o", "--output_path", dest = "output_path", type = str, required = False, help = "result path")


output:
*.xyz: xyz format file only has 3D coordinates of points 
*_aligned.ply: aligned model with only 3D coordinates of points 

"""
#!/usr/bin/env python



# import the necessary packages
import numpy as np 
import argparse

import os
import glob
import sys
import open3d as o3d
import copy

from scipy.spatial.transform import Rotation as Rot
import math
import pathlib

from matplotlib import pyplot as plt



# Find the rotation matrix that aligns vec1 to vec2
def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    if any(v): #if not all zeros then 
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))

    else:
        return np.eye(3) #cross of all zeros only occurs on identical directions




#colormap mapping
def get_cmap(n, name = 'tab10'):
    """get the color mapping""" 
    #viridis, BrBG, hsv, copper, Spectral
    #Returns a function that maps each index in 0, 1, ..., n-1 to a distinct 
    #RGB color; the keyword argument name must be a standard mpl colormap name
    return plt.cm.get_cmap(name,n)




# sort index according to the value in decenting order
def sort_index(lst, rev=True):
    index = range(len(lst))
    s = sorted(index, reverse=rev, key=lambda i: lst[i])
    return s
    

# slice array based on Z values
def get_pt_sel(Data_array_pt):
    
    ####################################################################
    
    # load points cloud Z values and sort it
    Z_pt_sorted = np.sort(Data_array_pt[:,2])
    
    #slicing_factor = 
    
    idx_sel = int(len(Z_pt_sorted)*slicing_ratio) 
    
    Z_mid = Z_pt_sorted[idx_sel]

    # mask
    Z_mask = (Data_array_pt[:,2] <= Z_mid) & (Data_array_pt[:,2] >= Z_pt_sorted[0]) 
    
    Z_pt_sel = Data_array_pt[Z_mask]
    
    
    return Z_pt_sel

# compute dimensions of point cloud 
def get_pt_parameter(Data_array_pt):
    
    
    ####################################################################
    pcd = o3d.geometry.PointCloud()
    
    pcd.points = o3d.utility.Vector3dVector(Data_array_pt)
    
    # get AxisAlignedBoundingBox
    aabb = pcd.get_axis_aligned_bounding_box()
    #aabb.color = (0, 1, 0)
    
    #Get the extent/length of the bounding box in x, y, and z dimension.
    aabb_extent = aabb.get_extent()
    
    aabb_extent_half = aabb.get_half_extent()
    
    # get the dimention of the points cloud in diameter based on bounding box
    pt_diameter_max = max(aabb_extent[0], aabb_extent[1])

    pt_diameter_min = min(aabb_extent_half[0], aabb_extent_half[1])

    
    pt_diameter = (pt_diameter_max + pt_diameter_min)*0.5
    
        
    return pt_diameter_max, pt_diameter_min, pt_diameter
    
    

    
# align  ply model with z axis
def model_alignment(model_file):
    
    
    # Load a ply point cloud
    pcd = o3d.io.read_point_cloud(model_file)
    
    #print(np.asarray(pcd.points))
    #o3d.visualization.draw_geometries([pcd_sel])


    # copy original point cloud for rotation
    pcd_r = copy.deepcopy(pcd)
    
    # get the model center postion
    model_center = pcd_r.get_center()
    
    # geometry points are translated directly to the model_center position
    pcd_r.translate(-1*(model_center))
    
    '''
    # get convex hull of a point cloud is the smallest convex set that contains all points.
    hull, _ = pcd_r.compute_convex_hull()
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color((1, 0, 0))
    
    # get AxisAlignedBoundingBox
    aabb = pcd_r.get_axis_aligned_bounding_box()
    
    # assign color for AxisAlignedBoundingBox
    aabb.color = (0, 1, 0)
    
    #Get the extent/length of the bounding box in x, y, and z dimension.
    aabb_extent = aabb.get_extent()
    aabb_extent_half = aabb.get_half_extent()
    '''
    
    # get OrientedBoundingBox
    obb = pcd_r.get_oriented_bounding_box()
    
    # assign color for OrientedBoundingBox
    obb.color = (0, 0, 1)
    
    # get the eight points that define the bounding box.
    pcd_coord = obb.get_box_points()
    
    #print(obb.get_box_points())
    
    #pcd_coord.color = (1, 0, 0)
    
    # From Open3D to numpy array
    np_points = np.asarray(pcd_coord)
    
    # create Open3D format for points 
    pcd_coord = o3d.geometry.PointCloud()
    pcd_coord.points = o3d.utility.Vector3dVector(np_points)
    
    # assign different colors for eight points in the bounding box.
    colors = []
    cmap = get_cmap(8)
    
    for idx in range(8):
    
        color_rgb = cmap(idx)[:len(cmap(idx))-1]
        colors.append(color_rgb)

    pcd_coord.colors = o3d.utility.Vector3dVector(colors)
    


    # check the legnth of the joint 3 vector in the bounding box to estimate the orientation of model 
    list_dis = [np.linalg.norm(np_points[0] - np_points[1]), np.linalg.norm(np_points[0] - np_points[2]), np.linalg.norm(np_points[0] - np_points[3])]
    
    # sort the length values and return the index
    idx_sorted = sort_index(list_dis)
    
    #print(list_dis)
    
    #print(idx_sorted)
    

    # estimate the orientation 
    if idx_sorted[0] == 0:
        
        center_0 = np.mean(np_points[[0,2,3,5]], axis=0)
        center_1 = np.mean(np_points[[1,4,6,7]], axis=0)
        
    elif idx_sorted[0] == 1:
        
        center_0 = np.mean(np_points[[0,1,3,6]], axis=0)
        center_1 = np.mean(np_points[[2,4,5,7]], axis=0)
    
    else:
        
        center_0 = np.mean(np_points[[0,1,2,7]], axis=0)
        center_1 = np.mean(np_points[[3,4,5,6]], axis=0)
    

    
    # define unit vector
    v_x = [1,0,0]
    v_y = [0,1,0]
    v_z = [0,0,1]
    
    
    # define model orientation vector
    m_center_vector = [(center_0[0] - center_1[0]), (center_0[1] -center_1[1]), (center_0[2] - center_1[2])]
    
    
    #compute the rotation matrix that aligns unit vector Z to orientation vector
    R_matrix = rotation_matrix_from_vectors(m_center_vector, v_z)
    
    # rotate the model using rotation matrix to align with unit vector Z 
    pcd_r.rotate(R_matrix, center = (0,0,0))
    
    
        
    # check the botttom and top direction 
    pts_bottom = get_pt_sel(np.asarray(pcd_r.points))
    
    
    '''
    ############################################################
    pcd_Z_mask = o3d.geometry.PointCloud()
    
    pcd_Z_mask.points = o3d.utility.Vector3dVector(pts_bottom)
    
    Z_mask_ply = result_path + "Z_mask.ply"
    
    o3d.visualization.draw_geometries([pcd_Z_mask])
    
    o3d.io.write_point_cloud(Z_mask_ply, pcd_Z_mask)
    ############################################################
    '''
    
    
    (ptb_diameter_max, ptb_diameter_min, ptb_diameter) = get_pt_parameter(pts_bottom)
    
    (pt_diameter_max, pt_diameter_min, pt_diameter) = get_pt_parameter(np.asarray(pcd_r.points))
    
    print(ptb_diameter_max, pt_diameter_max)
    
    # if model bottom and top need to be fliped  
    if ptb_diameter < pt_diameter*0.6:
        
        print("Model was aligned correctly with Z axis\n")
        
    else:
        
        print("Flip model along Z axis\n")
        
        v_z_reverse = [0,0,-1]
    
        #compute the rotation matrix that aligns unit vector Z to orientation vector
        #R_matrix_flip = rotation_matrix_from_vectors(v_z_reverse, v_z)
        
        R_matrix_flip = pcd_r.get_rotation_matrix_from_xyz((np.pi, 0, 0))
        
        # rotate the model using rotation matrix to align with unit vector Z 
        pcd_r.rotate(R_matrix_flip, center = (0,0,0))
        
    
    
    # return aligned model file
    return pcd_r
    

    

if __name__ == '__main__':
    
    
    # construct the argument and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", dest = "path", required = True, type = str, help = "path to 3D model file")
    ap.add_argument("-ft", "--filetype", dest = "filetype", type = str, required = False, default = 'ply', help = "3D model file filetype, default *.ply")
    ap.add_argument("-o", "--output_path", dest = "output_path", type = str, required = False, help = "result path")
    ap.add_argument("-sr", "--slicing_ratio", dest = "slicing_ratio", type = float, required = False, default = 0.10, help = "ratio of slicing the model from the bottom")

    args = vars(ap.parse_args())


    # setting path to model file 
    
    
    # path to model file 
    file_path = args["path"]
    
    ext = args['filetype']
    
    files = file_path + '*.' + ext
    
    # output path
    result_path = args["output_path"] if args["output_path"] is not None else file_path
    
    result_path = os.path.join(result_path, '')
    
    # result path
    print("results_folder: {}\n".format(result_path))
    
    
    slicing_ratio = args["slicing_ratio"]
    
    # obtain image file list
    fileList = sorted(glob.glob(files))
    
    #print(fileList)

    
    for input_file in fileList:

        # output input file info
        if os.path.isfile(input_file):

            # parse file info
            p = pathlib.Path(input_file)
           
            base_name = p.stem
            
            print("Processing 3D point cloud model {}...\n".format(p.name))
            
            
            # model alignment 
            pcd_r = model_alignment(input_file)
            
            #Save model file as ascii format in ply
            output_filename = result_path + base_name + '_aligned.ply'
            
            #write out point cloud file
            o3d.io.write_point_cloud(output_filename, pcd_r, write_ascii = True)
            
            #Save modelfilea as ascii format in xyz
            #filename = result_path + base_name + '.xyz'
            #o3d.io.write_point_cloud(filename, pcd_r, write_ascii = True)
            
            
            # check saved file
            if os.path.exists(output_filename):
                print("Aligned 3d model was saved at {0}\n".format(output_filename))
            
            
        else:
            print("File not exist")
            sys.exit()
        
    

    
    

        
        

