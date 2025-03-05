"""
Version: 1.5

Summary: load the segmentation of point cloud model

Author: suxing liu

Author-email: suxingliu@gmail.com

USAGE

    python3 las_seg.py -p ~/example/ -m test.las 


argument:
("-p", "--path", required=True,    help="path to *.ply model file")
("-m", "--model", required=True,    help="file name")

"""
#!/usr/bin/env python


# import the necessary packages
from plyfile import PlyData, PlyElement
import numpy as np 
from sklearn import preprocessing
from sklearn.preprocessing import MinMaxScaler
from operator import itemgetter
import argparse

from scipy.spatial import KDTree

import os
import sys
import open3d as o3d
import copy



#import networkx as nx
'''
import graph_tool.all as gt

import plotly.graph_objects as go

from matplotlib import pyplot as plt

from math import sqrt
'''


#from laspy.file import File

import laspy
import pathlib

from matplotlib import pyplot as plt

# get file information from the file path using python3
def get_file_info(file_full_path):
    
    p = pathlib.Path(file_full_path)

    filename = p.name

    basename = p.stem

    file_path = p.parent.absolute()

    file_path = os.path.join(file_path, '')

    return file_path, filename, basename


# calculate length of a 3D path or curve
def path_length(X, Y, Z):

    n = len(X)
     
    lv = [sqrt((X[i]-X[i-1])**2 + (Y[i]-Y[i-1])**2 + (Z[i]-Z[i-1])**2) for i in range (1,n)]
    
    L = sum(lv)
    
    return L

'''
# distance between two points
def distance_pt(p0, p1):
    
    dist = np.linalg.norm(p0 - p1)
    
    return dist
'''
#find the closest points from a points sets to a fix point using Kdtree, O(log n) 
def closest_point(point_set, anchor_point):
    
    kdtree = KDTree(point_set)
    
    (d, i) = kdtree.query(anchor_point)
    
    #print("closest point:", point_set[i])
    
    return  i, point_set[i]

#colormap mapping
def get_cmap(n, name = 'Spectral'):
    """get the color mapping""" 
    #viridis, BrBG, hsv, copper
    #Returns a function that maps each index in 0, 1, ..., n-1 to a distinct 
    #RGB color; the keyword argument name must be a standard mpl colormap name
    return plt.cm.get_cmap(name,n+1)

def visualize_skeleton(current_path, filename_skeleton, filename_ptcloud):
    
    model_skeleton = current_path + filename_skeleton
    print("Loading 3D skeleton file {}...\n".format(filename_skeleton))
    model_skeleton_name_base = os.path.splitext(model_skeleton)[0]
    
    #load the ply format skeleton file 
    try:
        with open(model_skeleton, 'rb') as f:
            plydata_skeleton = PlyData.read(f)
            num_vertex_skeleton = plydata_skeleton.elements[0].count
            N_edges_skeleton = len(plydata_skeleton['edge'].data['vertex_indices'])
            array_edges_skeleton = plydata_skeleton['edge'].data['vertex_indices']
            
            print("Ply data structure: \n")
            #print(plydata_skeleton)
            #print("\n")
            print("Number of 3D points in skeleton model: {0} \n".format(num_vertex_skeleton))
            print("Number of edges: {0} \n".format(N_edges_skeleton))

        
    except:
        print("Model skeleton file does not exist!")
        sys.exit(0)
    
    
    #Parse ply format skeleton file and Extract the data
    Data_array_skeleton = np.zeros((num_vertex_skeleton, 3))
    
    Data_array_skeleton[:,0] = plydata_skeleton['vertex'].data['x']
    Data_array_skeleton[:,1] = plydata_skeleton['vertex'].data['y']
    Data_array_skeleton[:,2] = plydata_skeleton['vertex'].data['z']
    
    X_skeleton = Data_array_skeleton[:,0]
    Y_skeleton = Data_array_skeleton[:,1]
    Z_skeleton = Data_array_skeleton[:,2]
    
    ####################################################################
    #mesh = trimesh.load(model_skeleton)
    
    
    '''
    pcd = o3d.geometry.PointCloud()
    
    pcd.points = o3d.utility.Vector3dVector(Data_array_skeleton)
    
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    
    
    # Build KDTree from point cloud for fast retrieval of nearest neighbors
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    
    print("Paint the 00th point red.")
    
    pcd.colors[0] = [1, 0, 0]
    
    print("Find its 200 nearest neighbors, paint blue.")
    
    [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[0], 100)
    
    np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]
    
    print("Visualize the point cloud.")
    
    o3d.visualization.draw_geometries([pcd])
    
    #build octree, a tree data structure where each internal node has eight children.
    # fit to unit cube
    pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()), center=pcd.get_center())
    pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(num_vertex_skeleton, 3)))
    o3d.visualization.draw_geometries([pcd])

    print('octree division')
    octree = o3d.geometry.Octree(max_depth=4)
    octree.convert_from_point_cloud(pcd, size_expand=0.01)
    o3d.visualization.draw_geometries([octree])

    print(octree.locate_leaf_node(pcd.points[243]))
    '''
    
    ####################################################################

    
    ###################################################################

    
    #Load ply point cloud file
    if not (filename_ptcloud is None):
        
        model_pcloud = current_path + filename_ptcloud
        
        print("Loading 3D point cloud {}...\n".format(filename_ptcloud))
        
        model_pcloud_name_base = os.path.splitext(model_pcloud)[0]
        
        pcd = o3d.io.read_point_cloud(model_pcloud)
        
        Data_array_pcloud = np.asarray(pcd.points)
        
        
        if pcd.has_colors():
            
            print("Render colored point cloud")
            
            pcd_color = np.asarray(pcd.colors)
            
            if len(pcd_color) > 0: 
                
                pcd_color = np.rint(pcd_color * 255.0)
            
            #pcd_color = tuple(map(tuple, pcd_color))
        else:
            
            print("Generate random color")
        
            pcd_color = np.random.randint(256, size = (len(Data_array_pcloud),3))
            
        #print(Data_array_pcloud.shape)
        
        #print(len(Data_array_pcloud))
        
        #print(pcd_color.shape)
        
        #print(type(pcd_color))
    
    
    
#colormap mapping
def get_cmap(n, name = 'hsv'):
    """get the color mapping""" 
    #viridis, BrBG, hsv, copper, Spectral
    #Returns a function that maps each index in 0, 1, ..., n-1 to a distinct 
    #RGB color; the keyword argument name must be a standard mpl colormap name
    return plt.get_cmap(name,n+1)




if __name__ == '__main__':
    
    
    # construct the argument and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", required = True, help = "path to *.ply model file")
    #ap.add_argument("-m1", "--model_skeleton", required = False, help = "skeleton file name")
    ap.add_argument("-m", "--pt_las", required = False, default = None, help = "point cloud model file name")
    args = vars(ap.parse_args())


    # setting input path to model file 
    current_path = args["path"]
    #current_path = os.path.join(current_path, '')
    
    #folder_name = os.path.basename(os.path.dirname(current_path))
    
    filename_las = args["pt_las"]
    
    pt_las_file = current_path + filename_las
    
   
    (file_path, filename, basename) = get_file_info(pt_las_file)

    print("Loading 3D point cloud file in las format {},{},{}...\n".format(file_path, filename, basename))

    
    # Load the LAS file
    las = laspy.read(pt_las_file)
    
    
    #An important point to note is the dimensions X, Y, Z are signed integers without the scale and offset applied. An offset is co-ordinate that moves the entire point cloud by some vector. A scale on the other hand defines by what value should the points be multiplied, to get the actual global point co-ordinates
    
    print(las.header)

    # Output - <LasHeader(1.2, <PointFormat(3, 0 bytes of extra dims)>)>
    # Indicates this file uses Point format 3.

    # Get point format
    point_format = las.point_format
    print(list(point_format.dimension_names))

    print(point_format[3].name)
    print(point_format[3].num_bits)
    print(point_format[3].kind)
    
    # Get number of points
    print(las.header.point_count)

    # Get the offset
    print(las.header.offset)
    # Get scale
    print(las.header.scale)

    # Get min and max of scaled and offset actual values
    print(las.header.min)
    print(las.header.max)

    # Get first points
    print(las.points[0].x)
    print(las.points[0].y)
    print(las.points[0].z)


    point_records = las.points

    # get coordinates values
    x_coords = las.x
    y_coords = las.y
    z_coords = las.z
    
    # get the segmentaion label index
    pt_index = las.extra
    
    unique_pt_index = list(set(pt_index))
    
    # Extract the point data (coordinates)
    points = np.vstack((las.x, las.y, las.z)).transpose()
    
    print(points.shape)
    
    
    
    

    print(f"Number of points: {len(point_records)}")
    print(f"First 10 X coordinates: {x_coords[:10]}")
    print(f"First 10 X Y Z coordinates: {points[:10]}")
    print(f"pt_index: {pt_index}")
    print(f"pt_index: {len(pt_index)}")
    print(f"unique_pt_index: {unique_pt_index}")
    

    
    
    # get number of segmentation 
    N_segmentation = len(unique_pt_index)
    
    # generate N_segmentation different kinds of colors 
    cmap = get_cmap(N_segmentation)
    
    # initilize empty array to save each RGB colors of all points
    color_array = np.empty(shape=(len(point_records), 3), dtype=float)
    
    #print(color_array)
    
    # assign colors based on the segmentation labels
    for idx in range(N_segmentation):
        
        # get current color value in tuple 
        color_rgb = cmap(idx)[:len(cmap(idx))-1]
        
        # get current segmentation label value
        curr_label = unique_pt_index[idx]
    
        print("idx = {}, color_rgb = {}, curr_label = {}".format(idx, color_rgb, curr_label))

        # get indices of all points with current segmentation label value
        row_indices = [i for i, x in enumerate(pt_index) if x == curr_label]
        
        # assign color values for current segmentation points
        color_array[row_indices,:] = np.array(color_rgb)
        

    
    # Create an Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    
    pcd.points = o3d.utility.Vector3dVector(points)
    
    #pcd.paint_uniform_color(color_rgb)
    
    pcd.colors = o3d.utility.Vector3dVector(color_array)
    
    #pcd.paint_uniform_color([0, 0, 1])
    
    o3d.visualization.draw_geometries([pcd])
    
    

    
    #result_path = current_path + basename + '.ply'
    
    #print ("results_folder: {}\n".format(result_path))


    #write out point cloud file
    #o3d.io.write_point_cloud(result_path, pcd, write_ascii = True)
    
    
    
    
    
    ###################################################################
    # check file name input and default file name
    '''
    if args["model_skeleton"] is None:

        # search for file with default name
        filename_skeleton = current_path + folder_name + '_skeleton.ply'
        
        print(filename_skeleton)
        
        if os.path.isfile(filename_skeleton):
            print("Default skeleton file: {}\n".format(filename_skeleton))
            filename_skeleton = folder_name + '_skeleton.ply'
        else:
            print("Skeleton model is not found!\n")
            sys.exit()
    else:
        filename_skeleton = args["model_skeleton"]
    

    if args["model_pcloud"] is None:
        
        # search for file with default name
        filename_ptcloud = current_path + folder_name + '_aligned.ply'
        
        if os.path.isfile(filename_ptcloud):
            print("Default model file: {}\n".format(filename_ptcloud))
            filename_ptcloud = folder_name + '_aligned.ply'
        else:
            print("Aligned pointclod model is not found!\n")
            sys.exit()

    else:
        filename_ptcloud = args["model_pcloud"]
    
    
    #file_path = current_path + filename

    print ("results_folder: " + current_path)

    visualize_skeleton(current_path, filename_skeleton, filename_ptcloud)
    '''
 
