#Display stored datasets from Neato LIDAR
#by Mike McGurrin ("ViennaMike")
#based on original real-time visualization code from Nicolas "Xevel" Saugnier


#requires vpython and pandas
import math
import pandas as pd
from visual import *

use_axes = True
use_points = True
use_outer_line = False
use_lines = False
use_intensity = True
use_height = False
offset = 140
    
def init():
    global point, pointb, pointc, pointh, outer_line, lines, lidar, x_axis, y_axis, z_axis
    # Ask user what file to display
    while True:
        try:
            file_name = raw_input('Enter the filename of the csv file with the dataset: ')   
            df = pd.read_csv(file_name, index_col = 0 )
            break
        except:
            print "That file name doesn't seem to be valid."
            #better try again... Return to the start of the loop
            continue
        
    num_points = len(df)
    # Set or reset the displayed sample and intensity points
    # point is for showing good data, displayed in green
    #pointb is for showing data with less intensity than expected for range
    #poincc is for showing the intensity level
    point = points(pos=[(0,0,0) for i in range(num_points)], size=5, color=(0 , 1, 0))
    pointb = points(pos=[(0,0,0) for i in range(num_points)], size=5, color=(0.4, 0, 0))
    pointc = points(pos=[(0,0,0) for i in range(num_points)], size=5, color=(0.4, 0, 0))
    pointh = points(pos=[(0,0,0) for i in range(num_points)], size=5, color=(0.4, 0, 0))    
    #lines
    outer_line= curve (pos=[(0,0,0) for i in range(num_points)], size=5, color=(1 , 0, 0))
    lines=[curve(pos=[(offset*cos(i* pi / 180.0),0,offset*-sin(i* pi / 180.0)),(offset*cos(i* pi / 180.0),0,offset*-sin(i* pi / 180.0))], color=[(0.1, 0.1, 0.2),(1,0,0)]) for i in range(num_points)]
    lidar = cylinder(pos=(0,-15,0), axis=(0,30,0), radius=37)
    x_axis = arrow(axis=(500,0,0), shaftwidth=10)
    y_axis = arrow(axis=(0,500,0), shaftwidth=10)
    z_axis = arrow(axis=(0,0,500), shaftwidth=10)
    
    for angle, row in df.iterrows():
        if row['quality_warning'] == 1: # intensity less than expected for given range
            pointb.pos[angle] = vector(row['x_pos'],row['z_pos'], row['y_pos'])
            lines[angle].color[1] = (0.4,0,0)
            outer_line.color[angle] = (0.4,0,0)
    
        else:  # intensity is fine
            point.pos[angle] = vector(row['x_pos'],row['z_pos'], row['y_pos'])
            point.color[angle] = (0 , 1, 0)
            lines[angle].color[1] = (1,0,0)
            outer_line.color[angle] = (1,0,0)

        # Set up a point set with intensity coded colors
        pointc.pos[angle] = vector(row['x_pos'],row['z_pos'],row['y_pos'])
        if row['intensity'] < 10:
            pointc.color[angle] = (1,0.6,0)
        elif row['intensity'] < 30:
            pointc.color[angle] = (1,1,0)
        elif row['intensity'] < 60:
            pointc.color[angle] = (0,0,1)
        else:
            pointc.color[angle] = (0 , 1, 0)
            
        # Set up a point set with height coded colors
        pointh.pos[angle] = vector(row['x_pos'],row['z_pos'], row['y_pos'])
        if row['z_pos'] <= -10:
            pointh.color[angle] = (1.,0.6,0.) # orange  
        elif row['z_pos'] <= 200:
            pointh.color[angle] = (1.,1,0.) # yellow
        elif row['z_pos'] <= 500:
            pointh.color[angle] = (0.,1,0.) # green 
        else:
            pointh.color[angle] = (0.,0.,1.) # blue         

        lines[angle].pos[1]= vector(row['x_pos'],row['z_pos'], row['y_pos'])
        outer_line.pos[angle]= vector(row['x_pos'],row['z_pos'], row['y_pos'])

    if use_axes:
        x_axis.visible = True
        y_axis.visible = True
        z_axis.visible = True
    else:
        x_axis.visible = False 
        y_axis.visible = False
        z_axis.visible = False
    
    return(df)

def checkKeys(df):
    global use_outer_line, use_lines, use_points, use_intensity, use_height, use_axes
    global point, pointb, pointc, pointh, lines, outer_line, x_axis, y_axis, z_axis
    if scene.kb.keys: # event waiting to be processed?
        s = scene.kb.getkey() # get keyboard info
        if s == "a": # toggle display coordinate axes
            use_axes = not use_axes
        if s=="o": # Toggle outer line
            use_outer_line = not use_outer_line
        elif s=="l": # Toggle rays
            use_lines = not use_lines
        elif s=="p": # Toggle points
            use_points = not use_points
        elif s=="i": # Toggle intensity
            use_intensity = not use_intensity
            if use_intensity == True:
                use_height = False
        elif s=="h": # Toggle height
            use_height = not use_height
            if use_height == True:
                use_intensity = False
        elif s=='s': # select new dataset
            # Must make all existing data points and lines disappear and then delete them
            point.visible = False; del point
            pointb.visible = False; del pointb
            pointc.visible = False; del pointc
            pointh.visible = False; del pointh
            for line in lines:
                line.visible = False; del line
            outer_line.visible = False; del outer_line
            x_axis.visible = False; del x_axis 
            y_axis.visible = False; del y_axis
            z_axis.visible = False; del z_axis
            init()
    
    # Update view based on keys
    for angle, row in df.iterrows():
        angle_radians = math.radians(angle)
        c = math.cos(angle_radians)
        s = math.sin(angle_radians)

    if use_points:
        if use_intensity:
            point.visible = False
            pointb.visible = False
            pointh.visible = False
            pointc.visible = True
        elif use_height:
            point.visible = False
            pointb.visible = False
            pointc.visible = False
            pointh.visible = True
        else:
            point.visible = True
            pointb.visible = True
            pointc.visible = False  
            pointh.visible = False
    else:
        pointb.visible = False
        point.visible = False
        pointc.visible = False
        pointh.visible = False

    if use_lines:
        for line in lines:
            line.visible = True
    else:
        for line in lines:
            line.visible = False
    
    if use_outer_line:
        outer_line.visible = True
    else:
        outer_line.visible = False
    
    if use_axes:
        x_axis.visible = True
        y_axis.visible = True
        z_axis.visible = True
    else:
        x_axis.visible = False 
        y_axis.visible = False3D_Test2.c3D_
        z_axis.visible = False

lidar_df = init()

while True:
    rate(5) # synchonous repaint at 40fps
    checkKeys(lidar_df)

    
