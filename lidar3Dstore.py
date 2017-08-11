#Display Data from Neato LIDAR
#based on code from Nicolas "Xevel" Saugnier
#requires vpython and pyserial

from threading import Thread
import time, sys, traceback, math, serial, collections, json
import pandas as pd
import altMaestro
import rotation as rot
#import moveUnit as move

com_port = "COM3" # example: 5 == "COM6" == "/dev/tty5"
baudrate = 115200
visualization = True

offset = 140
init_level = 0
index = 0

# Read the set of yaw and pitch angles to be scanned from 
with open('scan_angles.json', 'r') as f:
    positions =  json.load(f)
    
yaw_set = positions['yaw_angles']
yaw_set = { float(x): yaw_set[x] for x in yaw_set.keys() } # json insists on keys being strings, this converts to floats
pitch_set = positions['pitch_angles']
pitch_set = { float(x): pitch_set[x] for x in pitch_set.keys() } # json insists on keys being strings, this converts to floats
# Want to proceed smoothly from left to right, down to up, so put in order          
yaw_set = collections.OrderedDict(sorted(yaw_set.items(), key=lambda t: t[0]))  
pitch_set = collections.OrderedDict(sorted(pitch_set.items(), key=lambda t: t[0]))           
            
       
num_locations = len(yaw_set) * len(pitch_set)

# Ask user if they want to store the data set or not
scan = True # True until full scan completed
storing = False
input = raw_input('Do you want to save the dataset y/[n]? ')
if input == 'y':
    storing = True
    file_name = raw_input('Enter the filename to save the data to: ')
    columns = ['x_pos', 'y_pos', 'z_pos', 'intensity', 'quality_warning']
    lidar_df = pd.DataFrame(index = range(360*num_locations), columns=columns)


servo = altMaestro.Device('COM4','COM5')
servo.set_acceleration(0, 10)
servo.set_acceleration(1, 10)
servo.set_speed(0, 10)
servo.set_speed(1, 10)
               

lidarData = [[] for i in range(360)] #A list of 360 elements Angle, Distance , quality


if visualization:
    from visual import *
# sample and intensity points
    point = points(pos=[(0,0,0) for i in range(360*num_locations)], size=5, color=(0 , 1, 0))
    pointb = points(pos=[(0,0,0) for i in range(360*num_locations)], size=5, color=(0.4, 0, 0))
    #lines
    outer_line= curve (pos=[(0,0,0) for i in range(360*num_locations)], size=5, color=(1 , 0, 0))
    lines=[curve(pos=[(offset*cos(i* pi / 180.0),0,offset*-sin(i* pi / 180.0)),(offset*cos(i* pi / 180.0),0,offset*-sin(i* pi / 180.0))], color=[(0.1, 0.1, 0.2),(1,0,0)]) for i in range(360*num_locations)]
    lidar = cylinder(pos=(0,-15,0), axis=(0,30,0), radius=37)
    label_speed = label(pos = (0,-500,0), xoffset=1, box=False, opacity=0.1)
    label_errors = label(pos = (0,-1000,0), xoffset=1, text="errors: 0", visible = False, box=False)
    # Display Coordinate Axes
    x_axis = arrow(axis=(500,0,0), shaftwidth=10)
    y_axis = arrow(axis=(0,500,0), shaftwidth=10)
    z_axis = arrow(axis=(0,0,500), shaftwidth=10)
    use_points = True
use_outer_line = False
use_lines = False
use_intensity = False
use_height = True

def update_view(angle, data ):
    """Updates the view of a sample.
    Takes the angle (an int, from 0 to 359) and the list of four bytes of data in the order they arrived.
"""
    global offset, scan
    # unpack data using the denomination used during the discussions
    x = data[0]
    x1= data[1]
    x2= data[2]
    x3= data[3]
    
    angle_rad = angle * math.pi / 180.0
    c = math.cos(angle_rad)
    s = -math.sin(angle_rad)

    dist_mm = x | (( x1 & 0x3f) << 8) # distance is coded on 13 bits ? 14 bits ?
    quality = x2 | (x3 << 8) # quality is on 16 bits
    lidarData[angle] = [dist_mm,quality]

    # Modified for 3D vectors
    raw_dist_x = dist_mm*c
    raw_dist_y = dist_mm*s
    dist_x, dist_y, dist_z = rot.rotation(raw_dist_x, raw_dist_y, 0, yaw_angle, pitch_angle)
    dist_z = -dist_z
    if visualization:
        #reset the point display
        point.pos[angle+(360*loc)] = vector( 0, 0, 0 )
        pointb.pos[angle+(360*loc)] = vector( 0, 0, 0 )

        if not use_lines : lines[angle+(360*loc)].pos[1]=(offset*c,0,offset*s)
        if not use_outer_line :
            outer_line.pos[angle+(360*loc)]=(offset*c,0,offset*s)
            outer_line.color[angle+(360*loc)] = (0.1, 0.1, 0.2)
        
        
        # display the sample
        if x1 & 0x80: # is the flag for "bad data" set?
            # yes it's bad data
            lines[angle+(360*loc)].pos[1]=(offset*c,0,offset*s)
            outer_line.pos[angle+(360*loc)]=(offset*c,0,offset*s)
            outer_line.color[angle+(360*loc)] = (0.1, 0.1, 0.2)
        else:
            # no, it's cool
            if not x1 & 0x40:
                # X+1:6 not set : quality is OK
                if use_intensity or use_height:
                    if use_height:
                        if dist_z <= -10:
                            point.color[angle+(360*loc)] = (1.,0.6,0.) # orange  
                        elif dist_z <= 200:
                            point.color[angle+(360*loc)] = (1.,1,0.) # yellow
                        elif dist_z <= 500:
                            point.color[angle+(360*loc)] = (0.,1,0.) # green 
                        else:
                            point.color[angle+(360*loc)] = (0.,0.,1.) # blue                          
                    if use_intensity:                      
                        if quality < 25:
                            point.color[angle+(360*loc)] = (1.,0.6,0.)
                            print "orange"
                        elif quality < 50:
                            point.color[angle+(360*loc)] = (1.,1.,0.)
                            print "yellow"
                        elif quality < 80:
                            point.color[angle+(360*loc)] = (0.,0.,1.)
                            print "blue"
                if use_points and dist_z > -10: 
                    point.pos[angle+(360*loc)] = vector( dist_x, dist_z, dist_y)
                    if storing == True:
                        lidar_df.loc[angle+(360*loc)] = [dist_x, dist_y, dist_z, quality, 0]                             
                if use_lines : lines[angle+(360*loc)].color[1] = (1,0,0)
                if use_outer_line : outer_line.color[angle+(360*loc)] = (1,0,0)
            else:
                # X+1:6 set : Warning, the quality is not as good as expected
                if use_points and dist_z > -10: 
                    pointb.pos[angle+(360*loc)] = vector( dist_x,dist_z, dist_y)
                    if storing == True:
                        lidar_df.loc[angle+(360*loc)] = [dist_x, dist_y, dist_z, quality, 1] 
                if use_lines : lines[angle+(360*loc)].color[1] = (0.4,0,0)
                if use_outer_line : outer_line.color[angle+(360*loc)] = (0.4,0,0)
            if use_lines : lines[angle+(360*loc)].pos[1]=( dist_x, dist_z, dist_y)
            if use_outer_line : outer_line.pos[angle+(360*loc)]=( dist_x, dist_z, dist_y)            
              
            
def checksum(data):
    """Compute and return the checksum as an int.

data -- list of 20 bytes (as ints), in the order they arrived in.
"""
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append( data[2*t] + (data[2*t+1]<<8) )
    
    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d

    # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
    checksum = checksum & 0x7FFF # truncate to 15 bits
    return int( checksum )


def gui_update_speed(speed_rpm):
    label_speed.text = "RPM : " + str(speed_rpm)

def compute_speed(data):
    speed_rpm = float( data[0] | (data[1] << 8) ) / 64.0
    return speed_rpm

def read_Lidar():
    global init_level, angle, index
    nb_errors = 0
    while scan:
        try:            
            time.sleep(0.00001) # do not hog the processor power
            if init_level == 0 :
                b = ord(ser.read(1))
                # start byte
                if b == 0xFA :
                    init_level = 1
                else:
                    init_level = 0
            elif init_level == 1:
                # position index
                b = ord(ser.read(1))
                if b >= 0xA0 and b <= 0xF9 :
                    index = b - 0xA0
                    init_level = 2
                elif b != 0xFA:
                    init_level = 0
            elif init_level == 2 :
                # speed
                b_speed = [ ord(b) for b in ser.read(2)]
                
                # data
                b_data0 = [ ord(b) for b in ser.read(4)]
                b_data1 = [ ord(b) for b in ser.read(4)]
                b_data2 = [ ord(b) for b in ser.read(4)]
                b_data3 = [ ord(b) for b in ser.read(4)]

                # for the checksum, we need all the data of the packet...
                # this could be collected in a more elegent fashion...
                all_data = [ 0xFA, index+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3

                # checksum
                b_checksum = [ ord(b) for b in ser.read(2) ]
                incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)
    
                # verify that the received checksum is equal to the one computed from the data
                if checksum(all_data) == incoming_checksum:
                    speed_rpm = compute_speed(b_speed)
                    if visualization:
                        gui_update_speed(speed_rpm)
                    update_view(index * 4 + 1, b_data1)
                    update_view(index * 4 + 2, b_data2)
                    update_view(index * 4 + 3, b_data3)
                else:
                    # the checksum does not match, something went wrong...
                    nb_errors +=1
                    if visualization:
                        label_errors.text = "errors: "+str(nb_errors)
                    
                    # display the samples in an error state
                    update_view(index * 4 + 0, [0, 0x80, 0, 0])
                    update_view(index * 4 + 1, [0, 0x80, 0, 0])
                    update_view(index * 4 + 2, [0, 0x80, 0, 0])
                    update_view(index * 4 + 3, [0, 0x80, 0, 0])
                    
                init_level = 0 # reset and wait for the next packet
                
            else: # default, should never happen...
                init_level = 0
                                     
        except :
            traceback.print_exc(file=sys.stdout)
    return()

def store_snapshot(fn):
    lidar_df.to_csv(file_name)
    return()  
          
ser = serial.Serial(com_port, baudrate)

th1 = Thread(target=read_Lidar)
th1.daemon = True
th1.start()

start_time = time.time()
move_flag = True
yaw_index = 0
pitch_index = 0
loc = -1   
while scan:
    if visualization:
        rate(24) # synchonous repaint at 24fps
        yaw_angle = yaw_set.keys()[yaw_index]
        pitch_angle = pitch_set.keys()[pitch_index]
        if move_flag == True:
            loc += 1
            servo.set_target(0,yaw_set.values()[yaw_index]) 
            servo.set_target(1,pitch_set.values()[pitch_index])
            while (servo.is_moving(0) or servo.is_moving(1))  == True:
                time.sleep(0.001)
        elapsed_time = time.time() - start_time
        if elapsed_time < 3: 
            move_flag = False
        else:
            move_flag = True
            start_time = time.time()
            if pitch_index < len(pitch_set) - 1:
                pitch_index += 1
            else:
                pitch_index = 0
                if yaw_index < len(yaw_set) - 1:
                    yaw_index += 1
                else:
                    print 'Scan complete'
                    if storing == True:
                        print 'Storing snapshot in ' + file_name
                        store_snapshot(file_name)
                        print 'Finished'
                    scan = False




            
            