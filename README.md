# Neato-XV-Lidar-Tools
Software for visualizing, saving, and viewing the output of a Neato XV-11 lidar, for both 2D and crude 3D scans. The main programs are 
a 2D visualization program (lidar2Dstore.py) for viewing and saving 2D scans, lidar3Dstore.py for conducting, displaying, and saving 
crude 3D scans, and display_lidar_data.py for viewing previously saved lidar point cloud data.

Requires a Neato lidar and the XV Lidar Controller from Surreal. 3D use also requires a pan/tilt servo kit and a Pololu Maestro Servo Controller.

## Additional Info ##

lidar3Dstore uses a json file (scan_angles.json) to store the servo command and corresponding angles to cycle through for the pan and tilt 
servos when conducting a scan. 
rotation.py is a subroutine to do the necessary 3D coordinate conversions and altMaestro.py handles the interface to the servo controller.

The visualization is done using VPython 6, and the programs are all written to run in Python 2.7. I'd welcome anyone porting this to 
VPython 7 and Python 3, but if you do so, keep in mind that VPython 6 won't run in Python 3.X, while VPython 7 has some issues with 
some IDE's, especially in Python 2.x (at least as of this writing).

Because of how the display controls work in VPython, the authors of the original software I modified to create this swapped the y and z axis 
in the display. I have kept that convention. 

Included in the repository are sample stored 2D and 3D point data clouds.

## Authors ##

The original 2D display software was developed by Nicolas "Xevel" Saugnier
Others who did not provide their names went on to modify the code, 
and Mike McGurrin has further modified the 2D version and developed the 3D and display_lidar_data routines from that same base.

## License ##
The original code was released under the Apache license, so for simplicity, this modified version as well as the new code is released 
under the same license.

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

