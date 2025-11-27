#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

"""
Remote Control Transmitter for BFMC Simulator

This module provides keyboard-based remote control for the simulated car in the
BFMC (Bosch Future Mobility Challenge) simulator. It captures keyboard input and
translates it into ROS 2 command messages that control the car's speed and steering.

The controller uses the pynput library to capture keyboard events and publishes
JSON-formatted commands to the /automobile/command ROS 2 topic.

Key Controls:
    - w,a,s,d: Directional control
    - t,g,y,h,u,j,i,k,r,p: Parameter adjustment keys
    - z,x,v,b,n,m: PID tuning keys
    - ESC: Emergency stop and exit

Example:
    To run the control interface:
        $ ros2 run example tl_talker
"""

import json
from pynput import keyboard

from .RcBrainThread import RcBrainThread
from std_msgs.msg import String
import rclpy

class RemoteControlTransmitterProcess():
    """Remote control transmitter for the BFMC simulator car.
    
    This class handles keyboard input and converts it into control commands
    for the simulated RC car. It manages a ROS 2 publisher to send commands
    and uses pynput to capture keyboard events.
    
    Attributes:
        dirKeys (list): Direction control keys (w,a,s,d)
        paramKeys (list): Parameter adjustment keys
        pidKeys (list): PID tuning keys
        allKeys (list): Combined list of all recognized keys
        rcBrain (RcBrainThread): Brain thread for command processing
        publisher (Publisher): ROS 2 publisher for commands
    """
    
    def __init__(self):
        """Initialize the remote control transmitter.
        
        Sets up the ROS 2 node and publisher, initializes key mappings,
        and creates the RC brain thread for command processing.
        """
        self.dirKeys   = ['w', 'a', 's', 'd']
        self.paramKeys = ['t','g','y','h','u','j','i','k', 'r', 'p']
        self.pidKeys = ['z','x','v','b','n','m']

        self.allKeys = self.dirKeys + self.paramKeys + self.pidKeys
        
        self.rcBrain   =  RcBrainThread()   
        
        rclpy.init()
        node = rclpy.create_node('EXAMPLEnode')     
        self.publisher = node.create_publisher(String, '/automobile/command', 1)

    def run(self):
        """Start the keyboard listener and begin processing input.
        
        This method blocks until the ESC key is pressed or the listener
        is otherwise terminated.
        """
        with keyboard.Listener(on_press = self.keyPress, on_release = self.keyRelease) as listener: 
            print("Starting remote control transmitter...")
            print("Press ESC to exit")
            listener.join()

    def keyPress(self,key):
        """Handle key press events.
        
        Processes alphanumeric key presses and converts them into
        command messages for the RC car.
        
        Args:
            key (pynput.keyboard.Key): The key that was pressed
        """                                     
        print(key)
        # Check if key has char attribute (alphanumeric keys)
        if hasattr(key, 'char') and key.char in self.allKeys:
            keyMsg = 'p.' + str(key.char)
            self._send_command(keyMsg)
        
    def keyRelease(self, key):
        """Handle key release events.
        
        Processes key releases, including the ESC key for emergency stop
        and exit. Sends appropriate commands when control keys are released.
        
        Args:
            key (pynput.keyboard.Key): The key that was released
            
        Returns:
            bool: False if ESC pressed (terminates listener), True otherwise
        """ 
        if key == keyboard.Key.esc:
            # Emergency stop: send brake command before exiting
            self.publisher.publish(String(data='{"action":"3","steerAngle":0.0}'))
            return False

        # Check if key has char attribute (alphanumeric keys)
        if hasattr(key, 'char') and key.char in self.allKeys:
            keyMsg = 'r.'+str(key.char)
            self._send_command(keyMsg)
                 
    def _send_command(self, key):
        """Send control command to the simulator.
        
        Converts the key input into a JSON command using the RcBrainThread
        and publishes it to the /automobile/command topic.
        
        Args:
            key (str): Key identifier string (e.g., 'p.w', 'r.s')
        """
        command = self.rcBrain.getMessage(key)
        if command is not None:
            command = json.dumps(command)
            command = String(data=command)
            self.publisher.publish(command)  


def main():
    """Main entry point for the remote control node."""
    nod = RemoteControlTransmitterProcess()
    nod.run()
            

if __name__ == '__main__':
    nod = RemoteControlTransmitterProcess()
    nod.run()
