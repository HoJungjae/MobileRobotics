# Copyright 2024 Peter, Jessica

import asyncio
import sys
import cv2
import numpy as np
import cozmo
import time
import os
import math as math
from glob import glob
from cozmo.util import Angle
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
from statemachine import StateMachine, State
from PIL import Image, ImageDraw, ImageFont
from find_cube import *
try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
def nothing(x):
    pass

# Declare the Finite State Machine
class Start(StateMachine):
    search = State(initial=True)
    move = State()
    stop = State()

    cycle = (
        search.to(move)
        | move.to(stop)
        | stop.to(search)
    )
     
    def before_cycle(self, event: str, source: State, target: State, message: str = ""):
        message = ". " + message if message else ""
        return f"Running {event} from {source.id} to {target.id}{message}"

    def on_enter_search(self):
        print("Entering search state!")

    def on_exit_search(self):
        print("Exiting search state!")
    
    def on_enter_move(self):
        print("Entering move state!")

    def on_exit_move(self):
        print("Exiting move state!")

    def on_enter_stop(self):
        print("Entering stop state!")

    def on_exit_stop(self):
        print("Exiting stop state!")

# Global variables
class SharedData:
    marker_loc = None
    marker_vect = None
    state_machine = None
    face_display = None
    cube1_complete = False
    create_diamond = False
    cube2_start = False
    task_complete = False
    custom_cube = None
    cozmo = False
    last_y = 0
    gone = False
    cube2_rdy = False

shared_data = SharedData()

# Event handling functions
def handle_object_appeared(evt, **kw):
    # This will be called whenever an object comes into cozmo's view.
    if isinstance(evt.obj, CustomObject):
        print("Cozmo started seeing a %s" % str(evt.obj.object_type))
        shared_data.state_machine.send("cycle")
        shared_data.cozmo = True
        shared_data.face_display = "move.png"
        marker_coord = evt.obj.pose.position
        shared_data.marker_vect = [[marker_coord.x], [marker_coord.y], [1]]
        shared_data.gone = False  

def handle_object_disappeared(evt, **kw):
    # This will be called whenever an object goes out of cozmo's view.
    if isinstance(evt.obj, CustomObject):
        shared_data.gone = True

def handle_object_observed(evt, **kw):
    # This will be called whenever an object is visually identified by cozmo.
    if isinstance(evt.obj, CustomObject):
        marker_coord = evt.obj.pose.position
        shared_data.marker_vect = [[marker_coord.x], [marker_coord.y], [1]] 
        shared_data.obj_disap = False
        if shared_data.custom_cube.object_type is cozmo.objects.CustomObjectTypes.CustomType01:
            shared_data.cube2_rdy = True

    
def transformation(robot, marker_pose):
    # This calculates the position of the cube with respect to cozmo's world
    current_pose = robot.pose
    current_x, current_y = current_pose.position.x, current_pose.position.y  #translation vect
    rotation = current_pose.rotation
    angle_radians = rotation.angle_z.radians

    transform_matrix = [[math.cos(angle_radians), math.sin(angle_radians), -1*(math.cos(angle_radians)*current_x + math.sin(angle_radians)*current_y)],
              [-1*math.sin(angle_radians), math.cos(angle_radians), -1*(-1*math.sin(angle_radians)*current_x + math.cos(angle_radians)*current_y)],
              [0, 0, 1]]
    result = [[0],[0],[0]]

    if marker_pose is not None:
        result = np.matmul(np.array(transform_matrix), np.array(marker_pose))
        return result
    else:
        return None
    
async def run(robot: cozmo.robot.Robot):
    # Setting the lightning
    robot.world.image_annotator.annotation_enabled = False
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True
    fixed_gain, exposure, mode = 2, 3, 1

    shared_data.face_display = "search.png"
    
    # Initialize fsm
    shared_data.state_machine = Start()
    
    notes = [
        cozmo.song.SongNote(cozmo.song.NoteTypes.C2, cozmo.song.NoteDurations.Quarter)
    ]
    
    # Add event handlers for whenever Cozmo sees a new object
    robot.add_event_handler(cozmo.objects.EvtObjectAppeared, handle_object_appeared)
    robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, handle_object_disappeared)
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, handle_object_observed)
    
    #Set Cozmo head angle
    await robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE - cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()
    #Ignore cozmo logs
    cozmo.logger.setLevel(0)
    
    # Start the stuff
    try:
        while True:
            
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   #get camera image
            if event.image is not None:
                image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)
                time.sleep(0.01)
                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure, fixed_gain)

                if shared_data.cozmo == True:
                    shared_data.cozmo = False
                    robot.play_audio(cozmo.audio.AudioEvents.SfxSharedTimerEnd)
                if shared_data.create_diamond is False:
                    shared_data.custom_cube = await robot.world.define_custom_cube(CustomObjectTypes.CustomType00, CustomObjectMarkers.Triangles2, 30, 30, 30, True)
                else:
                    shared_data.custom_cube = await robot.world.define_custom_cube(CustomObjectTypes.CustomType01, CustomObjectMarkers.Hexagons2, 30, 30, 30, True)
                    print("inside define function - ", shared_data.custom_cube.object_type)

                # Using the pose of the robot and the marker, find the pose of the marker w.r.t the robot.
                shared_data.marker_loc = transformation(robot, shared_data.marker_vect)

                # Displaying state on Cozmo Face
                current_directory = os.path.dirname(os.path.realpath(__file__))
                hello_png = os.path.join(current_directory, shared_data.face_display)
                image = Image.open(hello_png)

                # Resize to fit on Cozmo's face screen
                resized_image = image.resize(cozmo.oled_face.dimensions(), Image.NEAREST)

                # Convert the image to the format used by the oled screen
                face_image = cozmo.oled_face.convert_image_to_screen_data(resized_image, invert_image=True)
                
                # Display image on cozmo's screen
                robot.display_oled_face_image(face_image, 1000)
                    
                # First cube moving
                if shared_data.marker_loc is not None and shared_data.task_complete is False and shared_data.gone is False:
                    shared_data.last_y = shared_data.marker_loc[1]
                    angle = math.degrees(math.atan(shared_data.marker_loc[1] / shared_data.marker_loc[0]))
                    print(shared_data.marker_loc[0])
                    if shared_data.marker_loc[0] > 115:
                        robot.drive_wheel_motors(20, 20)
                        if angle > 2:
                            robot.drive_wheel_motors(15, 30)
                        elif angle < -2:
                            robot.drive_wheel_motors(30, 15)
                    elif shared_data.cube1_complete is False:
                        print("Reached Cube 1 elif")
                        robot.stop_all_motors()
                        # Found Cube1 and changing state to stop
                        shared_data.state_machine.send("cycle")
                        robot.play_audio(cozmo.audio.AudioEvents.SfxSharedTimerEnd)
                        shared_data.face_display = "stop.png"
                        shared_data.cube1_complete = True
                        shared_data.cube2_start = True
                        shared_data.marker_vect = None
                        shared_data.create_diamond = True
                        # Changing state to search to search for cube 2
                        shared_data.state_machine.send("cycle")
                        robot.play_audio(cozmo.audio.AudioEvents.SfxSharedTimerEnd)
                        shared_data.face_display = "search.png"
                        print("Reached first cube!")
                    elif shared_data.cube2_rdy is True:
                        print("Reached Cube 2 else")
                        robot.stop_all_motors()
                        shared_data.state_machine.send("cycle")
                        shared_data.face_display = "stop.png"
                        print("Reached second cube!")
                else:
                    if shared_data.task_complete is False and shared_data.last_y > 0:
                        robot.drive_wheel_motors(-10, 10)
                    elif shared_data.task_complete is False and shared_data.last_y < 0:
                        robot.drive_wheel_motors(10, -10)
                    elif shared_data.task_complete is True:
                        robot.stop_all_motors()
                    else:
                        robot.drive_wheel_motors(10, -10)

    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
