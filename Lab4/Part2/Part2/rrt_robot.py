#Copyright 2024 Peter, Jessica
import cozmo
import math
from cmap import *
from gui import *
from utils import *
from time import sleep

MAX_NODES = 20000
OFFSET = 50

################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, a nodes is an object that has its own
# coordinate and parent if necessary. You could access its coordinate by node.x
# or node[0] for the x coordinate, and node.y or node[1] for the y coordinate
################################################################################

def step_from_to(node0, node1, limit=30):
    ############################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    if get_dist(node0, node1) < limit:
        return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    
    # Calculate direction from node0 to node1
    dx = node1.x - node0.x
    dy = node1.y - node0.y
    angle = np.arctan2(dy, dx)

    #create a new node limit distance away in the direction of node1
    new_node = Node((node0.x + limit * np.cos(angle), node0.y + limit * np.sin(angle)))
    return new_node
    ############################################################################


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    # Generate random x and y coordinates within the map boundaries
    while rand_node is None:
        x = np.random.uniform(0, cmap.width)
        y = np.random.uniform(0, cmap.height)
        potential_node = Node((x,y))

        # Check if the node is within the map boundaries and not inside any obstacles
        if cmap.is_inbound(potential_node) and not cmap.is_inside_obstacles(potential_node):
            rand_node = potential_node
    ############################################################################
    return rand_node


def RRT(cmap, start):
    cmap.add_node(start)

    map_width, map_height = cmap.get_size()

    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        rand_node = cmap.get_random_valid_node()

        # 2. Get the nearest node to the random node from RRT
        nearest_node = None
        min_dist = float('inf')
        for n in cmap.get_nodes():
            dist = get_dist(n, rand_node)
            if dist < min_dist:
                min_dist = dist
                nearest_node = n

        # 3. Limit the distance RRT can move
        new_node = step_from_to(nearest_node, rand_node)

        # 4. Add one path from nearest node to random node
        cmap.add_path(nearest_node, new_node)
        
        ########################################################################
        sleep(0.01)
        if cmap.is_solved():
            break

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
    else:
        print("Please try again :-(")


async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    await robot.set_head_angle(cozmo.robot.MIN_HEAD_ANGLE - cozmo.robot.MIN_HEAD_ANGLE).wait_for_completed()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions
    # (1) identifying a target cube, 
    # (2) using RRT to find a path to a specific face of the cube,
    # (3) following the path found by RRT, and 
    # (4) replanting to avoid any obstacle cubes that are added during
    
# SET UP THE COORDINATES, DO THE TRANSFORMATION
    map_x, map_y = cmap.get_size()
    cmap.set_start(Node(tuple([map_x/2, map_y/2])))
    cube = robot.world.get_light_cube(cube_id=1)
    while cube.is_visible is False:
        await robot.turn_in_place(angle=cozmo.util.Angle(degrees=10), in_parallel=True).wait_for_completed()
        cube = robot.world.get_light_cube(cube_id=1)
    if cube.is_visible:
        # Get the translation vector
        cozmo_pose = robot.pose
        cozmo_x, cozmo_y = cozmo_pose.position.x, cozmo_pose.position.y  #r_x and r_y
        rotation = cozmo_pose.rotation
        cozmo_theta = rotation.angle_z.radians
        robot_vect = [cozmo_x, cozmo_y, 1]

        translation_matrix = [[math.cos(0), -1*math.sin(0), map_x/2],
                [math.sin(0), math.cos(0), map_y/2],
                [0, 0, 1]]
        translation_vect = [[0],[0],[0]]
        translation_vect = np.matmul(np.array(translation_matrix), np.array(robot_vect).reshape((3, 1)))
        print(f"Translation_vect: {translation_vect}")
        # Transform the pose of cube from cozmo world -> map world
        cube_x, cube_y, cube_theta = cube.pose.position.x, cube.pose.position.y, cube.pose.rotation.angle_z.radians
        cube_vect = [cube_x, cube_y, 1]
        print(f"Cube Vect: {cube_vect}")
        transformation_matrix = [[math.cos(-cozmo_theta), -1*math.sin(-cozmo_theta), translation_vect[0][0]],
                [math.sin(-cozmo_theta), math.cos(-cozmo_theta), translation_vect[1][0]],
                [0, 0, 1]]
        
        updated_cube_vect = [[0],[0],[0]]
        updated_cube_vect = np.matmul(np.array(transformation_matrix), np.array(cube_vect).reshape((3, 1)))

        print("Cube pose w.r.t map is: \n", updated_cube_vect)
        nodeA = Node([updated_cube_vect[0][0] - OFFSET, updated_cube_vect[1][0] - OFFSET])
        nodeB = Node([updated_cube_vect[0][0] - OFFSET, updated_cube_vect[1][0] + OFFSET])
        nodeC = Node([updated_cube_vect[0][0] + OFFSET, updated_cube_vect[1][0] - OFFSET])
        nodeD = Node([updated_cube_vect[0][0] + OFFSET, updated_cube_vect[1][0] + OFFSET])
        nodes = [nodeB, nodeA, nodeC, nodeD]
        cmap.add_obstacle(nodes)
        cmap.add_goal(Node(tuple([updated_cube_vect[0][0] - OFFSET - 10, updated_cube_vect[1][0]])))
        goal = cmap.get_goals()
        print(goal)
        print(f"Goal added - x: {goal[0].x}, y: {goal[0].y}, theta: {math.degrees(cube_theta)}")
    else:
        print("not see a cube")    

    # print out the starting point of map
    starting_point = cmap.get_start()
    print("starting point: ", starting_point.x, " ", starting_point.y)

# USE RRT TO FIND A PATH, MOVE TOWARDS THE PATH
    # Instead of going to straight to the goal, we need to go to the next node

    # Step 1: Figure out the node we need to go to, get x and y of that next node
    RRT(cmap, cmap.get_start())
    ob1_found = False
    ob2_found = False
    while cmap._start is not goal[0]:
        # Starting at goal
        cur = goal[0]
        # Find next node to travel to
        while cur.parent is not cmap._start:
            cur = cur.parent
        # Update the current position of cozmo on the path.
        starting_point = cmap.get_start()
        cmap._start = cur
        # Step 2: Calculate distance from current position/starting point to the next node/goal
        delta_x = abs(starting_point.x - cur.x)
        delta_y = abs(starting_point.y - cur.y)
        distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))

        # Step 3: Move (using the distance and the angle)
        if starting_point.x < cur.x:
            theta = math.atan((cur.y - starting_point.y) / (cur.x - starting_point.x))
            await robot.turn_in_place(angle=cozmo.util.Angle(theta)).wait_for_completed()
            await robot.drive_straight(distance=cozmo.util.distance_mm(distance*1), speed=cozmo.util.speed_mmps(15)).wait_for_completed()
            await robot.turn_in_place(angle=cozmo.util.Angle(-theta)).wait_for_completed()
        else:
            theta = math.atan((cur.y - starting_point.y) / (cur.x - starting_point.x))
            await robot.turn_in_place(angle=cozmo.util.Angle(degrees=180)).wait_for_completed()
            await robot.turn_in_place(angle=cozmo.util.Angle(theta)).wait_for_completed()
            await robot.drive_straight(distance=cozmo.util.distance_mm(distance*1), speed=cozmo.util.speed_mmps(15)).wait_for_completed()
            await robot.turn_in_place(angle=cozmo.util.Angle(-theta)).wait_for_completed()
            await robot.turn_in_place(angle=cozmo.util.Angle(degrees=180)).wait_for_completed()

        # Step 4: Replanting the map (check if we see any other cubes, add them as obstacles in the map)
        obstacle_1 = robot.world.get_light_cube(cube_id=2)
        obstacle_2 = robot.world.get_light_cube(cube_id=3)

        if obstacle_1 is not None and ob1_found is False:
            cozmo_pose = robot.pose
            cozmo_x, cozmo_y = cozmo_pose.position.x, cozmo_pose.position.y  #r_x and r_y
            rotation = cozmo_pose.rotation
            cozmo_theta = rotation.angle_z.radians
            ob_x, ob_y, ob_theta = obstacle_1.pose.position.x, obstacle_1.pose.position.y, obstacle_1.pose.rotation.angle_z.radians
            ob_vect = [ob_x, ob_y, 1]
            transformation_matrix = [[math.cos(-cozmo_theta), -1*math.sin(-cozmo_theta), translation_vect[0][0]],
                [math.sin(-cozmo_theta), math.cos(-cozmo_theta), translation_vect[1][0]],
                [0, 0, 1]]
        
            updated_ob_vect = [[0],[0],[0]]
            updated_ob_vect = np.matmul(np.array(transformation_matrix), np.array(ob_vect).reshape((3, 1)))

            nodeA = Node([updated_ob_vect[0][0] - OFFSET, updated_ob_vect[1][0] - OFFSET])
            nodeB = Node([updated_ob_vect[0][0] - OFFSET, updated_ob_vect[1][0] + OFFSET])
            nodeC = Node([updated_ob_vect[0][0] + OFFSET, updated_ob_vect[1][0] - OFFSET])
            nodeD = Node([updated_ob_vect[0][0] + OFFSET, updated_ob_vect[1][0] + OFFSET])
            nodes = [nodeB, nodeA, nodeC, nodeD]
            cmap.add_obstacle(nodes)
            ob1_found = True
            print("Obstacle 1 found")
            cmap.clear_solved()
            cmap.clear_node_paths()
            cmap.clear_nodes()
            cmap.set_start(Node(tuple([cur.x, cur.y])))
            RRT(cmap, cmap.get_start())

        if obstacle_2 is not None and ob2_found is False:
            cozmo_pose = robot.pose
            cozmo_x, cozmo_y = cozmo_pose.position.x, cozmo_pose.position.y  #r_x and r_y
            rotation = cozmo_pose.rotation
            cozmo_theta = rotation.angle_z.radians
            ob_x, ob_y, ob_theta = obstacle_2.pose.position.x, obstacle_2.pose.position.y, obstacle_2.pose.rotation.angle_z.radians
            ob_vect = [ob_x, ob_y, 1]
            transformation_matrix = [[math.cos(-cozmo_theta), -1*math.sin(-cozmo_theta), translation_vect[0][0]],
                [math.sin(-cozmo_theta), math.cos(-cozmo_theta), translation_vect[1][0]],
                [0, 0, 1]]
        
            updated_ob_vect = [[0],[0],[0]]
            updated_ob_vect = np.matmul(np.array(transformation_matrix), np.array(ob_vect).reshape((3, 1)))

            nodeA = Node([updated_ob_vect[0][0] - OFFSET, updated_ob_vect[1][0] - OFFSET])
            nodeB = Node([updated_ob_vect[0][0] - OFFSET, updated_ob_vect[1][0] + OFFSET])
            nodeC = Node([updated_ob_vect[0][0] + OFFSET, updated_ob_vect[1][0] - OFFSET])
            nodeD = Node([updated_ob_vect[0][0] + OFFSET, updated_ob_vect[1][0] + OFFSET])
            nodes = [nodeB, nodeA, nodeC, nodeD]
            cmap.add_obstacle(nodes)
            ob2_found = True
            print("Obstacle 2 found")
            cmap.clear_solved()
            cmap.clear_node_paths()
            cmap.clear_nodes()
            cmap.set_start(Node(tuple([cur.x, cur.y])))
            RRT(cmap, cmap.get_start())

################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/emptygrid.json", node_generator)
    robot_thread = RobotThread()
    robot_thread.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
