import numpy as np
import pybullet as p
import itertools
import wall

class Robot():
    """ 
    The class is the interface to a single robot
    """
    def __init__(self, init_pos, robot_id, dt):
        self.id = robot_id
        self.dt = dt
        self.pybullet_id = p.loadSDF("../models/robot.sdf")[0]
        self.joint_ids = list(range(p.getNumJoints(self.pybullet_id)))
        self.initial_position = init_pos
        self.reset()

        #################Start of My Fields######################################
        self.delta = 2 # robot's connection is valid within 2m distance
        self.t = 0 # timer 
        self.is_surround = False # flag to see if 
        self.construct_desired_coor_dict() 
        self.in_shape = False # check if the formation is roughly in shape, switch off potential field for faster convergence.
        #################End of My Fields########################################

        # No friction between bbody and surface.
        p.changeDynamics(self.pybullet_id, -1, lateralFriction=5., rollingFriction=0.)

        # Friction between joint links and surface.
        for i in range(p.getNumJoints(self.pybullet_id)):
            p.changeDynamics(self.pybullet_id, i, lateralFriction=5., rollingFriction=0.)
            
        self.messages_received = []
        self.messages_to_send = []
        self.neighbors = []
    

    #################Start of My Methods######################################
    def construct_desired_coor_dict(self):
        # construct a dictionary that holds all the desired coordinates for different formations
        self.desired_coor_dict = dict()
        self.desired_coor_dict['square'] = np.array([[-0.5, -1], [-0.5, 0], 
                                                     [0, -1], [0, 0], 
                                                     [0.5, -1], [0.5, 0]])
        self.desired_coor_dict['diamond'] = np.array([[0, 0], [2/3, 0], [4/3, 0], [2, 0], [1, 1], [1, -1]])
        
        return False

    def formation_controller(self, formation, curr_id, curr_pos, messages):
        '''
        formation - string. Specify the formation
        curr_id - int. id of this robot
        curr_pos - np.array. 2-element array of the coordinates of this robot
        messages - np.array of np.array. array of received message

        Compute the formation control law of this specific robot, for this specific formation 
        '''
        desired_coor = self.desired_coor_dict[formation]
        dx, dy = 0, 0
        lim = 50
        for message in messages:
            neighbor_id = message[0]
            neighbor_pos = message[1]
            desired_distance = desired_coor[curr_id] - desired_coor[neighbor_id]

            # use edge function to update x and y coordiantes independently
            dx += self.edge_tension_dynamics(desired_distance[0], curr_pos[0], neighbor_pos[0])
            dy += self.edge_tension_dynamics(desired_distance[1], curr_pos[1], neighbor_pos[1])

            if formation == 'diamond' and not self.in_shape:
                dx_hat, dy_hat = self.potential_field(curr_pos, neighbor_pos, d_0 = 1, alpha=10, rejection=False)
                dx += dx_hat
                dy += dy_hat
                lim = 30
    
        
        dx = np.clip(dx, -lim, lim)
        dy = np.clip(dy, -lim, lim)
        return dx, dy

    def edge_tension_dynamics(self, d_ij, x_i, x_j):
        # implement the dynamics of the weighted graph-based feedback
        l_ij = x_i - x_j
        numerator = 2*(self.delta - np.abs(d_ij)) - np.abs(l_ij - d_ij)
        denominator = np.square(self.delta - np.abs(d_ij) - np.abs(l_ij-d_ij))

        return -(numerator/denominator) * (x_i-x_j-d_ij)
    
    def potential_field(self, r_i, r_j, d_0=1.5, alpha=5, rejection=False):
        # potential field for an object

        r_ij = np.linalg.norm(r_i - r_j)

        u = alpha*(1/r_ij - d_0/np.square(r_ij))
        u = np.max([u, -100])
        if rejection:
            u = np.min([u, 0])
        
        l_x, l_y, _ = r_i - r_j
        dx_hat = u * np.abs(l_x/r_ij) * 1 if -l_x >= 0 else u * np.abs(l_x/r_ij) * -1
        dy_hat = u * np.abs(l_y/r_ij) * 1 if -l_y >= 0 else u * np.abs(l_y/r_ij) * -1

        return dx_hat, dy_hat

    def follow_leader(self, leader_pos, curr_pos, messages, distance):
        # set a virtual leader and let the robots follow it

        # set a virtual leader
        dx, dy = self.potential_field(curr_pos, leader_pos, d_0=0.4, alpha=30, rejection=False)
        dx_hat, dy_hat = wall.room_potential(curr_pos[0], curr_pos[1])
        dx += 8*dx_hat
        dy += 8*dy_hat

        for message in messages:
            neighbor_pos = message[1]
            dx_hat, dy_hat = self.potential_field(curr_pos, neighbor_pos, d_0=distance, alpha=10, rejection=False)
            dx += dx_hat
            dy += dy_hat
        
        lim = 40
        dx = np.clip(dx, -lim, lim)
        dy = np.clip(dy, -lim, lim)

        return dx, dy
    
    def surround_ball(self, ball_pos, curr_pos, obstacle_pos, messages, radius):
        # make a circle around the designated location. Mainly used to surround ball.

        dx, dy = self.potential_field(curr_pos, ball_pos, d_0=0.55, alpha=60, rejection=False)
        dx_hat, dy_hat = wall.room_potential(curr_pos[0], curr_pos[1])
        dx += 8*dx_hat
        dy += 8*dy_hat

        if obstacle_pos:
            _dx, _dy = self.potential_field(curr_pos, obstacle_pos, d_0=1, alpha=30, rejection=True)
            dx_hat, dy_hat = wall.room_potential(curr_pos[0], curr_pos[1])
            _dx += dx_hat
            _dy += dy_hat

            dx += _dx
            dy += _dy

        for message in messages:
            neighbor_pos = message[1]
            dx_hat, dy_hat = self.potential_field(curr_pos, neighbor_pos, d_0=radius, alpha=10, rejection=False)
            dx += dx_hat
            dy += dy_hat
        
        lim = 40
        dx = np.clip(dx, -lim, lim)
        dy = np.clip(dy, -lim, lim)

        return dx, dy

    def get_formation_pos(self, curr_id, curr_pos, messages):
        # get the current formation coordinate info. Used to maintain surrounding a ball
        desired_pos = np.zeros((6,3))
        desired_pos[curr_id] = np.copy(curr_pos)
        for message in messages:
            neighbor_id = message[0]
            neighbor_pos = message[1]
            desired_pos[neighbor_id] = np.copy(neighbor_pos)

        return desired_pos
      
    def push_ball(self, curr_id, curr_pos, desired_pos, target_pos, messages):
        # push ball
        temp_pos = np.zeros((6,3))
        temp_pos[curr_id] = np.copy(curr_pos)

        dx = 0
        dy = 0

        for message in messages:
            neighbor_id = message[0]
            neighbor_pos = message[1]

            temp_pos[neighbor_id] = np.copy(neighbor_pos)

            desired_distance = desired_pos[curr_id] - desired_pos[neighbor_id]

            # use edge function to update x and y coordiantes independently
            dx += self.edge_tension_dynamics(desired_distance[0], curr_pos[0], neighbor_pos[0])
            dy += self.edge_tension_dynamics(desired_distance[1], curr_pos[1], neighbor_pos[1])
            dx_hat, dy_hat = self.potential_field(curr_pos, neighbor_pos, d_0 = 0.8, alpha=5, rejection=False)
            dx += dx_hat
            dy += dy_hat


        center_pos = np.mean(temp_pos, axis=0)
        
        fx = 3 * (target_pos[0] - center_pos[0])
        fy = 3 * (target_pos[1] - center_pos[1])

        dx += fx
        dy += fy
        dx_hat, dy_hat = wall.room_potential(curr_pos[0], curr_pos[1])
        dx += 8*dx_hat
        dy += 8*dy_hat

        return dx, dy
    #################End of My Methods#########################################



    def reset(self):
        """
        Moves the robot back to its initial position 
        """
        p.resetBasePositionAndOrientation(self.pybullet_id, self.initial_position, (0., 0., 0., 1.))
            
    def set_wheel_velocity(self, vel):
        """ 
        Sets the wheel velocity,expects an array containing two numbers (left and right wheel vel) 
        """
        assert len(vel) == 2, "Expect velocity to be array of size two"
        p.setJointMotorControlArray(self.pybullet_id, self.joint_ids, p.VELOCITY_CONTROL,
            targetVelocities=vel)

    def get_pos_and_orientation(self):
        """
        Returns the position and orientation (as Yaw angle) of the robot.
        """
        pos, rot = p.getBasePositionAndOrientation(self.pybullet_id)
        euler = p.getEulerFromQuaternion(rot)
        return np.array(pos), euler[2]
    
    def get_messages(self):
        """
        returns a list of received messages, each element of the list is a tuple (a,b)
        where a= id of the sending robot and b= message (can be any object, list, etc chosen by user)
        Note that the message will only be received if the robot is a neighbor (i.e. is close enough)
        """
        return self.messages_received
        
    def send_message(self, robot_id, message):
        """
        sends a message to robot with id number robot_id, the message can be any object, list, etc
        """
        self.messages_to_send.append([robot_id, message])
        
    def get_neighbors(self):
        """
        returns a list of neighbors (i.e. robots within 2m distance) to which messages can be sent
        """
        return self.neighbors
    
    
    
    def compute_controller(self):
        """ 
        function that will be called each control cycle which implements the control law
        TO BE MODIFIED
        
        we expect this function to read sensors (built-in functions from the class)
        and at the end to call set_wheel_velocity to set the appropriate velocity of the robots
        """
        
        # here we implement an example for a consensus algorithm
        neig = self.get_neighbors()
        messages = self.get_messages()
        pos, rot = self.get_pos_and_orientation()

        #self.trajectory.append(pos) # record the pos before we use the controller
        
        #send message of positions to all neighbors indicating our position
        for n in neig:
            self.send_message(n, pos)
        
        # check if we received the position of our neighbors and compute desired change in position
        # as a function of the neighbors (message is composed of [neighbors id, position])
        dx = 0.
        dy = 0.
        if messages:
            
            # form a square
            if self.t <= 2000:
                dx, dy = self.formation_controller('square', self.id, pos, messages)
            
            # get out of the room
            elif self.t > 2000 and self.t <= 7000:
                dx, dy = self.follow_leader([3, 3.5, 0.3], pos, messages, distance=1)
            
            # form a circle
            elif self.t > 7000 and self.t <= 9000:
                dx, dy = self.surround_ball([3, 3.5, 0.3], pos, None, messages, radius=0.9)
            
            # surround the purple ball
            elif self.t > 9000 and self.t <= 11000:
                dx, dy = self.surround_ball([2, 4, 0.3], pos, None, messages, radius=0.7)
            
            # push the purple ball
            elif self.t > 11000 and self.t <= 20000:
                if not self.is_surround:
                    self.desired_coor_dict['push'] = self.get_formation_pos(self.id, pos, messages)
                    self.is_surround = True
                elif self.is_surround:
                    dx, dy = self.push_ball(self.id, pos, self.desired_coor_dict['push'], [2.6, 5.6, 0.3], messages)
            
            # release the purple ball
            elif self.t > 20000 and self.t <= 22000:
                self.is_surround = False
                dx, dy = self.surround_ball([2.6, 5.6, 0.3], pos, None, messages, radius=1.5)
            
            # surround the red ball
            elif self.t > 22000 and self.t <= 26000:
                dx, dy = self.surround_ball([4, 2, 0.3], pos, [2.6, 5.6, 0.3], messages, radius=0.7)
            
            # push the red ball
            elif self.t > 26000 and self.t <= 36000:
                if not self.is_surround:
                    self.desired_coor_dict['push'] = self.get_formation_pos(self.id, pos, messages)
                    self.is_surround = True
                elif self.is_surround:
                    dx, dy = self.push_ball(self.id, pos, self.desired_coor_dict['push'], [0.4, 5.6, 0.3], messages)
            
            # release the red ball
            elif self.t >= 36000 and self.t <= 38000:
                self.is_surround = False
                dx, dy = self.surround_ball([0.3, 5.6, 0.3], pos, None, messages, radius=1.5)
            
            # get ready to re-enter the room
            elif self.t >= 38000 and self.t <= 41000:
                dx, dy = self.surround_ball([3, 3.5, 0.3], pos, [0.3, 5.6, 0.3], messages, radius=0.8)
            
            # re-enter the room
            elif self.t > 41000 and self.t <= 46000:
                dx, dy = self.follow_leader([1.72, 0, 0.3], pos, messages, distance=0.7)
            
            # form a diamond
            elif self.t > 46000:
                if self.t > 47000:
                    self.in_shape = True
                dx, dy = self.formation_controller('diamond', self.id, pos, messages)
            
            #compute velocity change for the wheels
            vel_norm = np.linalg.norm([dx, dy]) #norm of desired velocity
            if vel_norm < 0.01:
                vel_norm = 0.01
            des_theta = np.arctan2(dy/vel_norm, dx/vel_norm)
            right_wheel = np.sin(des_theta-rot)*vel_norm + np.cos(des_theta-rot)*vel_norm
            left_wheel = -np.sin(des_theta-rot)*vel_norm + np.cos(des_theta-rot)*vel_norm
            self.set_wheel_velocity([left_wheel, right_wheel])
            
            self.t += 1
        

    
       
