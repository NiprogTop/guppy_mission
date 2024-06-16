#!/usr/bin/env python3

import json
import os
from pathlib import Path

from math import pi, copysign, atan2, sin, cos, radians, degrees

import rospy
import smach

from std_srvs.srv import SetBool
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

PATH = os.path.dirname(__file__)
FILE_PATH = "../missions"
FILE_NAME = "/mision.json"


class Controller:
    enabled_ = False
    depth_ = 0.0
    heading_ = 0.0
    start_heading_ = 0
    depth_eps_ = 0.1
    heading_eps_ = 2.5
    surge_effort = 0.0
    mission_publisher_ = rospy.Publisher('mission_data', String, queue_size=1)

    def __init__(self):
        self.rate = rospy.Rate(10)

        rospy.Subscriber('depth', Float64, self.depth_callback)
        rospy.Subscriber('heading', Float64, self.heading_callback)
        rospy.Subscriber('sm_msg', String, self.msg_save)
        rospy.Subscriber('sm_msg_signal', Int16, self.msg_signal)
        self.start_service_ = rospy.Service('~start', Empty, self.handle_start)

        rospy.loginfo('State machine waiting sevices')
        rospy.wait_for_service('/pid/switch')
        self.pid_enable_service = rospy.ServiceProxy('/pid/switch', SetBool)

        self.command_publisher_ = rospy.Publisher('teleop_command', Twist, queue_size=1)
        self.depth_sp_publisher = rospy.Publisher('pid/depth_pid/setpoint', Float64, queue_size=1)
        self.heading_sp_publisher = rospy.Publisher('pid/heading_pid/setpoint', Float64, queue_size=1)
        

        self.state_change_time = rospy.Time.now()
        rospy.loginfo('State machine started')


    def handle_start(self, req):
        self.enabled_ = True
        return EmptyResponse()
    

    def submerge(self, depth):
        self.depth_sp_publisher.publish(depth)


    def emerge(self):
        self.depth_sp_publisher.publish(0)


    def switch_autopilot(self, en):
        self.pid_enable_service(en)
        rospy.loginfo(en)


    def depth_approached(self, d):
        if abs(d - self.depth_) < self.depth_eps_:
            return True
        else:
            return False


    def depth_callback(self, msg):
        self.depth_ = msg.data

    
    def heading_callback(self, msg):
        self.heading_ = msg.data


    def heading_on_start(self):
        self.start_heading_ = self.heading_


    def move(self, x, y, z):
        cmd = Twist()
        cmd.linear.x = x
        cmd.linear.y = y
        cmd.linear.z = z
        self.command_publisher_.publish(cmd)


    def turn_angle(self, heading):
        h = heading + self.start_heading_
        if (h < 0):
            h = 360 + h
        self.heading_sp_publisher.publish(h)

    
    def turn_approached(self, ang):
        ang_end = (self.start_heading_ + ang) % 360
        if (int(self.heading_) == int(ang_end)):
            return True
        else:
            return False


    def turn_time(self, y, z):
        cmd = Twist()
        cmd.angular.y = y
        cmd.angular.z = z
        self.command_publisher_.publish(cmd)

    
    def msg_signal(self, msg):
        self.mission_pub()

    
    def msg_save(self, msg):
        os.chdir(PATH)
        os.chdir(FILE_PATH)
        MISSION_PATH = os.getcwd()
        dd = json.loads(msg.data)
        with open(MISSION_PATH + FILE_NAME, 'w') as outfile:
            json.dump(dd, outfile)
            rospy.loginfo("Mission saved!")

    
    def mission_pub(self):
        os.chdir(PATH)
        os.chdir(FILE_PATH)
        MISSION_PATH = os.getcwd()
        self.blocks_sequence = []
        with open(MISSION_PATH + FILE_NAME) as data_file:
            data = data_file.read()
            data = json.loads(data) 
        self.mission_publisher_.publish(str(data))



class Wait(smach.State):
    def __init__(self, c, pid_enabled=False):
        smach.State.__init__(self, outcomes=['out_1'])
        self.cntr = c
        self.pid_enabled = pid_enabled


    def execute(self, userdata):
        self.cntr.enabled_ = False
        self.cntr.switch_autopilot(self.pid_enabled)
        while not rospy.is_shutdown():
            if self.cntr.enabled_:
                return 'out_1'
            else:
                self.cntr.rate.sleep()


class Submerge(smach.State):
    def __init__(self, c, depth):
        smach.State.__init__(self, outcomes=['out_1'])
        self.cntr = c
        self.depth = depth


    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.switch_autopilot(True)
        self.cntr.submerge(self.depth)

        while not rospy.is_shutdown():
            if self.cntr.depth_approached(self.depth):
                return 'out_1'
            else:
                self.cntr.rate.sleep()


class Emerge(smach.State):
    def __init__(self, c):
        smach.State.__init__(self, outcomes=['out_1'])
        self.cntr = c


    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.switch_autopilot(True)
        self.cntr.emerge()
        while not rospy.is_shutdown():
            if self.cntr.depth_approached(0) or ((rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(10)):
                self.cntr.switch_autopilot(False)
                self.cntr.enabled_ = False
                return 'out_1'
            else:
                self.cntr.rate.sleep()


class Move(smach.State):
    def __init__(self, c, effort_x, effort_y, effort_z, time):
        smach.State.__init__(self, outcomes=['out_1'])
        self.cntr = c
        self.time = time
        self.effort_x = effort_x
        self.effort_y = effort_y
        self.effort_z = effort_z


    def execute(self, userdata):
        self.cntr.switch_autopilot(True)
        self.cntr.state_change_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(self.time):
                self.cntr.move(0.0, 0.0, 0.0)
                return 'out_1'
            else:
                self.cntr.move(self.effort_x, self.effort_y, self.effort_z)
                self.cntr.rate.sleep()


class Turning_by_time(smach.State):
    def __init__(self, c, effort_y, effort_z, time):
        smach.State.__init__(self, outcomes=['out_1'])
        self.cntr = c
        self.time = time
        self.effort_y = effort_y
        self.effort_z = effort_z


    def execute(self, userdata):
        self.cntr.switch_autopilot(True)
        self.cntr.state_change_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(self.time):
                self.cntr.turn(0.0, 0.0)
                return 'out_1'
            else:
                self.cntr.turn(self.effort_y, self.effort_z)
                self.cntr.rate.sleep()


class Turning(smach.State):
    def __init__(self, c, angle):
        smach.State.__init__(self, outcomes=['out_1'])
        self.cntr = c
        self.angle = angle


    def execute(self, userdata):
        self.cntr.state_change_time = rospy.Time.now()
        self.cntr.switch_autopilot(True)
        self.cntr.turn_angle(self.angle)

        rospy.loginfo( "targ. angle: " + str((self.cntr.start_heading_ + self.angle) % 360))

        while not rospy.is_shutdown():
            if self.cntr.turn_approached(self.angle):
                return 'out_1'
            else:
                self.cntr.rate.sleep()

            
class Waiting(smach.State):
    def __init__(self, c, time):
        smach.State.__init__(self, outcomes=['out_1'])
        self.cntr = c
        self.time = time


    def execute(self, userdata):
        self.cntr.switch_autopilot(True)
        self.cntr.state_change_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if (rospy.Time.now() - self.cntr.state_change_time) > rospy.Duration(self.time):
                return 'out_1'
            else:
                self.cntr.rate.sleep()


class Mission_contriol:
    def __init__(self):
        self.c = Controller()
        self.MISSION_STATUS = 0
        self.mission_data = {}
        self.sm = smach.StateMachine(outcomes=['out_1'])
        self.blocks_sequence = []

    
    def file_parser(self):
        os.chdir(PATH)
        os.chdir(FILE_PATH)
        MISSION_PATH = os.getcwd()
        self.blocks_sequence = []

        with open(MISSION_PATH + FILE_NAME) as data_file:
            data = data_file.read()
            data = json.loads(data)
            for block_name in data:
                self.blocks_sequence.append(block_name)
                for block_type in data[block_name].keys():
                    print(data[block_name][block_type])
        rospy.loginfo(self.blocks_sequence)
        return data
    
    
    def switch_function(self, c, type, data):
        if type == 'Submerge':
            return Submerge(c, depth=data['depth'])
        elif type == 'Turning':
            return Turning(c, angle=data['angle'])
            # return Turning(c, effort_y=data['axis_y'], effort_z=data['axis_z'], time=data['time'])
        elif type == 'Move':
            return Move(c, effort_x=data['axis_x'], effort_y=data['axis_y'], effort_z=data['axis_z'], time=data['time'])
        elif type == 'Waiting': 
            return Waiting(c, time=data['time'])
        elif type == 'Emerge':
            return Emerge(c)


    def generation(self):
        self.rewrite()
        turn_on_start = 0
        self.mission_data = self.file_parser()
        ind_lead = lambda a, b: a if len(self.blocks_sequence) > a else b

        for index, block_name in enumerate(self.mission_data):
                for block_type in self.mission_data[block_name].keys():
                    with self.sm:
                        rospy.loginfo(self.mission_data[block_name])
                        rospy.loginfo(ind_lead(1, None))
                        if (index) == len(self.blocks_sequence)-1:
                            smach.StateMachine.add(block_name,
                                                    self.switch_function(self.c, block_type ,self.mission_data[block_name][block_type]),
                                                    transitions={'out_1': None})
                        else:
                            smach.StateMachine.add(block_name,
                                                    self.switch_function(self.c, block_type ,self.mission_data[block_name][block_type]),
                                                    transitions={'out_1': self.blocks_sequence[index + 1]})
                    # print(self.mission_data[block_name][block_type])


    def rewrite(self):
        self.sm = None
        self.sm = smach.StateMachine(outcomes=['out_1'])
        

    def start(self):
        self.start_angle = self.c.heading_on_start() 
        rospy.loginfo(self.sm.get_initial_states())
        if self.sm.get_initial_states() == ['None']:
            rospy.loginfo(self.blocks_sequence[0])
            self.sm.set_initial_state(self.blocks_sequence[0])
            rospy.loginfo("#####  UPDATE  #####")
        self.sm.execute()


if __name__ == '__main__':
    rospy.init_node('state_machine')
    rospy.sleep(rospy.Duration(1))

    mis = Mission_contriol()

    while not rospy.is_shutdown():
        if mis.MISSION_STATUS == 1:
                rospy.loginfo("#### Generation! ####")
                mis.generation()
                mis.MISSION_STATUS = 2
                rospy.set_param('mis_status', mis.MISSION_STATUS)
        if mis.c.enabled_:
            rospy.loginfo(mis.MISSION_STATUS)
            if mis.MISSION_STATUS == 2:
                rospy.loginfo("#### Start! ####")
                mis.c.heading_sp_publisher.publish(mis.c.heading_)
                mis.MISSION_STATUS = 0
                rospy.set_param('mis_status', mis.MISSION_STATUS)
                mis.start()
                rospy.loginfo("Mission End!")
                rospy.set_param('control_type', "web")
            else:
                rospy.loginfo("#### Generation! ####")
                mis.generation()
                mis.MISSION_STATUS = 2
                rospy.set_param('mis_status', mis.MISSION_STATUS)
        
        mis.MISSION_STATUS = rospy.get_param('mis_status')
        rospy.sleep(rospy.Duration(1))
