#!/usr/bin/python2

import rospy, Queue, sys
from rise_assembler_control.msg import *

class RISE_Assembler_Controller:
    def __init__(self, node_name, program_path):
        rospy.loginfo("Assembler controller initialized.")

        # Initialize controller instance
        rospy.init_node(node_name)
        self.program_io = open(program_path)
        self.program_io.seek(0)
        self.task_queue = Queue.Queue()

        # Create publishers that deliver movement commands
        self.irb120_control_publisher = rospy.Publisher("/assembler_move/irb120/command", IRB120_move, 100)
        self.gripper_control_publisher = rospy.Publisher("/assembler_move/gripper/command", gripper_move, 100)

        # Create and initialize messages
        self.irb120_msg = IRB120_move()
        self.gripper_msg = gripper_move()
        
    
    # Task parser: single step
    def parse_task(self):
        line_buf = ""
        task_count = 0
        while True:
            # Read line-by-line until reached EOF
            line_buf = self.program_io.readline()
            if line_buf == "":
                rospy.loginfo("Task parsing complete. %d tasks parsed." % task_count)
                break
            
            # Print the command that is being processed
            rospy.loginfo("Command being processed: %s" % line_buf)
            
            if line_buf[:4] == "OPEN":
                # No additional argument is to be given.
                # Call function for command OPEN.
                self.task_queue.put(["OPEN"])
                rospy.loginfo("Command [ OPEN ] pushed to task queue.")
            elif line_buf[:4] == "GRIP":
                # Given argument is the force by gripper.
                # Call function for command GRIP.
                self.task_queue.put(["GRIP"])
                rospy.loginfo("Command [ GRIP ] pushed to task queue.")
            elif line_buf[:4] == "WAIT":
                # Given arugment is the time to sleep.
                # Parse the argument and wait for that time.
                line_buf = line_buf[line_buf.index('(')+1:line_buf.index(')')]
                self.task_queue.put(["WAIT", float(line_buf)])
                rospy.loginfo("Command [ WAIT ] pushed to task queue.")
            elif line_buf[:4] == "MOVE":
                # Given argument is the desired configuration of the robot.
                # Call the service that would move the robot.
                config_buf = line_buf[line_buf.index('[')+1:line_buf.index(']')]
                configuration = [float(x) for x in config_buf.replace(' ','').split(',')]
                second_comma_pos = line_buf.index(',', line_buf.index(']')+1)
                travel_time = float(line_buf[second_comma_pos+1:line_buf.index(';')].strip())
                self.task_queue.put(["MOVE", configuration, travel_time])
                rospy.loginfo("Command [ MOVE ] pushed to task queue.")
    
    def execute_all(self):
        while not self.task_queue.empty():
            task = self.task_queue.get_nowait()
            if task[0] == "OPEN":
                # Deliver open command to gripper control node
                pass
            elif task[0] == "GRIP":
                # Deliver grip command to gripper control node
                pass
            elif task[0] == "MOVE":
                # Call move service from IRB120_Controller node
                self.irb120_msg.destination = task[1]
                self.irb120_msg.travel_time = task[2]
                self.irb120_msg.path_mode = IRB120_move.PATH_LINEAR
                self.irb120_control_publisher.publish(self.irb120_msg)
            elif task[0] == "WAIT":
                rospy.sleep(task[1])

if __name__ == "__main__":
    try:
        rospy.loginfo("assembler_control.py called.")
        assembler = RISE_Assembler_Controller("assembler_control_node", sys.argv[1])
        assembler.parse_task()
        assembler.execute_all()
    except rospy.ROSInterruptException:
        pass
        