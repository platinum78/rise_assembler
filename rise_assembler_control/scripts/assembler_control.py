import rospy, Queue
from rise_assembler_control.srv import *

class RISE_Assembler_Controller:
    def __init__(self, program_path):
        self.program_io = open(program_path)
        self.program_io.seek(0)
        self.task_queue = Queue.Queue()
        pass
    
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
                self.task_queue.put(["OPEN", None])
            elif line_buf[:4] == "GRIP":
                # Given argument is the force by gripper.
                # Call function for command GRIP.
                self.task_queue.put(["GRIP", None])
            elif line_buf[:4] == "WAIT":
                # Given arugment is the time to sleep.
                # Parse the argument and wait for that time.
                line_buf = line_buf[line_buf.index('(')+1:line_buf.index(')')]
                self.task_queue.put(["WAIT", float(line_buf)])
            elif line_buf[:4] == "MOVE":
                # Given argument is the desired configuration of the robot.
                # Call the service that would move the robot.
                line_buf = line_buf[line_buf.index('(')+1:line_buf.index(')')]
                configuration = [float(x) for x in line_buf.replace(' ','').split(',')]
                self.task_queue.put(["MOVE", configuration])
    
    def execute_all(self):
        while !self.task_queue.empty():
            task = self.task_queue.get_nowait()
            if task[0] == "OPEN":
                # Deliver open command to gripper control node
                pass
            elif task[0] == "GRIP":
                # Deliver grip command to gripper control node
                pass
            elif task[0] == "MOVE":
                # Call move service from IRB120_Controller node
                configuration = task[1]
                rospy.wait_for_service("/irb120_move")
                try:
                    irb120_move = rospy.ServiceProxy("/irb120_move", IRB120_move)
                    irb120_move(configuration)
                    rospy.loginfo("Movement to ", configuration, "complete.")
                except rospy.ServiceException, e:
                    rospy.logerr("Service call failed: %s" % e)
                pass
            elif task[0] == "WAIT":
                rospy.sleep(task[1])
