import rospy, time
import numpy as np
from std_msgs.msg import Float64

class TaskParser:
    def __init__(self, script_path):
        rospy.init_node("/task_parser", anonymous=True)
        self.pub = rospy.Publisher("/")
        self.rate = rospy.Rate(10)
        self.script_path = script_path
        self.configuration = np.zeros(6)
        
        # Raise error if given improper file
        if self.script_path[-5:] != ".prog":
            raise ValueError("File extension should be given in .prog .")
        
        # Open file
        self.script_io = open(script_path, "r")
    
    def execute_step(self):
        # Open file and read line-by-line
        self.script_io.seek(0)
        line_buf = ""
        while True:
            # Read line-by-line until reached EOF
            line_buf = self.script_io.readline()
            if line_buf == "":
                break
            
            # Print the command that is being processed
            rospy.loginfo("Command being processed: %s" % line_buf)
            
            if line_buf[:4] == "OPEN":
                # No additional argument is to be given.
                # Call function for command OPEN.
                print("Open gripper...")
            elif line_buf[:4] == "GRIP":
                # Given argument is the force by gripper.
                # Call function for command GRIP.
                print("Close gripper...")
            elif line_buf[:4] == "WAIT":
                # Given arugment is the time to sleep.
                # Parse the argument and wait for that time.
                line_buf = line_buf[line_buf.index('(')+1:line_buf.index(')')]
                print("Sleep for %f seconds..." % float(line_buf))
                time.sleep(float(line_buf))
            elif line_buf[:4] == "MOVE":
                # Given argument is the desired configuration of the robot.
                # Call the service that would move the robot.
                line_buf = line_buf[line_buf.index('(')+1:line_buf.index(')')]
                configuration_buf = [float(x) for x in line_buf.replace(' ','').split(',')]
                self.configuration = np.array(configuration_buf)
                print("Move to ", self.configuration, "...")
        
        rospy.loginfo("Your requested task is finished!")

    def execute_step(self):
        # Open file and read line-by-line
        self.script_io.seek(0)
        line_buf = ""
        while True:
            # Read line-by-line until reached EOF
            line_buf = self.script_io.readline()
            if line_buf == "":
                break
            
            # Print the command that is being processed
            rospy.loginfo("Command being processed: %s" % line_buf)
            
            if line_buf[:4] == "OPEN":
                # No additional argument is to be given.
                # Call function for command OPEN.
                pass
            elif line_buf[:4] == "GRIP":
                # Given argument is the force by gripper.
                # Call function for command GRIP.
                pass
            elif line_buf[:4] == "WAIT":
                # Given arugment is the time to sleep.
                # Parse the argument and wait for that time.
                line_buf = line_buf[line_buf.index('(')+1:line_buf.index(')')]
                time.sleep(float(line_buf))
            elif line_buf[:4] == "MOVE":
                # Given argument is the desired configuration of the robot.
                # Call the service that would move the robot.
                line_buf = line_buf[line_buf.index('(')+1:line_buf.index(')')]
                configuration_buf = [float(x) for x in line_buf.replace(' ','').split(',')]
                self.configuration = np.array(configuration_buf)
                pass
        
        rospy.loginfo("Your requested task is finished!")

if __name__ == "__main__":
    print("Select operating mode.")
    print("a: Run all at once")
    print("s: Step single line")
    option = input(">>> ")
    
    if option == "a":
        execute_step