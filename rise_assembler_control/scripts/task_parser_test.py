import time

script = ""
script_io = open("../programs/assembly_naive.prog")

line_buf = script_io.readline()
script_io.seek(0)
while True:
    # Read line-by-line until reached EOF
    line_buf = script_io.readline()
    line_buf = line_buf[:-1]
    if line_buf == "":
        print "Command finished!"
        break
    
    # Print the command that is being processed
    print "Command being processed: %s" % line_buf
    
    if line_buf[:4] == "OPEN":
        # No additional argument is to be given.
        # Call function for command OPEN.
        print "Open gripper..."
    elif line_buf[:4] == "GRIP":
        # Given argument is the force by gripper.
        # Call function for command GRIP.
        print "Close gripper..."
    elif line_buf[:4] == "WAIT":
        # Given arugment is the time to sleep.
        # Parse the argument and wait for that time.
        line_buf = line_buf[line_buf.index('(')+1:line_buf.index(')')]
        print "Sleep for %f seconds..." % float(line_buf)
        time.sleep(float(line_buf))
    elif line_buf[:4] == "MOVE":
        # Given argument is the desired configuration of the robot.
        # Call the service that would move the robot.
        line_buf = line_buf[line_buf.index('(')+1:line_buf.index(')')]
        configuration = [float(x) for x in line_buf.replace(' ','').split(',')]
        print "Move to ", configuration, "..."
