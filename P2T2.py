from interface.py import iRobot

if __name__ == "__main__":
    robot = iRobot() # A
    robot.safe()
    robot.start()
    while True:
        if not robot.MOVING and robot.safe_status() and (robot.clean_pressed or robot.power_pressed): # B
            robot.drive_straight(float('inf'))
            if robot.LWD or robot.RWD:
                break
            