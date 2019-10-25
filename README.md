Methods:
--------
* iRobot()
    - Usage: Establishes a connection with the Roomba
    - Arguments: None 
    - Return: None
* start()
    - Usage: Sends the start command to the Roomba and starts a thread for data reading
    - Arguments: None
    - Return: None
* reset()
    - Usage: Sends the reset command to the Roomba
    - Arguments: None
    - Return: None
* stop()
    - Usage: Sends the stop command to the Roomba
    - Arguments: None
    - Return: None
* safe()
    - Usage: Puts the Roomba in safe mode
    - Arguments: None
    - Return: None
* full()
    - Usage: Puts the Roomba in full mode
    - Arguments: None
    - Return: None
* read_data()
    - Usage: Method used by a thread to constantly read and update sensor information
    - Arguments: None
    - Return: None
* unwrap()
    - Usage: Treats the return value of the read command as a circular list to align data
    - Arguments: v1 and v2, representing the 2 values wanted to be in the front of the list
    - Return: A list where v1 and v2 are the two first values or [] if it isn't possible to make v1 and v2 the first values
* parse_data()
    - Usage: Parses the data returned by the Roomba
    - Arguments: List that contains the raw, unpacked data
    - Return: None
* decodeWDAB()
    - Usage: Decodes the return of Packet ID 7 into valid information of wheel drops and bump sensors
    - Arguments: a 1-byte number
    - Return: None
* decodeB()
    - Usage: Decodes the return of Packet ID 18 into valid information of button presses
    - Arguments: A 1-byte number
    - Return: None
* drive()
    - Usage: A wrapper method for driving the Roomba
    - Arguments:
        * speed, a speed in meters per second that the roomba should attempt to travel. Defaults to one fifth of the max speed
        * radius, the radius in meters that the roomba should turn in. Defaults to straight
        * delay, a boolean depicting whether or not the Roomba should add a delay after the drive command. Defaults to false
    - Return: None
* drive_straight()
    - Usage: Drives the Roomba in a straight line
    - Arguments:
        * distance, a distance in meters that the Roomba should travel for.
        * speed, a speed in meters per second that the Roomba should move at. Defaults to one fifth the maximum speed.
    - Return: None. Raises Error "Button Pressed" if a button is pressed while moving
* turn()
    - Usage: Makes the Roomba turn in place
    - Arguments:
        * angle, an angle in degrees that the Roomba turn travel for.
        * speed, a speed in meters per second that the Roomba should move at. Defaults to one fifth the maximum speed.
    - Return: None. Raises Error "Button Pressed" if a button is pressed while moving
* stop_drive()
    - Usage: Stops the Roomba's movement
    - Arguments: None
    - Return: None
* drive_direct()
    - Usage: Drives the Roomba by setting a velocity to each of the Roomba's wheels
    - Arguments:
        * t, a time in seconds that the Roomba should travel for.
        * vl, a speed in meters per second that the Roomba's left wheel should turn at.
        * vr, a speed in meters per seconf that the Roomba's right wheel should turn at.
    - Return: None
* safe_to_drive()
    - Usage: Used to determine if a Roomba should be allowed to drive
    - Arguments: None
    - Return: Boolean that represents whether or not the Roomba can drive
* safe_to_turn()
    - Usage: Used to determine if a Roomba should be allowed to turn in place
    - Arguments: None
    - Return: Boolean that represents whether or not the Roomba can turn in place
* button_pressed()
    - Usage: Used to determine if a button is being pressed
    - Arguments: None
    - Return: Boolean that represents whether or not a button is being pressed