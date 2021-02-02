# Provides a python API, manual testing, and demo ability for the LUKE(DEKA) hand.
# Calls wrapped C++ driver in low_cost_lkdriver.cpp
#
# Aidan Lethaby
# 30 June 2020


import lk_module # Import our module so we can use it
import array # Lets us use arrays
import time # Lets us use sleep (delay)


############################## Helper Functions ##############################

# Convert commands from standard -1 to 1 range to lk hand specified range
# Takes a list of control commands from -1 to 1
# Will clamp any values outside of this range
# Returns converted list of commands of the following range:
#    {-1023 to 1023 for wrist_roll,
#     -1023 to 1023 for wrist_pitch,
#     0 to 1023 for thumb_flex,
#     -1023 to -512 for thumb_abd,
#     0 to 1023 for index_flex,
#     0 to 1023 for mrp_flex}
def convert_command(command_list_convert):
    # Clamp values outside range
    index = 0
    for command in command_list_convert:
        if command > 1:
            command_list_convert[index] = 1
        elif command < -1:
            command_list_convert[index] = -1
        index += 1
        
    proportionalize_wrist_commands(command_list_convert)  # Makes the wrist movement proportional to its distance from the desired position
    command_list_convert[0] = command_list_convert[0] * 1023  # Convert wrist_roll
    command_list_convert[1] = command_list_convert[1] * 1023  # Convert wrist_pitch
    command_list_convert[2] = command_list_convert[2] * 511.5 + 511.5  # Convert thumb_flex
    command_list_convert[3] = command_list_convert[3] * 255.5 - 767.5  # Convert thumb_abd
    command_list_convert[4] = command_list_convert[4] * 511.5 + 511.5  # Convert index_flex
    command_list_convert[5] = command_list_convert[5] * 511.5 + 511.5  # Convert mrp_flex
    return command_list_convert


# Makes the wrist movement speed proportional to its distance from the desired position
# Takes a list of control commands from -1 to 1
# Fleshed out for clarity
# Returns list of commands with proportionate wrist speed
def proportionalize_wrist_commands(command_list_wrist):
    sensor_values = lk_get_sensor()
    rotation_difference = round(command_list_wrist[0] - (sensor_values[0] + 28) / 145, 4)  # Sensor value scaled to be -1 to 1 (Sensor range is originally -173 to 117)
    deviation_difference = round(command_list_wrist[1] - sensor_values[1] / 53, 4) # Sensor value scaled to be -1 to 1 (Sensor range is originally -53 to 53)
    
    # Clamp values outside range as max speed is -1 or 1
    if rotation_difference > 1:
        rotation_difference = 1
    elif rotation_difference < -1:
        rotation_difference = -1
    if deviation_difference > 1:
        deviation_difference = 1
    elif deviation_difference < -1:
        deviation_difference = -1
    
    # Note a threshold is used as motors are not precise enough to achieve exact position
    if rotation_difference > 0.08:
        command_list_wrist[0] = rotation_difference * -1.0  # More positive position is reached via a negative/clockwise speed
    elif rotation_difference < -0.08:
        command_list_wrist[0] = rotation_difference * -1.0  # More negative position is reached via a positive/counter-clockwise speed
    else:
        command_list_wrist[0] = 0  # At desired position, stop movement
    if deviation_difference > 0.08:
        command_list_wrist[1] = deviation_difference  # More positive position is reached via a positive speed
    elif deviation_difference < -0.08:
        command_list_wrist[1] = deviation_difference  # More negative position is reached via a negative speed
    else:
        command_list_wrist[1] = 0  # At desired position, stop movement
    return command_list_wrist


# Set the commands for controlling the hand directly by calling lk_modules.lk_set_command()
# Takes a list of direct control commands of the following range and order:
#    {-1023 to 1023 for wrist_roll,
#     -1023 to 1023 for wrist_pitch,
#     0 to 1023 for thumb_flex,
#     -1023 to -512 for thumb_abd,
#     0 to 1023 for index_flex,
#     0 to 1023 for mrp_flex}
# Will clamp any values outside of their given range
# Returns a list of the command values that were set (for verification/testing purposes)
def lk_set_command_direct(command_list_direct):
    # Clamp wrist_roll value
    if command_list_direct[0] > 1023:
        command_list_direct[0] = 1023
    if command_list_direct[0] < -1023:
        command_list_direct[0] = -1023
    # Clamp wrist_pitch value
    if command_list_direct[1] > 1023:
        command_list_direct[1] = 1023
    if command_list_direct[1] < -1023:
        command_list_direct[1] = -1023
    # Clamp thumb_flex value
    if command_list_direct[2] > 1023:
        command_list_direct[2] = 1023
    if command_list_direct[2] < 0:
        command_list_direct[2] = 0
    # Clamp thumb_abd value
    if command_list_direct[3] > -512:
        command_list_direct[3] = -512
    if command_list_direct[3] < -1023:
        command_list_direct[3] = -1023
    # Clamp index_flex value
    if command_list_direct[4] > 1023:
        command_list_direct[4] = 1023
    if command_list_direct[4] < 0:
        command_list_direct[4] = 0
    # Clamp mrp_flex value
    if command_list_direct[5] > 1023:
        command_list_direct[5] = 1023
    if command_list_direct[5] < 0:
        command_list_direct[5] = 0
    return lk_module.lk_set_command(array.array('d', command_list_direct))


# Performs LUKE demo where hand responds to certain sensor touches.
# Touch index tip sensor to cause actuation of index finger.
# Touch middle, ring, or pinky sensor to cause actuation of middle, ring, and pinky finger.
# Touch thumb tip sensor to cause actuation of thumb (yaw axis).
# Touch palm distal sensor to cause actuation of wrist (deviation).
# Apply pressure to hand dorsal sensor to exit LUKE demo.
def luke_demo():
    while True:
        sensor_values = lk_get_sensor()
        if sensor_values[17] >= 18: #Check hand dorsal sensor value
            lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
            return #Exit LUKE demo
        elif sensor_values[10] >= 1.5: #Check index tip sensor value
            lk_set_command([0, 0, -1, 1, 1, -1]) #Actuate index finger
            time.sleep(0.5)
            lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
            time.sleep(0.5)
        elif sensor_values[11] >= 2.0 or sensor_values[12] >= 2.0 or sensor_values[13] >= 2.0: #Check mrp sensor values
            lk_set_command([0, 0, -1, 1, -1, 1]) #Actuate mrp fingers
            time.sleep(0.5)
            lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
            time.sleep(0.5)
        elif sensor_values[20] >= 1.0: #Check thumb tip sensor value
            lk_set_command([0, 0, 1, 1, -1, -1]) #Actuate thumb
            time.sleep(0.5)
            lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
            time.sleep(0.5)
        elif sensor_values[14] >= 2.0: #Check palm distal sensor value
            lk_set_command([0, -1, -1, 1, -1, -1]) #Actuate wrist inward
            time.sleep(0.5)
            lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
            time.sleep(0.5)
    return


# Performs demo where hand displays all six degrees of freedom.
def demo():
    lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
    time.sleep(1.0)
    lk_set_command([0, 0, 1, 1, -1, -1]) #Actuate thumb (yaw)
    time.sleep(1.0)
    lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
    time.sleep(1.5)
    lk_set_command([0, 0, -1, -1, -1, -1]) #Actuate thumb (pitch)
    time.sleep(1.0)
    lk_set_command([0, 0, 1, -1, -1, -1]) #Actuate thumb (yaw)
    time.sleep(1.0)
    lk_set_command([0, 0, -1, -1, -1, -1]) #Undo thumb (yaw)
    time.sleep(1.0)
    lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
    time.sleep(1.5)
    lk_set_command([0, 0, -1, 1, 1, -1]) #Actuate index finger
    time.sleep(1.0)
    lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
    time.sleep(1.5)
    lk_set_command([0, 0, -1, 1, -1, 1]) #Actuate mrp fingers
    time.sleep(1.0)
    lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
    time.sleep(1.5)
    lk_set_command([0, -1, -1, 1, -1, -1]) #Actuate wrist inward
    time.sleep(0.5)
    lk_set_command([0, 1, -1, 1, -1, -1]) #Actuate wrist outward
    time.sleep(0.5)
    lk_set_command([0, 0, -1, 1, -1, -1]) #Actuate wrist inward to nuetral position
    time.sleep(0.5)
    lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
    time.sleep(0.5)
    lk_set_command([-1, 0, -1, 1, -1, -1]) #Rotate wrist clockwise
    time.sleep(0.5)
    lk_set_command([1, 0, -1, 1, -1, -1]) #Rotate wrist counter-clockwise
    time.sleep(0.5)
    lk_set_command([0, 0, -1, 1, -1, -1]) #Rotate wrist clockwise to nuetral position
    time.sleep(0.5)
    lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
    time.sleep(0.5)
    return


############################## API ##############################

# Starts the thread for controlling the hand by calling lk_modules.lk_start()
# Returns an informative string regarding the outcome of the start call
def lk_start():
    if lk_module.lk_start() == 0:
        return "Thread successfully started."
    else:
        return "Failed, thread already running."


# Stops the thread for controlling the hand by calling lk_modules.lk_stop()
# Returns an informative string regarding the outcome of the start call
def lk_stop():
    if lk_module.lk_stop() == 0:
        return "Thread successfully stopped."
    else:
        return "Failed, no thread stopped."


# Check if the thread for controlling the hand is active by calling lk_modules.lk_running()
# Returns an informative string regarding the outcome of the start call
def lk_running():
    if lk_module.lk_running() == 0:
        return "Thread NOT running."
    else:
        return "Thread running."


# Gets the sensor data from the hand by calling lk_modules.lk_get_sensor()
# Returns a list of the 22 sensor values in the order:
#    {wrist_rotation, wrist_deviation, thumb_yaw, thumb_pitch, index, mrp,
#     compliance_thumb_roll, compliance_thumb_pitch, compliance_index,
#     touch_index_lateral, touch_index_tip, touch_middle, touch_ring, touch_pinky,
#     palm_distal, palm_proximal, hand_edge, hand_dorsal,
#     thumb_ulnar, thumb_radial, thumb_tip, thumb_dorsal}
def lk_get_sensor():
    return lk_module.lk_get_sensor()


# Set the commands for controlling the hand indirectly given a standard range from -1 to 1 for each degree of freedom
# Takes a list of control commands of the order:
#    {wrist_roll, wrist_pitch, thumb_flex, thumb_abd, index_flex, mrp_flex}
def lk_set_command(command_list):
    direct_command_list = convert_command(command_list.copy())
    lk_set_command_direct(direct_command_list)
    
    # While the wrist is still in motion (rotation or deviation)
    while direct_command_list[0] != 0 or direct_command_list[1] != 0:
        direct_command_list = convert_command(command_list.copy())
        lk_set_command_direct(direct_command_list) 
    return


############################## Terminal Prompt for Testing With Manual Inputs ##############################

# Runs when file is called from the terminal
if __name__ == '__main__':
    print("\nlow_cost_lkdriver tester portion running...")
    print("\nCommands:")
    print("start -> starts the thread that controls the hand")
    print("stop -> stops the thread that controls the hand and terminates testing")
    print("running -> checks if the thread that controls the hand is running")
    print("sensor -> returns the sensor data of the hand")
    print("command -> request a list of values then send them to the hand to be executed")
    print("luke -> enters luke demo, hand responds to certain sensor touches")
    print("demo -> enters demo, displays hand's ranges of movement\n")

    while True:
        command = input("\nType a command: ")

        if command == "start":
            print(lk_start())  # run lk_start and print return value
#             lk_set_command([0, 0, -1, 1, -1, -1]) #Stop reset hand to original position
        elif command == "stop":
            print(lk_stop())  # run lk_stop and print return value
            break
        elif command == "running":
            print(lk_running())  # run lk_running and print return value
        elif command == "sensor":
            sensor_list = lk_get_sensor()  # run lk_get_sensor and print sensor data
            print("Sensor Values:\nwrist_rotation: " + str(sensor_list[0]))
            print("wrist_deviation: " + str(sensor_list[1]) + "\nthumb_yaw: " + str(sensor_list[2]) + "\nthumb_pitch: " + str(sensor_list[3]))
            print("index: " + str(sensor_list[4]) + "\nmrp: " + str(sensor_list[5]) + "\ncompliance_thumb_roll: " + str(sensor_list[6]))
            print("compliance_thumb_pitch: " + str(sensor_list[7]) + "\ncompliance_index: " + str(sensor_list[8]) + "\ntouch_index_lateral: " + str(sensor_list[9]))
            print("touch_index_tip: " + str(sensor_list[10]) + "\ntouch_middle: " + str(sensor_list[11]) + "\ntouch_ring: " + str(sensor_list[12]))
            print("touch_pinky: " + str(sensor_list[13]) + "\npalm_distal: " + str(sensor_list[14]) + "\npalm_proximal: " + str(sensor_list[15]))
            print("hand_edge: " + str(sensor_list[16]) + "\nhand_dorsal: " + str(sensor_list[17]) + "\nthumb_ulnar: " + str(sensor_list[18]))
            print("thumb_radial: " + str(sensor_list[19]) + "\nthumb_tip: " + str(sensor_list[20]) + "\nthumb_dorsal: " + str(sensor_list[21]))
        elif command == "command":
            commands = []
            print("\nEach degree of freedom can take a value -1 to 1, do not add any leading or trailing spaces.\n")
            commands.append(float(input("Wrist rotation (- is counter-clockwise, + is clockwise, with palm up 0 is ~12 o'clock if position control): ")))
            commands.append(float(input("Wrist deviation (- is inward, + is outward): ")))
            commands.append(float(input("Thumb yaw (- is outward, + is inward): ")))
            commands.append(float(input("Thumb pitch (- is inward, + is outward): ")))
            commands.append(float(input("Index (- is outward, + is inward): ")))
            commands.append(float(input("MRP (- is outward, + is inward): ")))
            lk_set_command(commands)  # run lk_set_command and print command data
        elif command == "luke":
            print("\nEntering LUKE demo.\n")
            print("Touch index tip sensor to cause actuation of index finger.")
            print("Touch middle, ring, or pinky sensor to cause actuation of middle, ring, and pinky finger.")
            print("Touch thumb tip sensor to cause actuation of thumb (yaw axis).")
            print("Touch palm distal sensor to cause actuation of wrist (deviation).")
            print("Apply pressure to hand dorsal sensor to exit LUKE demo.")
            luke_demo();
            print("\nExiting LUKE demo.\n")
        elif command == "demo":
            print("\nEntering demo.\n")
            demo();
            print("\nExiting demo.\n")
        else:
            print("Unknown command! (check spelling and whitespace)")

        print("")  # newline for formatting

    print("End testing")
