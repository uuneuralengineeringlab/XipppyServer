import numpy as np

def deka_pos_vel_map(SS):
    # get sensor values from wrist
    # 13 is rotation
    # 14 is flexion
    rotation = SS['cur_sensors'][13]
    flexion = -SS['cur_sensors'][14] #for deka, negative is flexion
    
    # normalize [-1, 1]
    if rotation >= 0:
        mapped_rot = rotation/11200 # max pronation
    else:
        mapped_rot = rotation/7680 # max supination
    
    if flexion >= 0:
        mapped_flex = flexion/3520 # max flexion 
    else:
        mapped_flex = flexion/3520 # max extension
    
    
    if SS['wrist_mode'] == 'Pos':
        # for active decode
        new_rotation = (SS['xhat'][5] - mapped_rot)*5
        new_rotation = np.clip(new_rotation, -1, 1)
        new_flex = (SS['xhat'][4] - mapped_flex)*2
        new_flex = np.clip(new_flex, -1, 1)
    else:
        # for active decode
        new_rotation = SS['xhat'][5]
        new_flex = SS['xhat'][4]
        
    # for active decode
    SS['xhat'][5] = new_rotation
    SS['xhat'][4] = new_flex

    # for mimicry training - kinematics are always position mode
    new_rotation = SS['kin'][5] - mapped_rot
    new_rotation = np.clip(new_rotation, -1, 1)
    new_flex = SS['kin'][4] - mapped_flex
    new_flex = np.clip(new_flex, -1, 1)
            
    
    # for mimicry training
    SS['kin'][5] = new_rotation
    SS['kin'][4] = new_flex
    
    return SS

