def from_quaternion_to_euler(data) : 
    import numpy as np
    import tf_transformations
    roll = []
    pitch = []
    yaw = []
    or_x = data["qx"]
    or_y = data["qy"]
    or_z = data["qz"]
    or_w = data["qw"]

    for i in range(len(data)):
        qtr = np.array([
            or_x[i],
            or_y[i],
            or_z[i],
            or_w[i]
        ])
        roll_angle, pitch_angle, yaw_angle = tf_transformations.euler_from_quaternion(qtr)
        roll.append(roll_angle)
        pitch.append(pitch_angle)
        yaw.append(yaw_angle)
    return roll, pitch, yaw
