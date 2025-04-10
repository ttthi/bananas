import math

def get_absolute_angles(rel_angles):
    '''Returns the absolute angles, which is the angle relative to the x-plan'''
    abs_angles = []
    running_sum = 0.0
    for i in range(len(rel_angles)):
        running_sum += rel_angles[i]
        abs_angles.append(running_sum)
    return abs_angles

def forward_kinematics(rel_angles, lengths, base_pos):
    '''Calculates the position of each joint'''

    abs_angles = get_absolute_angles(rel_angles)

    x, y = base_pos
    positions = []
    for i, ang in enumerate(abs_angles):
        x += lengths[i] * math.cos(ang)
        y += lengths[i] * math.sin(ang)
        positions.append((x, y))
    return positions

def ccd_inverse_kinematics(rel_angles, lengths, base_pos, target, max_iters, min_angles=None, max_angles=None, dst_treshold=0.1):
    '''Iteratively calculates the angle of the joints needed to reach the target position'''

    new_angles = rel_angles
    for _ in range(max_iters):

        # Check if target has been reached
        joint_positions = forward_kinematics(new_angles, lengths, base_pos)
        end_effector = joint_positions[-1]
        if math.dist(end_effector, target) < dst_treshold:
            break

        # Go through joints from the end to the base
        for i in reversed(range(len(new_angles))):
            if i == 0:
                pivot_pos = base_pos
            else:
                pivot_pos = joint_positions[i - 1]

            # Get current forward kinematics
            joint_positions = forward_kinematics(new_angles, lengths, base_pos)
            end_effector = joint_positions[-1]
            # Get vectors from pivot to end effector and target
            pivot_to_end = (end_effector[0] - pivot_pos[0], end_effector[1] - pivot_pos[1])
            pivot_to_target = (target[0] - pivot_pos[0], target[1] - pivot_pos[1])
            # Calculate the current arm and angle
            angle_end = math.atan2(pivot_to_end[1], pivot_to_end[0])
            angle_tgt = math.atan2(pivot_to_target[1], pivot_to_target[0])

            #
            abs_angles = get_absolute_angles(new_angles)
            old_abs_angle_i = abs_angles[i]
            new_abs_angle_i = old_abs_angle_i + angle_tgt - angle_end

            # Convert new absolute angle to relative
            if i == 0:
                # If it's the first joint, relative angle is just the absolute angle
                new_angles[i] = new_abs_angle_i
            else:
                new_abs_angle_prev = abs_angles[i - 1]
                new_angles[i] = new_abs_angle_i - new_abs_angle_prev

            # Optionally clamp to [min_angles[i], max_angles[i]]
            if min_angles is not None and max_angles is not None:
                new_angles[i] = max(min_angles[i], min(new_angles[i], max_angles[i]))

            # Recompute forward kinematics for next iteration
            joint_positions = forward_kinematics(new_angles, lengths, base_pos)

    return new_angles
