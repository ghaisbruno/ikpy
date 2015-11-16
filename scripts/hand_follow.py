import poppy_inverse_kinematics.creature as model_creature
import numpy as np
import poppy_inverse_kinematics.plot_utils as plt_utils
import poppy_inverse_kinematics.meta_model as meta_model

# Create creatures
right_arm = model_creature.creature("torso_right_arm")
left_arm = model_creature.creature("torso_left_arm")
torso = meta_model.MetaModel()
torso.add_model(right_arm)
torso.add_model(left_arm)

# Set right arm position
target_right = np.array([-0.2, -0.1, 0.1])
right_arm.target = target_right
right_arm.goto_target()

# Choose left arm target
target_left = right_arm.forward_kinematic() + np.array([0.23, 0, 0])
left_arm.target = target_left
left_arm.goto_target()

# Plot result
torso.plot_meta_model()