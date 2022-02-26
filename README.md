# ContinuousRotationCM
Github repository for the code used for the kinematics and analysis of a continuous rotation compliant mechanism (CRCM)

The code is built around the CRCM object. Look into *ExampleScript.m*, *PlaybackFullChain.m* and *SweepExample.m* for usage. In short, most interactions with the CRCM object are through these methods:

* `c = CRCM()` creates the chain object and constructs the link transformations for each joint. A constructor for non-default chain parameters is also provided.
* `c.updateTheta(theta)` updates the chain kinematics given a (`c.N` x 1) vector of joint angles.
* `c.generateTrajectory(phis)` generates joint angle trajectories that cause the rotor to follow the given crank angle `phi`

# Example Overviews

1. *ExampleScript.m*: Running this script will generate a chain trajectory for one full turn of the crank for a default chain. The trajectory and chain information is saved in a *.mat* file that can be loaded into *PlaybackFullChain.m* for visualization
2. *SweepExample.m*: Running this script will perform a sweep through different combinations of chain parameters. For each combination, the trajectory for a full crank revolution is computed and saved (along with the strain energy at each `phi`) is generated and saved in a *.mat* file.

# Support Chain Parameters
Various chain parameters can be modified:
* `N`: Number of links in the chain (ground to rotor, this is a *half chain* in Matt Moses' work)
* `C`: 1/2 the distance from the ground to the rotor. Because the link sizes are fixed, changing this effectively changes the ratio of link height (joint to joint length) to the height of the rotor.
* `alpha`: The skew angle of successive links (in radians). This is default to pi/2. Do note that plotting the chain for different alphas will look strange because the STL files will still have a skew angle of pi/2
* `k`: Joint stiffness. This can either be a scalar (all joints have the same stiffness) or a (`c.N` x 1) vector of individual stiffness of each joint

If you would like to try different parameters for the chain, YOU MUST DO THIS THROUGH THE CONSTRUCTOR. Changing the object's properties does not reconstruct the chain links.

# Trajectory Generation
In `c.generateTrajectory(phis)`, the chain configuration for each crank angle `phi` in `phis` is created over 3 steps:

1. Current rotor frame is moved onto the desired rotor frame (`c.moveChainToRotor`)
2. Chain is 'relaxed' based on cost at each joint (`c.relaxChain`). In Matt Moses' paper, the cost at each joint is the joint torque (stiffness * theta). In this work, the joints are iteratively moved in the direction that minimizes the total strain energy using gradient descent. The movements in this step are restricted to only those that do not change the rotor configuration (i.e. joint velocities are in the nullspace of the Jacobian).
3. In Matt Moses' work, the chain is moved as close to the desired backbone curve as possible (`c.moveChainToBackbone`). This is basically the same as step 2 but the cost at each joint is based on how moving that joint will reduce the summed euclidean distances from each joint origin to the chain. This is not used in this work.

# Prior work
This work is a follow-up to a previous conference paper by Matt Moses:

"Origami Rotors: Imparting Continuous Rotation to a Moving Platform Using Compliant Flexure Hinges", DETC2013-12753, by Matthew S. Moses, M. Kendal Ackerman, and Gregory S. Chirikjian. In Proceedings of the ASME 2013 International Design Engineering Technical Conferences & Computers and Information in Engineering Conference, IDETC/CIE 2013, August 4-August 7, 2013, Portland, OR, USA.

[Origami Rotors Github url](https://github.com/mattmoses/origami-rotors)
 
