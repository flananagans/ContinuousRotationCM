# ContinuousRotationCM
Github repository for the code used for the kinematics and analysis of a continuous rotation compliant mechanism

The important scripts that are new are *GenerateTrajectory.m* and *crcdPlaybackFullChain.m*. 
In *GenerateTrajectory.m*, the chain configuration for each crank angle phi is created over 3 steps:

1. Current rotor frame is moved onto the desired rotor frame
2. Chain is 'relaxed' based on cost at each joint. In the paper, the cost at each joint is set as the joint torque where all joints have the same stiffness (`kr` variable). Currently in the code, the joints are iteratively moved in the direction that most minimizes the total strain energy until it cannot move anymore without changing the rotor frame.
3. ~Chain is moved as close to the backbone curve as possible. This is basically the same as step 2 but the cost at each joint is based on how moving that joint will reduce the summed euclidean distances from each joint origin to the chain.~ We have stopped using this step

Running this entire script will output the joint angles produced for each phi angle. This is saved to a .mat file that can be loaded into the *crcdPlaybackFullChain.m* script and visualized

If you would like to try different numbers of joints, change the `NJ` variable in *crcdLoadT2.m* to your desired number of joints (should be odd). The height of the rotor is scaled linearly based on the numbers chosen in the paper.

# Prior work
This work is a follow-up to a previous conference paper by Matt Moses:

"Origami Rotors: Imparting Continuous Rotation to a Moving Platform Using Compliant Flexure Hinges", DETC2013-12753, by Matthew S. Moses, M. Kendal Ackerman, and Gregory S. Chirikjian. In Proceedings of the ASME 2013 International Design Engineering Technical Conferences & Computers and Information in Engineering Conference, IDETC/CIE 2013, August 4-August 7, 2013, Portland, OR, USA.

[Origami Rotors Github url](https://github.com/mattmoses/origami-rotors)
 
