package frc.team6502.robot.auto

import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpiutil.math.numbers.*
import frc.team6502.robot.Constants
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.auto.Navigator
import kyberlib.sensors.gyros.KPigeon


/**
 * A Subsystem to manage and update the Robots position
 * @param initialPose the pose of the robot when Navigation begins
 * @author TateStaples
 */
object Navigation : Navigator(KPigeon(Constants.PIGEON_PORT)){
    init {
        applyMovementRestrictions(Constants.velocity, Constants.acceleration)
        applyKinematics(Drivetrain.kinematics)
    }
}