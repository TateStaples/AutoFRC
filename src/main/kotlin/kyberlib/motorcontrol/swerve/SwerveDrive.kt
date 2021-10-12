package kyberlib.motorcontrol.swerve

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kyberlib.math.units.extensions.degrees
import kyberlib.sensors.gyros.KGyro

/**
 * TODO:
 * test everything
 * test Field oriented
 * test breaks
 * tune closed loop
 */

class SwerveDrive(val gyro: KGyro, vararg val swerveModules: SwerveModule) : SubsystemBase() {
    // controls
    private val kinematics = SwerveDriveKinematics(*swerveModules.map { it.location }.toTypedArray())
    private val odometry = SwerveDriveOdometry(kinematics, 0.degrees)

    // field relative settings
    private var fieldHeading = gyro.heading  // what is the 0 absolute heading

    /**
     * Drive the robot is given directions
     * @param speeds The speeds to move the robot
     * @param fieldOriented whether to drive relative to the driver or relative to the robot
     */
    fun drive(speeds: ChassisSpeeds, fieldOriented: Boolean = true) {
        if (fieldOriented) {
            val fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, gyro.heading.minus(fieldHeading))
            drive(*kinematics.toSwerveModuleStates(fieldSpeeds))
        }
        else drive(speeds)
    }
    fun drive(vararg states: SwerveModuleState) {
        assert(states.size == swerveModules.size) { "The size of states(${states.size}) do no match the number of modules ${swerveModules.size}" }
        swerveModules.zip(states).forEach { it.first.state = it.second }
    }

    /**
     * Locks the wheels to prevent being pushed
     */
    fun lock() {
        drive(*swerveModules.map { it.breakState }.toTypedArray())
    }

    override fun periodic() {
        val moduleStates = swerveModules.map { it.state }
        odometry.update(gyro.heading, *moduleStates.toTypedArray())
    }

}