package kyberlib.mechanisms.drivetrain.swerve

import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.feetPerSecond
import kyberlib.sensors.gyros.KGyro

/**
 * TODO:
 * test everything
 * test Field oriented
 * test breaks
 * tune closed loop
 */

class SwerveDrive(private val gyro: KGyro,
                  private vararg val swerveModules: SwerveModule,
                  val constraints: TrapezoidProfile.Constraints = TrapezoidProfile.Constraints(10.feetPerSecond.value, 10.feetPerSecond.value)
                        ) : SubsystemBase() {
    // controls
    private val kinematics = SwerveDriveKinematics(*swerveModules.map { it.location }.toTypedArray())
    private val odometry = SwerveDriveOdometry(kinematics, 0.degrees)

    // field relative settings
    private var fieldHeading = gyro.heading  // TODO: what is the 0 absolute heading

    var states
        get() = swerveModules.map { it.state }
        set(value) {swerveModules.zip(value).forEach { it.first.state = it.second }}

    val speed: ChassisSpeeds
        get() =  kinematics.toChassisSpeeds(*states.toTypedArray())

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

    /**
     * Individually sets the states for each module
     */
    fun drive(vararg states: SwerveModuleState) {
        assert(states.size == swerveModules.size) { "The size of states(${states.size}) do no match the number of modules ${swerveModules.size}" }
        SwerveDriveKinematics.normalizeWheelSpeeds(states, constraints.maxVelocity)
        swerveModules.zip(states).forEach { it.first.state = it.second }
    }

    /**
     * Locks the wheels to prevent being pushed
     */
    fun lock() {
        drive(*swerveModules.map { it.breakState }.toTypedArray())
    }

    /**
     * Called periodically.
     * Updates the robot location with each of the modules states
     */
    override fun periodic() {
        val moduleStates = swerveModules.map { it.state }
        odometry.update(gyro.heading, *moduleStates.toTypedArray())
    }

    val xControl = PIDController(0.7, 0.0, 0.1)
    val yControl = PIDController(0.7, 0.0, 0.1)
    val thetaControl = ProfiledPIDController(0.7, 0.0, 0.1, constraints)

    /**
     * generates a command to follow a trajectory
     */
    fun auto(trajectory: Trajectory): SwerveControllerCommand {
        return SwerveControllerCommand(
            trajectory,
            odometry::getPoseMeters,
            kinematics,
            xControl, yControl, thetaControl,
            this::drive,
            this
        )
    }

}