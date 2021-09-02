package Utilities

/*
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.simulation.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj.system.plant.LinearSystemId
import edu.wpi.first.wpiutil.math.VecBuilder
import frc.team6502.kyberlib.math.units.extensions.inches


fun main() {
    Simulation()
}
// https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj/simulation/package-summary.html
class Simulation {
    // TODO: Add notes on how to alter network tables in shuffleboard and smartdashboard
    val field = Field2d()

    init {
//        SmartDashboard.putData("Field", field);
    }

    private val drivetrain = DifferentialDrivetrainSim( // Create a linear system from our characterization gains.
        LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
        DCMotor.getNEO(2),  // 2 NEO motors on each side of the drivetrain.
        7.29,  // 7.29:1 gearing reduction.
        0.7112,  // The track width is 0.7112 meters.
        3.inches.value,  // The robot uses 3" radius wheels.
        // The standard deviations for measurement noise:
        // x and y:          0.001 m
        // heading:          0.001 rad
        // l and r velocity: 0.1   m/s
        // l and r position: 0.005 m
        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
    )

    val rightEncoder = EncoderSim(Encoder(0, 0))
    val leftEncoder = EncoderSim(Encoder(1, 1))

    val gyro = AnalogGyroSim(4)

    val controller = XboxControllerSim(5)

    fun timestep(time: Double) {
        drivetrain.update(time)  // miliseconds
        leftEncoder.distance = 0.0 //drivetrain.leftPositionMeters TODO()
        leftEncoder.rate = TODO()
        rightEncoder.distance = 0.0 //drivetrain.rightPositionMeters TODO()
        rightEncoder.rate = TODO() //drivetrain.rightVelocityMetersPerSecond TODO()

        gyro.angle = -drivetrain.heading.degrees
    }

    fun timestep() {timestep(20.0)}


}

 */