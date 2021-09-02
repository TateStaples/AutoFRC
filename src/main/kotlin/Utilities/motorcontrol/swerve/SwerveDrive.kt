/* sources for code
* https://github.com/Robostangs/SideStep/tree/master/src/org/usfirst/frc/team548/robot
* https://www.chiefdelphi.com/t/swerve-drive-programming/159349
 */
import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.kyberlib.input.KAxis
import frc.team6502.kyberlib.math.units.extensions.meters
import frc.team6502.kyberlib.motorcontrol.KMotorController
import java.lang.Math.atan2
import java.lang.Math.sqrt
import kotlin.math.abs

/*
abstract class SwerveDrive : SubsystemBase() {
    abstract val forwardAxis: KAxis
    abstract val strafeAxis: KAxis
    abstract val rotationAxis: KAxis

    abstract val width : Double
    abstract val length: Double
    abstract val gyro: PigeonIMU
    abstract val trackWidth: Double
    private val r = sqrt((length * length) + (width * width))

    abstract val frontLeft: KMotorController
    abstract val frontRight: KMotorController
    abstract val backLeft: KMotorController
    abstract val backRight: KMotorController

    val kinematics = DifferentialDriveKinematics(trackWidth)
    val odometry = DifferentialDriveOdometry(Rotation2d())

    var heading
        get() = gyro.fusedHeading
        set(value) = setLocation(value)

    fun angleToLoc(angle: Double): Double {
        return if (angle < 0) {
            .5 + (180.0 - abs(angle)) / 360.0
        } else {
            angle / 360.0
        }
    }

    fun input() {
        drive(forwardAxis.value, strafeAxis.value, rotationAxis.value)

    }

    fun drive(fwd: Double, str: Double, rot: Double) {
        val a = str - (rot * (length / r));
        val b = str + (rot * (length / r));
        val c = fwd - (rot * (width/ r));
        val d = fwd + (rot * (width / r));

        var ws1 = sqrt((b * b) + (c * c));
        var ws2 = sqrt((b * b) + (d * d));
        var ws3 = sqrt((a * a) + (d * d));
        var ws4 = sqrt((a * a) + (c * c));

        val wa1 = atan2(b, c) * 180 / Math.PI;
        val wa2 = atan2(b, d) * 180 / Math.PI;
        val wa3 = atan2(a, d) * 180 / Math.PI;
        val wa4 = atan2(a, c) * 180 / Math.PI;

        var max = ws1;
        max = max.coerceAtLeast(ws2);
        max = max.coerceAtLeast(ws3);
        max = max.coerceAtLeast(ws4);
        if (max > 1) {
            ws1 /= max;
            ws2 /= max;
            ws3 /= max;
            ws4 /= max;
        }
        setPower(ws4, ws2, ws1, ws3);
        setLocation(angleToLoc(wa4), angleToLoc(wa2), angleToLoc(wa1), angleToLoc(wa3));
    }

    fun setPower(p1: Double, p2: Double, p3: Double, p4: Double) {
        frontLeft.setPower(p1)
        frontRight.setPower(p2)
        backLeft.setPower(p3)
        backRight.setPower(p4)
    }

    fun rotateTo(direction: Double) {}

    fun setLocation(test: Double) {setLocation(test, test, test, test)}
    fun setLocation(l1: Double, l2: Double, l3: Double, l4: Double) {
    }

    fun tick() {
        odometry.update(
            Rotation2d.fromDegrees(heading),
            leftEnc.position,
            rightEnc.position
        )
    }



}
*/