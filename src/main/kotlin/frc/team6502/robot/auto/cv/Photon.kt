package frc.team6502.robot.auto.cv

import edu.wpi.cscore.HttpCamera
import edu.wpi.cscore.VideoMode
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.commands.balls.Intake
import kyberlib.auto.Navigator
import kyberlib.math.units.Pose2d
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.inches
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.extensions.radians
import kyberlib.math.units.transform
import kyberlib.simulation.field.KField2d
import org.photonvision.PhotonCamera

/**
 * Camera that sends images back to robot for global position updates and HSV thresholding
 */
class Photon : SubsystemBase() {
    // camera setups
//    private val url = "http://url10.65.2.11"
//    private val video = HttpCamera("raw", "$url:1182/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer)
//    private val camera = PhotonCamera("$url:5800")

    // this should be initiated on the first update
    private val cameraOffset = Pose2d(3.inches, 0.inches, 0.degrees)

    // Network Tables Info
    private val tableInstance = NetworkTableInstance.getDefault()
    private val table = tableInstance.getTable("SLAM")
    private var lastUpdate = 0.0
    private val xEntry = table.getEntry("X")
    private val yEntry = table.getEntry("Y")
    private val zEntry = table.getEntry("Z")
    private val outputTimeEntry = table.getEntry("OUTPUT TIME")
    private var unitConversion = 1.0  // TODO: figure out how to tune it

    init {
        val video = CameraServer.getInstance().startAutomaticCapture()
        video.videoMode = VideoMode(video.videoMode.pixelFormat, 640, 480, 30)
    }

    override fun periodic() {
        slamUpdate()
        targetUpdate()
    }

    /**
     * Uses the Limelight's HSV thresholding to detect yellow balls
     */
    private fun targetUpdate() {
//        val res = camera.latestResult
//        if (res.hasTargets()) {
//            val imageCaptureTime = Timer.getFPGATimestamp() - res.latencyMillis
//            for (target in res.targets) {
//                val camToTargetTrans: Transform2d = target.cameraToTarget
//                val estimatedPosition = RobotContainer.navigation.pose.plus(camToTargetTrans)
//                KField2d.addGoal(estimatedPosition.translation, imageCaptureTime, "ball", Intake())
//            }
//        }
    }
//

    // todo: tune these
    val xHat = doubleArrayOf(0.0, 0.0, 0.0)
    val yHat = doubleArrayOf(0.0, 0.0, 0.0)
    /**
     * Checks the output of the UcoSlam in Network Tables.
     * Then it applies those results to the pose estimator
     */
    private fun slamUpdate() {
        val updateTime = outputTimeEntry.getDouble(0.0)
        // todo: update initial pose
        if (updateTime != lastUpdate && lastUpdate != 0.0) {
            println("using slam output")
            return
            val cameraVector = doubleArrayOf(xEntry.getDouble(0.0), yEntry.getDouble(0.0), zEntry.getDouble(0.0))
            val x  = dot(cameraVector, xHat).meters
            val y  = dot(cameraVector, yHat).meters
            val slamPose = Pose2d(x, y, Navigator.instance!!.heading)
            val adjustedPose = slamPose.transformBy(cameraOffset.transform)
            RobotContainer.navigation.update(adjustedPose, updateTime)
        }
        lastUpdate = updateTime
    }

    // dot product of vectors
    fun dot(v1: DoubleArray, v2: DoubleArray): Double = v1
        .apply { require(v1.size == v2.size) }
        .zip(v2)
        .sumByDouble { (a, b) -> a * b }
}