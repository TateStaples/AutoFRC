package frc.team6502.robot.auto.cv

import edu.wpi.cscore.HttpCamera
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Transform2d
import frc.team6502.robot.auto.Navigation
import kyberlib.math.units.Pose2d
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.inches
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.extensions.radians
import kyberlib.math.units.transform
import org.photonvision.PhotonCamera

/**
 * Testing Photon vision stuff
 */
object Photon {
    // camera setups
    private const val url = "http://10.65.2.11"
    private val video = HttpCamera("raw", "$url:1182/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer)
    private val camera = PhotonCamera("$url:5800")
    private val cameraOffset = Pose2d(0.inches, 5.inches, 0.degrees)  // TODO: tune this a bit

    // Network Tables Info
    private val tableInstance = NetworkTableInstance.getDefault()
    private val table = tableInstance.getTable("SLAM")
    private var lastUpdate = 0.0
    private val xEntry = table.getEntry("X")
    private val yEntry = table.getEntry("Y")
    private val thetaEntry = table.getEntry("THETA")
    private val outputTimeEntry = table.getEntry("OUTPUT TIME")
    private var unitConversion = 1.0  // TODO: figure out how to tune it

    init {
        CameraServer.getInstance().addCamera(video)
    }

    /**
     * Uses the Limelight's HSV thresholding to detect yellow balls
     */
    fun targetUpdate() {
        val res = camera.latestResult
        if (res.hasTargets()) {
            val imageCaptureTime = Timer.getFPGATimestamp() - res.latencyMillis
            for (target in res.targets) {
                val camToTargetTrans: Transform2d = target.cameraToTarget
                val estimatedPosition = Navigation.pose.plus(camToTargetTrans)
                Navigation.field.addGoal(estimatedPosition.translation, imageCaptureTime, "ball")
            }
        }
    }

    /**
     * Checks the output of the UcoSlam in Network Tables.
     * Then it applies those results to the pose estimator
     */
    fun slamUpdate() {
        val updateTime = outputTimeEntry.getDouble(0.0)
        if (updateTime != lastUpdate && lastUpdate != 0.0) {
            val x  = (xEntry.getDouble(0.0) * unitConversion).meters
            val y  = (yEntry.getDouble(0.0) * unitConversion).meters
            val theta  = thetaEntry.getDouble(0.0).radians
            val slamPose = Pose2d(x, y, theta)
            val adjustedPose = slamPose.transformBy(cameraOffset.transform)
            Navigation.update(adjustedPose, updateTime)
        }
        lastUpdate = updateTime
    }
}