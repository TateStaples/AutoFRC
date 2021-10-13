package frc.team6502.robot.auto.cv

import edu.wpi.cscore.HttpCamera
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import frc.team6502.robot.RobotContainer
import kyberlib.math.units.Pose2d
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.inches
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.transform
import org.photonvision.PhotonCamera

/**
 * Testing Photon vision stuff
 */
object Photon {
    private const val cameraName = "http://gloworm.local:5800/"
    private val cameraOffset = Pose2d(0.inches, 0.inches, 0.degrees)
    val camera = PhotonCamera(cameraName)
    val feed = HttpCamera("limelight", "$cameraName/stream.mjpg")

    init {
//        val vid = VideoSource()
//        CameraServer.getInstance().addCamera(

    }

    fun update() {
        val targetPose = Pose2d(1.meters, 1.meters, 0.degrees)
        val res = camera.latestResult
        if (res.hasTargets()) {
            val imageCaptureTime: Double = Timer.getFPGATimestamp() - res.latencyMillis
            val camToTargetTrans: Transform2d = res.bestTarget.cameraToTarget
            val camPose: Pose2d = targetPose.transformBy(camToTargetTrans.inverse())
            val location = camPose.transformBy(cameraOffset.transform)
            RobotContainer.navigation.update(
                location, imageCaptureTime
            )
        }
    }
}