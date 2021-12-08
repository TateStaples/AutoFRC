package frc.team6502.robot.auto.cv

import edu.wpi.cscore.VideoMode
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.MedianFilter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.commands.balls.Intake
import kyberlib.auto.Navigator
import kyberlib.command.Debug
import kyberlib.math.units.Translation2d
import kyberlib.math.units.extensions.inches
import kyberlib.math.units.extensions.meters
import kyberlib.simulation.field.KField2d
import org.photonvision.PhotonCamera

/**
 * Camera that sends images back to robot for global position updates and HSV thresholding
 */
object Vision : SubsystemBase(), Debug {
    // camera setups
    private val url = "http://url10.65.2.11"
//    private val video = HttpCamera("raw", "$url:1182/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer)
    private val camera = PhotonCamera("$url:5800")

    val xFilter = MedianFilter(5)
    val yFilter = MedianFilter(5)
    var lastPos = Translation2d()

    // this should be initiated on the first update
    var cameraOffset: Translation2d? = null

    // Network Tables Info
    private val tableInstance = NetworkTableInstance.getDefault()
    private val table = tableInstance.getTable("SLAM")
    private var lastUpdate = 0.0
    private val xEntry = table.getEntry("X")
    private val yEntry = table.getEntry("Y")
    private val zEntry = table.getEntry("Z")
    private val outputTimeEntry = table.getEntry("OUTPUT TIME")

    init {
        val video = CameraServer.getInstance().startAutomaticCapture()
        video.videoMode = VideoMode(video.videoMode.pixelFormat, 640, 480, 30)
    }

    override fun periodic() {
        debugDashboard()
        slamUpdate()
//        targetUpdate()
    }

    /**
     * Uses the Limelight's HSV thresholding to detect yellow balls
     */
    private fun targetUpdate() {
        val res = camera.latestResult
        if (res.hasTargets()) {
            val imageCaptureTime = Timer.getFPGATimestamp() - res.latencyMillis
            for (target in res.targets) {
                val camToTargetTrans: Transform2d = target.cameraToTarget
                val estimatedPosition = RobotContainer.navigation.pose.plus(camToTargetTrans)
                KField2d.addGoal(estimatedPosition.translation, imageCaptureTime, "ball", Intake())
            }
        }
    }

    val targetResult
        get() = camera.latestResult

    // todo: find this automatically
    private val xScale = 110.inches.meters
    private val yScale = 117.inches.meters
    private val xHat = doubleArrayOf(15/xScale, -729/xScale, -181/xScale)
    private val yHat = doubleArrayOf(309/yScale, -450/yScale, 127/yScale)
    /**
     * Checks the output of the UcoSlam in Network Tables.
     * Then it applies those results to the pose estimator
     */
    private fun slamUpdate() {
        val updateTime = outputTimeEntry.getDouble(0.0)
        // todo: update initial pose
        if (updateTime != lastUpdate && lastUpdate != 0.0) {
            val cameraVector = doubleArrayOf(xEntry.getDouble(0.0), yEntry.getDouble(0.0), zEntry.getDouble(0.0))
            val x  = dot(cameraVector, xHat)
            val y  = dot(cameraVector, yHat)
            if (cameraOffset == null)
                cameraOffset = Translation2d(x, y)

            val slamPos = Translation2d(x, y)
            val adjustedPos = slamPos - cameraOffset

            val filteredX = xFilter.calculate(x)
            val filteredY = yFilter.calculate(y)
            val filteredPos = Translation2d(filteredX, filteredY)
            lastPos = adjustedPos

            if (filteredPos.getDistance(slamPos) < 2.0)  // prevent wild values from corrupting results
                RobotContainer.navigation.update(edu.wpi.first.wpilibj.geometry.Pose2d(adjustedPos, Navigator.instance!!.heading), Timer.getFPGATimestamp() - 0.2)
        }
        lastUpdate = updateTime
    }

    // dot product of vectors
    fun dot(v1: DoubleArray, v2: DoubleArray): Double = v1
        .apply { require(v1.size == v2.size) }
        .zip(v2)
        .sumByDouble { (a, b) -> a * b }

    override fun debugValues(): Map<String, Any?> {
        val map = mutableMapOf<String, Double>(
            "global/x" to lastPos.x,
            "global/y" to lastPos.y
        )
        val res = targetResult
        if (res.hasTargets()) {
            res.targets.forEachIndexed { index, photonTrackedTarget ->
                map.putAll(
                    mapOf(
                        "target$index/pitch" to photonTrackedTarget.pitch,
                        "target$index/yaw" to photonTrackedTarget.yaw,
                        "target$index/skew" to photonTrackedTarget.skew,
                        "target$index/area" to photonTrackedTarget.area,
                    )
                )
            }
        }
        return map.toMap()
    }
}