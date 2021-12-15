package frc.team6502.robot.auto.cv

import edu.wpi.cscore.VideoMode
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.MedianFilter
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.commands.balls.Intake
import kyberlib.auto.Navigator
import kyberlib.command.Debug
import kyberlib.command.DebugLevel
import kyberlib.math.units.Translation2d
import kyberlib.math.units.extensions.KRotation
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.inches
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.transform
import kyberlib.simulation.field.KField2d
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

/**
 * Camera that sends images back to robot for global position updates and HSV thresholding
 */
object Vision : SubsystemBase(), Debug {
    // camera setups
    private val url = "http://10.65.2.11"
//    private val video = HttpCamera("raw", "$url:1182/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer)
    private val camera = PhotonCamera("gloworm")

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
        ballSearch()
//        targetUpdate()
    }

    private val glowormTable = NetworkTableInstance.getDefault().getTable("photonvision/gloworm")
    private val yawEntry = glowormTable.getEntry("targetYaw")
    private val pitchEntry = glowormTable.getEntry("targetPitch")
    val yaw: KRotation
        get() {
//            return Vision.targetResult.bestTarget.yaw
            return yawEntry.getDouble(0.0).degrees
        }

    val pitch: KRotation
        get() {
            return pitchEntry.getDouble(0.0).degrees
        }

    private val cameraHeight = 14.inches
    private val distance
        get() = cameraHeight * tan(pitch.radians.absoluteValue)
    fun ballSearch() {
        val res = camera.latestResult
        if (res.hasTargets() && res.bestTarget.pitch > 0) {
            if (pitch.degrees > -10.0) return
            val currentPose = RobotContainer.navigation.pose
            val estimatedBallPose = currentPose.transformBy(Pose2d(cos(distance.meters), sin(distance.meters), yaw).transform)
            val time =  Timer.getFPGATimestamp() - res.latencyMillis
            log( "object : $distance")
            log("estimated pose: $estimatedBallPose, currentPose: $currentPose")
            if (distance.meters > 3.0) {
                log("object to far away")
                return
            }
            KField2d.addGoal(estimatedBallPose.translation, time, "ball", Intake())
        }
    }

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

    override var priority: DebugLevel = DebugLevel.LowPriority
        get() = DebugLevel.LowPriority

    override fun debugValues(): Map<String, Any?> {
        val map = mutableMapOf<String, Double>(
            "global/x" to lastPos.x,
            "global/y" to lastPos.y
        )
        val res = camera.latestResult
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
            val photonTrackedTarget = res.bestTarget
            map.putAll(
                mapOf(
                    "bestTarget/pitch" to photonTrackedTarget.pitch,
                    "bestTarget/yaw" to photonTrackedTarget.yaw,
                    "bestTarget/skew" to photonTrackedTarget.skew,
                    "bestTarget/area" to photonTrackedTarget.area,
                )
            )
        }
        return map.toMap()
    }
}