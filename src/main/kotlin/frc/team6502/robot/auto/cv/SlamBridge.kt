package frc.team6502.robot.auto.cv

import edu.wpi.cscore.HttpCamera
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlinx.serialization.Serializable
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import kyberlib.command.Debug
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.imgcodecs.Imgcodecs
import org.opencv.imgproc.Imgproc
import java.io.File

/**
 * Bridge between the robot and UcoSlam algorithm.
 * Reads images from CameraServer and writes to image for frc_monocular
 * Reads output from UcoSlam and writes to networkTables
 *
 * @property slamValues the file where to store the json out of UcoSlam
 * @property table NetworkTable that hold the UcoSlam output
 *
 * @throws crash should not be intialized on real robot, only simulation
 */
class SlamBridge : SubsystemBase() {
    init {
        val url = "http://10.65.2.2:1181/?action=stream"
        CameraServer.getInstance().startAutomaticCapture(HttpCamera("USB Camera 0", url, HttpCamera.HttpCameraKind.kMJPGStreamer))
    }
    private val tableInstance = NetworkTableInstance.getDefault()
    private val table = tableInstance.getTable("SLAM")
    private var lastUpdate = 0.0
    private val xEntry = table.getEntry("X")
    private val yEntry = table.getEntry("Y")
    private val thetaEntry = table.getEntry("THETA")
    private val outputTimeEntry = table.getEntry("OUTPUT TIME")

    private val slamValues = File("./UcoSlam/slamValues.json")

    init {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        if (!slamValues.exists()) {
            val defaultVals = SlamValues(0.0, 0.0, 0.0,0.0, 0.0, 1)
            val jsonString = Json.encodeToString(defaultVals)
            slamValues.writeText(jsonString)
        }
    }

    var index = 0
    fun save_image() {
        val mat = Mat()
        CameraServer.getInstance().video.grabFrame(mat)
        Imgcodecs.imwrite("./UcoSlam/calibration/image$index.jpg", mat)
        println("here to images/image$index.jpg")
        index++
    }

    override fun periodic() {
        save_image()
        return
        // read to SLAM output
        var deserialized = Json.decodeFromString<SlamValues>(slamValues.readText())
        if (lastUpdate != deserialized.outputImgTime) {
            lastUpdate = deserialized.outputImgTime
            xEntry.setNumber(deserialized.X)
            yEntry.setNumber(deserialized.Y)
            thetaEntry.setNumber(deserialized.THETA)
            outputTimeEntry.setNumber(deserialized.outputImgTime)
            slamValues.writeText(Json.encodeToString(deserialized))
        }

        // write the SLAM input
        val mat = Mat()
//        TimeUnit.SECONDS.sleep((1.0 / 3.0).toLong())
        CameraServer.getInstance().video.grabFrame(mat)
//        cap.read(mat)
//        val resized = Mat()
//        Imgproc.resize(mat, resized, Size(320.0, 240.0))
        deserialized = Json.decodeFromString(slamValues.readText())
        if (!mat.empty()) {
            Imgcodecs.imwrite("./UcoSlam/slamImage.jpg", mat)
            deserialized.newImageTime = Timer.getFPGATimestamp()
            slamValues.writeText(Json.encodeToString(deserialized))
        }
        deserialized.debugDashboard()
    }
}


@Serializable
data class SlamValues (
    var X:Double, var Y: Double, var THETA: Double, var outputImgTime: Double,
    var newImageTime: Double, var running: Int) : Debug {

    override fun debugValues(): Map<String, Any> {
        return mapOf(
            "x" to X,
            "y" to Y,
            "theta" to THETA,
            "out" to outputImgTime,
            "new" to newImageTime,
            "running" to running,
        )
    }
}
