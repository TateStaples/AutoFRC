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
import org.opencv.calib3d.Calib3d
import org.opencv.core.*
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

    var found = 0

    val objectPoints = mutableListOf<Mat>()
    val imagePoints = mutableListOf<Mat>()
    fun calibrate() {
        println("found $found images")
        if (found > 10) {
            val outputMatrix = Mat()
            val dist = Mat()
            val rvecs = mutableListOf<Mat>()
            val tvecs = mutableListOf<Mat>()
            Calib3d.calibrateCamera(objectPoints, imagePoints, Size(640.0, 480.0),outputMatrix, dist, rvecs, tvecs)
            println("Camera Matrix: $outputMatrix")
            println("dist: $dist")
            println("r: $rvecs")
            println("t: $tvecs")
            0/0
        }
        val mat = Mat()
        CameraServer.getInstance().video.grabFrame(mat)
        if (mat.empty()) return
        val gray = Mat()
        Imgproc.cvtColor(mat, gray, Imgproc.COLOR_BGR2GRAY)
        val corners = MatOfPoint2f()
        val hasBoard = Calib3d.findChessboardCorners(gray, Size(6.0, 8.0), corners)
        if (hasBoard) {
            found++
            var points: MatOfPoint3f

            val a: Mat = MatOfPoint3f()
            for (y in 0 until 6) {
                for (x in 0 until 8) {
                    points = MatOfPoint3f(Point3(x.toDouble(), y.toDouble(), 0.0))
                    a.push_back(points)
                }
            }
            objectPoints.add(a)
            imagePoints.add(corners)

        }
    }

    override fun periodic() {
        calibrate()
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
