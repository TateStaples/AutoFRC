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
import kyberlib.runCommand
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
class SlamBridge {
    init {
        val url = "http://10.65.2.2:1181/?action=stream"
//        CameraServer.getInstance().startAutomaticCapture(HttpCamera("USB Camera 0", url, HttpCamera.HttpCameraKind.kMJPGStreamer))
    }


    companion object {
        @JvmStatic
        fun main(args: Array<String>) {
            val s = SlamBridge()
            while (true) {
                s.periodic()
            }
        }
    }
    private val tableInstance = NetworkTableInstance.getDefault()
    private val table = tableInstance.getTable("SLAM")
    private var lastUpdate = 0.0
    private val xEntry = table.getEntry("X")
    private val yEntry = table.getEntry("Y")
    private val zEntry = table.getEntry("Z")
    private val outputTimeEntry = table.getEntry("OUTPUT TIME")

    init {
        tableInstance.startClientTeam(6502);  // where TEAM=190, 294, etc, or use inst.startClient("hostname") or similar
        tableInstance.startDSClient();  // recommended if running on DS computer; this gets the robot IP from the DS
    }

    private val slamValues = File("./UcoSlam/slamValues.json")

    init {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        if (!slamValues.exists()) {
            val defaultVals = SlamValues(0.0, 0.0, 0.0, 0.0,0.0, 1)
            val jsonString = Json.encodeToString(defaultVals)
            slamValues.writeText(jsonString)
        }
        else {
            val deserialized = Json.decodeFromString<SlamValues>(slamValues.readText())
            deserialized.running = 1
            slamValues.writeText(Json.encodeToString(deserialized))

        }
//        ".\\UcoSlam\\frc_monocular.exe -debug 2".runCommand(File(".\\UcoSlam"))
    }

    fun periodic() {
        // read to SLAM output
        try {
            var deserialized = Json.decodeFromString<SlamValues>(slamValues.readText())
            if (lastUpdate != deserialized.outputImgTime) {
                println("writing!")
                lastUpdate = deserialized.outputImgTime
                xEntry.setNumber(deserialized.X)
                yEntry.setNumber(deserialized.Y)
                zEntry.setNumber(deserialized.Z)
                outputTimeEntry.setNumber(deserialized.outputImgTime)
                slamValues.writeText(Json.encodeToString(deserialized))
            }

            // write the SLAM input
            val mat = Mat()
            CameraServer.getInstance().video.grabFrame(mat)
            deserialized = Json.decodeFromString(slamValues.readText())
            if (!mat.empty()) {
                Imgcodecs.imwrite("./UcoSlam/slamImage.jpg", mat)
                deserialized.newImageTime = Timer.getFPGATimestamp()
                slamValues.writeText(Json.encodeToString(deserialized))
            }
            deserialized.debugDashboard()
        } catch (e: Exception) {
            println(e.stackTrace.toString())
        }
    }
}


@Serializable
data class SlamValues (
    var Z: Double,
    var X:Double, var Y: Double, var outputImgTime: Double,
    var newImageTime: Double, var running: Int) : Debug {

    override fun debugValues(): Map<String, Any> {
        return mapOf(
            "x" to X,
            "y" to Y,
            "z" to Z,
            "out" to outputImgTime,
            "new" to newImageTime,
            "running" to running,
        )
    }
}
