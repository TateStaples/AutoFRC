package frc.team6502.robot.tests

import edu.wpi.cscore.HttpCamera
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team6502.robot.SlamValues
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import kyberlib.command.KRobot
import kyberlib.sensors.Limelight
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs
import java.io.File

class VisionRobotTestbed : KRobot() {
    private val url = "http://10.65.2.11"  // http://10.TE.AM.11:5800
    private val video = HttpCamera("raw", "$url:1182/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer)
    private val lime = Limelight().apply { driverMode = true }

    init {
//        Shuffleboard.getTab("Limelight feed").add(video)
        CameraServer.getInstance().startAutomaticCapture(video)
//        CameraServer.getInstance().addCamera(video)
//        CameraServer.getInstance().putVideo("test", 320, 240)
    }
    var test = 1.0
    override fun robotPeriodic() {
        SmartDashboard.putBoolean("image", video.isConnected)
    }


    private val slamValues = File("./UcoSlam/slamValues.json")
    private val table = NetworkTableInstance.getDefault().getTable("SLAM")
    private val xEntry = table.getEntry("X")
    private val yEntry = table.getEntry("Y")
    private val thetaEntry = table.getEntry("THETA")
    private val outputTimeEntry = table.getEntry("OUTPUT TIME")
    private var prevOutputTime = -1.0

    override fun simulationInit() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        if (!slamValues.exists()) {
            val defaultVals = SlamValues(0.0, 0.0, 0.0,0.0, 0.0, true)
            val jsonString = Json.encodeToString(defaultVals)
            slamValues.writeText(jsonString)
        }
//        "frcMonocular.exe".runCommand(File("./UcoSlam/"))
    }

    override fun simulationPeriodic() {
        // read to SLAM output
        val deserialized = Json.decodeFromString<SlamValues>(slamValues.readText())
        if (prevOutputTime != deserialized.outputImgTime) {
            prevOutputTime = deserialized.outputImgTime
            xEntry.setNumber(deserialized.X)
            yEntry.setNumber(deserialized.Y)
            thetaEntry.setNumber(deserialized.THETA)
            outputTimeEntry.setNumber(deserialized.outputImgTime)
        }

        // write the SLAM input
        val mat = Mat()
        CameraServer.getInstance().video.grabFrame(mat)
        if (!mat.empty()) {
            Imgcodecs.imwrite("./UcoSlam/slamImage.jpg", mat)
            deserialized.newImgTime = Timer.getFPGATimestamp()
        }
        val jsonString = Json.encodeToString(deserialized)
        println(jsonString)
        slamValues.writeText(jsonString)
    }
}