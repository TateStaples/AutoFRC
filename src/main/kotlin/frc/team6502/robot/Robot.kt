package frc.team6502.robot

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.commands.balls.Intake
import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.commands.general.CommandManager
import kotlinx.serialization.Serializable
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import kyberlib.command.Debuggable
import kyberlib.command.KRobot
import kyberlib.math.units.zeroPose
import kyberlib.runCommand
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs
import java.io.File


/**
 * Main robot class. Runs the main control loops
 */
class Robot : KRobot() {
    override fun robotInit() {
        // initialize RobotContainer and by extension subsystems
        RobotContainer
    }

    override fun autonomousInit() {
        Constants.AUTO = true
        Navigation.pose = zeroPose
        val traj = Navigation.trajectory(Translation2d(1.0,0.0), Translation2d(1.0, 2.0))
        val drive1 = AutoDrive(traj)  // Drives here
//        val seq = CommandManager.sequence(drive1, Intake())
//        val drive2 = AutoDrive(Translation2d(10.0, 3.0))  // drives here
        CommandManager.enqueue(drive1)
        CommandManager.enqueue(Intake())
    }

    override fun teleopInit() {
        Constants.AUTO = false
        Navigation.pose = zeroPose
    }

    private val slamValues = "slamValues.json"
    private val table = NetworkTableInstance.getDefault().getTable("SLAM")
    private val xEntry = table.getEntry("X")
    private val yEntry = table.getEntry("Y")
    private val thetaEntry = table.getEntry("THETA")
    private val outputTimeEntry = table.getEntry("OUTPUT TIME")
    private var prevOutputTime = -1.0

    override fun simulationInit() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        val saveFile = File(slamValues)
        if (!saveFile.exists()) {
            val defaultVals = SlamValues(0.0, 0.0, 0.0,0.0, 0.0, 1)
            val jsonString = Json.encodeToString(defaultVals)
            File(slamValues).writeText(jsonString)
        }
        val runCommand = " .\\UcoSlam\\frc_monocular.exe -camConfigPath UcoSlam\\data\\limelight.yml -params UcoSlam\\data\\fileout.yml -voc Ucoslam\\data\\orb.fbow -valJson Ucoslam\\slamValues.json\n"
        //  .\UcoSlam\frc_monocular.exe -camConfigPath UcoSlam\data\limelight.yml -params UcoSlam\data\fileout.yml -voc Ucoslam\data\orb.fbow -valJson Ucoslam\slamValues.json
//        "frcMonocular.exe".runCommand(File("./UcoSlam/"))
    }

    override fun simulationPeriodic() {
        // read to SLAM output
        val deserialized = Json.decodeFromString<SlamValues>(slamValues)
        if (prevOutputTime == deserialized.outputImgTime) return
        prevOutputTime = deserialized.outputImgTime
        xEntry.setNumber(deserialized.X)
        yEntry.setNumber(deserialized.Y)
        thetaEntry.setNumber(deserialized.THETA)
        outputTimeEntry.setNumber(deserialized.outputImgTime)

        // write the SLAM input
        val mat = Mat()
        CameraServer.getInstance().video.grabFrame(mat)
        Imgcodecs.imwrite("slam.jpg", mat)
        deserialized.newImageTime = Timer.getFPGATimestamp()
        val jsonString = Json.encodeToString(deserialized)
        println(jsonString)
        File(slamValues).writeText(jsonString)
    }
}

@Serializable
data class SlamValues (
    var X:Double, var Y: Double, var THETA: Double, var outputImgTime: Double,
    var newImageTime: Double, var running: Int) : Debuggable() {

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
