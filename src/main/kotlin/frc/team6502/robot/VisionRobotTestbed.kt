package frc.team6502.robot

import edu.wpi.cscore.HttpCamera
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
//import frc.team6502.robot.auto.cv.Camera
import kyberlib.command.KRobot
import kyberlib.vision.Limelight
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.videoio.VideoWriter

class VisionRobotTestbed : KRobot() {
    private val url = "http://10.65.2.11"  // http://10.TE.AM.11:5800
    private val video = HttpCamera("raw", "$url:1182/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer)
    private val lime = Limelight().apply { driverMode = true }

    init {
//        Shuffleboard.getTab("Limelight feed").add(video)
//        CameraServer.getInstance().startAutomaticCapture(video)
        CameraServer.getInstance().addCamera(video)
//        CameraServer.getInstance().putVideo("test", 320, 240)
    }

    override fun robotPeriodic() {
        SmartDashboard.putBoolean("image", video.isConnected)
    }

    val frame = Mat()
    val size =  Size(320.0, 240.0)
    val fourcc = VideoWriter.fourcc('a', 'v', 'c', '1')
    val save = "example.mp4"
    val writer = VideoWriter()
    override fun autonomousInit() {
        writer.open(save, fourcc, 20.0, size)
    }
    override fun autonomousPeriodic() {
        CameraServer.getInstance().video.grabFrame(frame)
        writer.write(frame)
    }

    override fun teleopInit() {
        writer.release()
    }
}