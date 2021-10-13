package frc.team6502.robot

import edu.wpi.cscore.HttpCamera
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kyberlib.command.KRobot
import kyberlib.sensors.Limelight


class VisionRobotTestbed : KRobot() {
    private val url = "http://10.65.2.11"  // http://10.TE.AM.11:5800
    private val video = HttpCamera("raw", "$url:1182/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer)
    private val lime = Limelight().apply { driverMode = true }

    init {
//        Shuffleboard.getTab("Limelight feed").add(video)
//        CameraServer.getInstance().startAutomaticCapture(video)
        CameraServer.getInstance().addCamera(video)
//        CameraServer.getInstance().video
//        CameraServer.getInstance().putVideo("test", 320, 240)
    }

    override fun robotPeriodic() {
        SmartDashboard.putBoolean("image", video.isConnected)
    }
}
