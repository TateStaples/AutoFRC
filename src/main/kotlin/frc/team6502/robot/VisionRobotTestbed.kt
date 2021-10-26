package frc.team6502.robot

import edu.wpi.cscore.HttpCamera
import edu.wpi.cscore.MjpegServer
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kyberlib.command.KRobot
import kyberlib.sensors.Limelight
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs

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


    override fun simulationInit() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
    }
     override fun simulationPeriodic() {
         val thing = Mat()
//         CameraServer.getInstance().video.grabFrame(thing)
         CameraServer.getInstance().video.grabFrameNoTimeout(thing)
         if (!thing.empty()) {
             println("grabbed image")
             Imgcodecs.imwrite("robotImgTest.jpg", thing)
             SmartDashboard.putNumber("test", test)
             println(test)
             test++
             if (test > 100) 0/0
         }
     }
}