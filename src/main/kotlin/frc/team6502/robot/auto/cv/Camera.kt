package frc.team6502.robot.auto.cv

import edu.wpi.cscore.HttpCamera
import edu.wpi.first.cameraserver.CameraServer
import kyberlib.math.units.Translation2d
import kyberlib.math.units.extensions.feet
import kyberlib.sensors.Limelight


object Camera {
    private val url = "http://10.65.2.11"  // http://10.TE.AM.11:5800
    private val video = HttpCamera("raw", "$url:1182/stream.mjpg", HttpCamera.HttpCameraKind.kMJPGStreamer)
    private val lime = Limelight().apply { driverMode = true }

    init {
//        Shuffleboard.getTab("Limelight feed").add(video)
        CameraServer.getInstance().addCamera(video)
    }
    val cameraOffset = Translation2d(0.feet, 0.feet)  // how far off the camera is from the center of the robot (shouldn't be super important)


}