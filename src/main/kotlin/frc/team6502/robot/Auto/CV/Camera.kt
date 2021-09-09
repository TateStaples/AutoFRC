package frc.team6502.robot.Auto.CV

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.kyberlib.math.units.extensions.Distance
import frc.team6502.kyberlib.math.units.extensions.degrees
import frc.team6502.kyberlib.math.units.extensions.meters
import frc.team6502.robot.RobotContainer
import org.opencv.core.Mat

object Camera {
    private val executables = ArrayList<() -> Unit>()
    const val width = 640
    const val height = 480

    val hFOV = 90.0.degrees
    val vFOV = 90.0.degrees

    private val camera = CameraServer.getInstance().startAutomaticCapture()
    private val cvSink = CameraServer.getInstance().video
    private val outputStream = CameraServer.getInstance().putVideo("Video", 640, 480)

    private val source = Mat()

    fun addFunction(f: ()->Unit) {
        executables.add(f)
    }

    val thread = Thread {
        while (!Thread.interrupted()) {
            if (currentImage.empty())
                continue
            for (e in executables)
                e()
        }
    }

    init {
        thread.start()
    }

    fun deproject(detectedPosition: Translation2d, xPixel: Int, yPixel: Int, dis: Distance, cameraRotation: Rotation2d = RobotContainer.navigation.heading): Translation2d {
        val hPercent = width / xPixel - 0.5
//        val vPercent = height / yPixel - 0.5

        // these will be circular coordinates
        val hTheta = hPercent * hFOV.degrees
//        val vTheta = vPercent * vFOV.degrees
        val delta = Translation2d(dis.meters, hTheta.degrees.plus(cameraRotation))  // TODO no idea if this works at all
        val currentPos = detectedPosition.minus(delta) // this doesnt work
        return currentPos
    }

    val currentImage: Mat
        get() {
            cvSink.grabFrame(source)
            currentImageTime = Timer.getFPGATimestamp()
            outputStream.putFrame(source)
            return source
        }

    var currentImageTime = Timer.getFPGATimestamp()

}