package frc.team6502.robot.auto.CV

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import kyberlib.math.units.extensions.inches
import kyberlib.math.units.extensions.meters
import frc.team6502.robot.RobotContainer
import org.opencv.core.Mat
import org.opencv.objdetect.QRCodeDetector
import java.awt.image.DataBufferByte
import java.io.File
import javax.imageio.ImageIO
import kotlin.math.tan


object QRTracker {
    val tracker = QRCodeDetector()
    val QRWidth = 4.inches


    fun start() {
        Camera.addFunction { execute() }
    }

    val locationMap = mapOf(
        "location 1" to Translation2d(3.0, 3.0)
    )

    private fun execute() {
        val points = Mat()
        val image = Camera.currentImage
        val time = Camera.currentImageTime
        if (image.empty()) return
        val message = tracker.detectAndDecode(Camera.currentImage, points)
        if (!locationMap.contains(message)) {
            println("Unknown QR code Found: $message, $points")
            return
        }
        val buffer = IntArray(points.rows() * 2)
        val xVals = IntArray(points.rows())
        val yVals = IntArray(points.rows())
        points.get(0, 0, buffer)
        for (i in 0..buffer.size) {
            if (i.div(2) == 0) yVals[i.div(2)] = buffer[i]   // even index
            else xVals[i.div(2)] = buffer[i]  // odd index
        }
        val x = xVals.sum() / xVals.size
        val y = yVals.sum() / yVals.size
//        val rotation = 0.degrees

//        val width = QRWidth.value
        val height = QRWidth.value
        val dis = height.meters / tan(xVals.maxOrNull()!!-xVals.minOrNull()!! / Camera.width * Camera.hFOV.radians)

        val location = Camera.deproject(locationMap[message]!!, x, y, dis)
        RobotContainer.navigation.update(Pose2d(location, RobotContainer.navigation.heading), time)
//        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY)
//        outputStream.putFrame(output)
    }

    val exampleImage: Mat
        get() {
            val bufferedImage = ImageIO.read(File("QR-code.png"))
            val pixels = (bufferedImage.raster.dataBuffer as DataBufferByte).data
            val source = Mat()
            source.put(0, 0, pixels)
            return source
        }

}