package frc.team6502.robot.Auto.CV

import org.opencv.aruco.Aruco
import org.opencv.core.Mat
import java.awt.image.DataBufferByte
import java.io.File
import javax.imageio.ImageIO
import org.opencv.aruco.CharucoBoard

object FiducialTracker {

    @JvmStatic
    fun main(args: Array<String>) {
//        create(1)
    }

    fun create(amount: Int) {
        val pts = Mat()
        val test = Aruco()
//        val b = CharucoBoard()

    }

    val exampleImage: Mat
        get() {
            val bufferedImage = ImageIO.read(File("QR-code.png"))
            val pixels = (bufferedImage.raster.dataBuffer as DataBufferByte).data
            val source = Mat()
            source.put(0, 0, pixels)
            return source
        }

    fun execute() {
//        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY)
//        outputStream.putFrame(output)
    }
}