package frc.team6502.robot.Auto

//import edu.wpi.first.cameraserver.CameraServer
import org.opencv.core.Mat
import org.opencv.objdetect.QRCodeDetector
import org.opencv.videoio.VideoCapture
import java.awt.image.DataBufferByte
import java.io.File
import javax.imageio.ImageIO


object FiducialTracker {
    const val width = 640
    const val height = 480

//    val camera = CameraServer.getInstance().startAutomaticCapture()
//    val cvSink = CameraServer.getInstance().video
//    val outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480)
    val source = Mat()
    val output = Mat()

    val tracker = QRCodeDetector()

    val test = VideoCapture(0)

    @JvmStatic
    fun main(args: Array<String>) {
//        create(1)
    }


    val thread = Thread {
        while (!interrputCV) {
            if (image.empty()){// == 0L) {
                continue
            }
            execute()
        }
    }

    fun create(amount: Int) {
        val pts = Mat()
        val string = tracker.detectAndDecode(image, pts)
        println(string)
        println(pts.toString())

    }

    init {
//        camera.setResolution(width, height)
    }

    val image: Mat
        get() {
            test.read(source)
            val bufferedImage = ImageIO.read(File("QR-code.png"))
            var pixels = (bufferedImage.raster.dataBuffer as DataBufferByte).data
            source.put(0, 0, pixels)
            return source
        }

    val interrputCV
        get() = !Thread.interrupted()

    fun start() {
        thread.start()
    }

    fun execute() {
//        Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY)
//        outputStream.putFrame(output)
    }

}