package frc.team6502.robot.auto.cv

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import kyberlib.math.units.extensions.Distance
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.meters
import frc.team6502.robot.RobotContainer
import org.opencv.core.Mat


object Camera {
    val matrix = Mat()  // represent rotation and stuff (this should be getter method because depends on fused heading)
    val distortion = Mat() // represents camera distortion
    val cameraOffset = Translation2d(0.0, 0.0)  // how far off the camera is from the center of the robot (shouldn't be super important)

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
            val mat = Mat()
            source.copyTo(mat)
            return mat
        }

    var currentImageTime = Timer.getFPGATimestamp()

    fun matToArray(mat: Mat): Array<DoubleArray> {
        val array = Array(mat.height()) { DoubleArray(mat.width())
        }
        for (i in 0 until mat.height()) {
            for (j in 0 until mat.width()) {
                array[i][j] = mat.get(i, j)[0]
            }
        }
        return array
    }

    fun arrayToMat(array: Array<DoubleArray>): Mat {
        val rows = array.size
        val cols = array[0].size
        val matObject = Mat()
        for (row in 0..rows) {
            for (col in 0..cols)
                matObject.put(row, col, array[row][col])
        }
        return matObject
    }

    fun arrayToMat(array: DoubleArray): Mat {
        val matObject = Mat()
        matObject.put(0, 0, *array)  // idk if this works
        return matObject

    }


}