package frc.team6502.robot.auto.cv

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team6502.robot.RobotContainer
import kyberlib.math.units.extensions.inches
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.extensions.radians
import org.opencv.aruco.Aruco
import org.opencv.aruco.Board
import org.opencv.calib3d.Calib3d.Rodrigues
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs.imwrite


object FiducialTracker {
    val map = mapOf(
        1 to Pose2d(0.0, 0.0, 0.radians)  // change to 3d points
    )

    val width = 10.inches  // marker size

    private val dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_6X6_250)
    private val corners = ArrayList<Mat>()
    private val ids = Mat()
    val rotations = Mat()
    val translations = Mat()
    val board = Board.create(corners, dictionary, ids)

//    init {
//        val keys = map.keys.toIntArray()
//        ids.put(0, 0, keys)
//    }

    @JvmStatic
    fun main(args: Array<String>) {
        create(1)
    }

    fun create(amount: Int) {
        val image = Mat()
        val dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_6X6_250)
        for (id in 0..amount) {
            Aruco.drawMarker(dictionary, id, 200, image)
            imwrite("marker$id.png", image)
        }
    }

    fun calibrate() {
        val cameraRotation = Mat()
        val cameraDistortion = Mat()
        val image = Camera.currentImage
        Aruco.detectMarkers(Camera.currentImage, dictionary, corners, ids) // this doesn't work, need multiple viewpoints
        if (corners.isEmpty()) {
            println("no aruco found")
            return
        }
        Aruco.calibrateCameraAruco(corners, ids, Mat(), board, image.size(), cameraRotation, cameraDistortion)
        println("Rotation: $cameraRotation")
        println("Distortion: $cameraDistortion")

    }

    fun execute() {
        val image = Camera.currentImage
        val time = Camera.currentImageTime
        Aruco.detectMarkers(image, dictionary, corners, ids)
        if (corners.isEmpty()) return
//        Aruco.estimatePoseBoard(corners, ids, board, Camera.matrix, Camera.distortion, rotations, translations, false)

        Aruco.estimatePoseSingleMarkers(corners, width.meters.toFloat(), Camera.matrix, Camera.distortion, rotations, translations)

        val r = Mat()
        for (index in 0..ids.rows()) {
            val id = ids[index, 0][0].toInt()
//            val corners = corners[index]
//            val boardPose = map[id]

            val translation = translations.row(index)
            val rotation = rotations.row(index)

            // https://stackoverflow.com/questions/52833322/using-aruco-to-estimate-the-world-position-of-camera
            Rodrigues(rotation, r)
            val extrinsicVals = arrayOf(
                doubleArrayOf(r[0, 0][0], r[0, 1][0], r[0, 2][0], translation[0, 0][0]),
                doubleArrayOf(r[1, 0][0], r[1, 1][0], r[1, 2][0], translation[0, 0][0]),
                doubleArrayOf(r[2, 0][0], r[2, 1][0], r[0, 2][0], translation[0, 0][0]),
                doubleArrayOf(0.0, 0.0, 0.0, 1.0)
            )
            val extrinsic = Camera.arrayToMat(extrinsicVals)
            val extrinsicInv = extrinsic.inv()
            val worldPos = doubleArrayOf(extrinsicInv[0,3][0], extrinsicInv[1,3][0], extrinsicInv[2,3][0])
            val location = Translation2d(worldPos[0], worldPos[1])
            println("camera update took ${Timer.getFPGATimestamp() - time} seconds")
            RobotContainer.navigation.update(Pose2d(location, RobotContainer.navigation.heading), time)
        }
    }

    fun debug(vararg test: Mat) {
        for ((counter, thing) in test.withIndex()) {
            SmartDashboard.putString("Test $counter", thing.toString())
        }
    }
}