package frc.team6502.robot.Auto.CV

import edu.wpi.first.wpilibj.geometry.Translation2d
import org.opencv.aruco.Aruco
import org.opencv.aruco.Board
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs.imwrite


object FiducialTracker {
    val map = mapOf(
        1 to Translation2d(0.0, 0.0)
    )

    private val dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_6X6_250)
    private val corners = ArrayList<Mat>()
    private val ids = Mat()
    val rotations = Mat()
    val translations = Mat()
    val board = Board.create(corners, dictionary, ids)

    init {
        val keys = map.keys.toIntArray()
        ids.put(0, 0, keys)
    }

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
        Aruco.detectMarkers(Camera.currentImage, dictionary, corners, ids)
        Aruco.estimatePoseBoard(corners, ids, board, Camera.matrix, Camera.distorsion, rotations, translations, false)
        val arrayTrans = Camera.matToArray(translations)
        val arrayRot = Camera.matToArray(rotations)

        for (index in 0..ids.rows()) {
            val id = ids[index, 0][0].toInt()
            val translation = arrayTrans[index]
            val rotation = arrayRot[index]
        }
    }
}