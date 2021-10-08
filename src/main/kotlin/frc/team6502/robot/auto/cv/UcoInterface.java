package frc.team6502.robot.auto.cv;

import org.opencv.core.Mat;

public class UcoInterface {
    native static void setup(String mode, String worldName, Mat cameraMatrix, Mat distorsion);
    native static Mat update(Mat img);

    native static void listWorlds();
    native static void deleteWorld(String worldName);
}
