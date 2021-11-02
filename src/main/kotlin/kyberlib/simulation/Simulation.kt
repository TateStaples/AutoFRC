package kyberlib.simulation

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * Simulation that will run a loop to update simulatable objects
 */
object Simulation : SubsystemBase() {
    private val sims = ArrayList<Simulatable>()
    private var prevTime = -1.0
    private val time: Double
        get() = Timer.getFPGATimestamp()
    private val startTime = time
    val elapsedTime
        get() = time - startTime
    val field = Field2d()  // todo: use KField

    init {
        SmartDashboard.putData("Field", field)
    }

    override fun periodic() {
        if (RobotBase.isReal()) {
            println("incorrectly called Simulation.kt")
            return
        }

        if (prevTime < 0) {
            prevTime = time
            return
        }
        val dt = time - prevTime
        for (sim in sims) {
            sim.simUpdate(dt)
        }
        prevTime = time
    }

    fun include(simulatable: Simulatable) {
        sims.add(simulatable)
    }
}