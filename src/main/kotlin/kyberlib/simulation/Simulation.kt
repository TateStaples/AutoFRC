package kyberlib.simulation

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * Simulation that will run a loop to update simulatable objects
 */
object Simulation : SubsystemBase() {
    private val sims = ArrayList<Simulatable>()
    private var prevTime = -1.0
    private val time: Double
        get() = Timer.getFPGATimestamp()

    override fun periodic() {
        if (RobotBase.isReal()) {
            println("incorrectly called Simulation")
            return
        }

        if (prevTime < 0) {
            prevTime = time
            return
        }
        val dt = prevTime - time
        for (sim in sims) {
            sim.simUpdate(dt)
        }
    }

    fun include(simulatable: Simulatable) {
        sims.add(simulatable)
    }
}