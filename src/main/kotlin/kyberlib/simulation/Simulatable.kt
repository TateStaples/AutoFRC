package kyberlib.simulation


/**
 * Interface that allows a subsystem to hook into the simulation
 */
interface Simulatable {
    fun simUpdate(dt: Double)
}