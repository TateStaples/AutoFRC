package kyberlib.auto.pathing

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import kyberlib.auto.trajectory.KTrajectory
import kyberlib.auto.trajectory.KTrajectoryConfig
import kyberlib.command.Debug
import kyberlib.command.DebugLevel
import kyberlib.math.units.Translation2d
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.feet
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.extensions.metersPerSecond
import kyberlib.math.units.towards
import kyberlib.simulation.field.KField2d
import kyberlib.simulation.field.Obstacle
import kotlin.math.sqrt
import kotlin.random.Random

/**
 * Class that generates optimal pathes between various locations on a field
 */
object Pathfinder : Debug {
    /** Tree storing the connections between locations */
    internal val tree = Tree()
    /** Random number generator for point sampling. Seed isn't important. */
    private val random = Random(6)
    /** Data class to store optimization information */
    internal lateinit var information: PathingInformation

    /** Margin of error for a point to be considered at the goal */
    var minGoalDistance = 0.2.feet.value
    /** Whether the goal has been found */
    var pathFound = false
        private set
    /** The node at the goal */
    private var endNode: Node? = null
    /** The series of nodes going from start to end */
    val path: ArrayList<Node>?
        get() = endNode?.let { tree.trace(it) }

    /** how many nodes to create before giving up finding target */
    private const val explorationDepth = 5000
    /** how many nodes to dedicate to optimization */
    private const val optimizationDepth = 50

    /**
     * Generates a trajectory to get from current estimated pose to a separate target
     * @param startPose2d the start location
     * @param endPose2d the end location
     * @return trajectory to get from start to end
     */
    fun pathTo(startPose2d: Pose2d, endPose2d: Pose2d): Trajectory {
        resetTree(startPose2d.translation, endPose2d.translation)
        return treeToTrajectory(startPose2d, endPose2d)
    }

    /**
     * Generate a trajectory to a seperation position
     * @param startPose2d the initial pose of the robot
     * @param endPosition where the robot should end up
     * @return trajectory that takes the robto from startPose2d to endPosition
     */
    fun pathTo(startPose2d: Pose2d, endPosition: Translation2d): Trajectory {
        resetTree(startPose2d.translation, endPosition)
        return treeToTrajectory(startPose2d, endPosition)
    }

    /**
     * Resets the tree to path-find a new route.
     */
    private fun resetTree(startPosition: Translation2d, endPosition: Translation2d) {
        if (tree.nodeCount > 0 && endPosition != information.endPosition) {
            reset()
            tree.pruneInformed()
        }
        loadTree(startPosition, endPosition)
    }

    /**
     * Updates a trajectory when obstacles move
     * @param trajectory the old trajectory that may need correction
     * @return a trajectory that won't collide with any of the updated obstacles
     */
    fun updateTrajectory(trajectory: Trajectory, currentPose: Pose2d): Trajectory {
        tree.pruneBlocked()
        if (tree.vertices.contains(endNode!!)) // this is not optimal if moving obstacles
            return trajectory
        return pathTo(trajectory.states.last().poseMeters, currentPose)
    }

    /**
     * Converts a navigation tree into a path for the robot to follow
     * @return a trajectory that follows the Tree recommended path
     */
    private fun treeToTrajectory(startPose2d: Pose2d, endPose2d: Pose2d): Trajectory {
        if (!pathFound)
            return KTrajectory("default", startPose2d, emptyList(), endPose2d)
        val smooth = smoothPath()
        smooth.removeFirst()
        smooth.removeLast()
        return KTrajectory("Pathfinder path", startPose2d, smooth, endPose2d)  // test edit
    }

    private fun treeToTrajectory(startPose2d: Pose2d, endPosition: Translation2d): Trajectory {
        if (!pathFound)
            return KTrajectory("default", startPose2d, emptyList(), Pose2d(endPosition, startPose2d.translation.towards(endPosition)))
        val smooth = smoothPath()
        smooth.removeFirst()
        smooth.removeLast()
        val endRotation = if(smooth.isEmpty()) startPose2d.translation.towards(endPosition) else smooth.last().towards(endPosition)
        return KTrajectory("Pathfinder path", startPose2d, smooth, Pose2d(endPosition, endRotation))  // test edit
    }

    /**
     * Removes unnecesary nodes from path so that the robot can more smoothly interpolate between fewer points
     */
    private fun smoothPath(): ArrayList<Translation2d> {
        val points = path!!.map { it.position }.reversed()
        var improvement = true
        val newPoints = ArrayList<Translation2d>()
        newPoints.add(points.first())
        var firstIndex = 0
        while (improvement) {
            improvement = false
            for (i in (firstIndex+1 until points.size).reversed()) {
                if (i > firstIndex && KField2d.inField(points[i], newPoints.last())) {
                    newPoints.add(points[i])
                    firstIndex = i
                    improvement = true
                }
            }
        }
        return newPoints
    }

    /**
     * Creates the initial tree of nodes
     */
    private fun loadTree(startPosition: Translation2d, endPosition: Translation2d) {  // to allow dynamic movement, maybe startPoint = goal and end is robot
        // look @ BIT*
        // current version is Informed RRT*
        pathFound = false
        if (tree.nodeCount == 0)
            tree.addNode(Node(startPosition))
        information = PathingInformation(startPosition, endPosition)
        for (i in tree.nodeCount..explorationDepth) {
            if (information.pathFound) break
            val point = randomPoint()
            addPoint(point)
        }
        if (information.pathFound)
            for (i in tree.vertices.count { it.informed }..optimizationDepth) {
                val informed = informedPoint()
                addPoint(informed)
            }
        else println("path not found")
    }

    /**
     * Branches tree towards a point
     */
    private fun addPoint(point: Translation2d) {
        val nearest = tree.nearestNode(point)!!  // asserts not null
        var delta = point.minus(nearest.position)
        val magnitude = delta.getDistance(Translation2d(0.0, 0.0))
        if (magnitude > tree.maxBranchLength) {
            delta = delta.times(tree.maxBranchLength/magnitude)  // resize the vector
        }
        val new = nearest.position.plus(delta)
        if (!KField2d.inField(new, nearest.position)) {
            return
        }
        val node = Node(new, parent = nearest, informed = pathFound)
        tree.addNode(node)
        tree.optimize(node)
        val endDis = new.getDistance(information.endPosition)
        if (endDis < minGoalDistance && !(pathFound && endDis < path!!.first().pathLengthFromRoot)) {
            pathFound = true
            endNode = node
            information.update(path!!.first().pathLengthFromRoot)
        }
    }

    /**
     * Generates random point in the field
     * @return a valid position in the field
     */
    internal fun randomPoint(): Translation2d {
        var x: Double
        var y: Double
        do {
            x = random.nextDouble(KField2d.width.meters)
            y = random.nextDouble(KField2d.width.meters)
        } while(!KField2d.inField(x, y))  // the convertions here might cause issues
        return Translation2d(x, y)
    }

    /**
     * Modified random sample that chooses from inside an oval.
     * This is done because once a rough path is found, no nodes outside the oval can improve the path
     */
    private fun informedPoint(): Translation2d {
        val theta = random.nextDouble(2 * Math.PI)
        val rho = sqrt(random.nextDouble(1.0))
        return information.get(rho, theta)
    }

    /**
     * Illustrate to tree of values
     */
    internal fun drawTreePath() {
        TreeIllustration(this).appear()
    }

    /**
     * Resets the pathing tree
     */
    private fun reset() {
        tree.vertices.clear()
        path?.clear()
    }

    override val priority = DebugLevel.LowPriority

    override fun debugValues(): Map<String, Any?> {
        val map = mutableMapOf<String, Any?>(
            "explored nodes" to tree.nodeCount,
            "path" to path.toString()
        )
        if (this::information.isInitialized) {
            map.putAll(information.debugValues())
        }
        return map.toMap()
    }
}


/**
 * A way to test pathfinding functionality
 */
object PathingTest {
    lateinit var start: Translation2d
    lateinit var end: Translation2d

    @JvmStatic
    fun main(args: Array<String>) {
        KTrajectory.generalConfig = KTrajectoryConfig(2.metersPerSecond, 1.metersPerSecond)
        // look @ BIT*
        // current version is informed RRT*
        start = Translation2d(0.meters, 0.meters)
        end = Translation2d(2.meters, 2.meters)
        for (i in 0..5) {
            val p = Pathfinder.randomPoint()
            val o = Obstacle(Pose2d(p, 0.degrees), 0.2, 0.2)
            if (o.contains(start) || o.contains(end)) continue
           KField2d.obstacles.add(o)
        }
        Pathfinder.pathTo(Pose2d(start, 0.degrees), Pose2d(end, 0.degrees))
        Pathfinder.drawTreePath()
        println(Pathfinder.path)
    }
}