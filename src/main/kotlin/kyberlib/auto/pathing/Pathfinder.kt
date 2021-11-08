package kyberlib.auto.pathing

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import kyberlib.auto.trajectory.KTrajectory
import kyberlib.auto.trajectory.KTrajectoryConfig
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
 * @param field KField2d that contains the dimensions of the field and is updated with the obstacles
 */
open class Pathfinder {
    val field = KField2d
    internal val tree = Tree()
    private val random = Random(6)
    internal lateinit var information: PathingInformation

    var minGoalDistance = 0.2.feet.value  // margin of error for pathfinding node
    var pathFound = false  // whether the Planner currently has a working path
    var endNode: Node? = null  // the node the represents the end goal [robot position] (think about changing to growing 2 seperate trees)
    val path: ArrayList<Node>?   // the working path of points to get from robot position to target goal
        get() = endNode?.let { tree.trace(it) }

    /** how many nodes to create before giving up finding target */
    private val explorationDepth = 5000
    /** how many nodes to dedicate to optimization */
    private val optimizationDepth = 100

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

    fun pathTo(startPose2d: Pose2d, endPosition: Translation2d): Trajectory {
        resetTree(startPose2d.translation, endPosition)
        return treeToTrajectory(startPose2d, endPosition)
    }

    private fun resetTree(startPosition: Translation2d, endPosition: Translation2d) {
        if (tree.nodeCount > 0 && endPosition != information.endPosition) {
            println("resetting tree")
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
        if (!pathFound) return Trajectory()
        val smooth = smoothPath()
        smooth.removeFirst()
        smooth.removeLast()
//        println("start: $startPose2d, through: $smooth, end: $endPose2d")
        return KTrajectory("Pathfinder path", startPose2d, smooth, endPose2d)  // test edit
    }

    private fun treeToTrajectory(startPose2d: Pose2d, endPosition: Translation2d): Trajectory {
        if (!pathFound) return Trajectory()
        val smooth = smoothPath()
        smooth.removeFirst()
        smooth.removeLast()
//        println("start: $startPose2d, through: $smooth, end: $endPose2d")
        val endRotation = if(smooth.isEmpty()) startPose2d.translation.towards(endPosition) else smooth.last().towards(endPosition)
        return KTrajectory("Pathfinder path", startPose2d, smooth, Pose2d(endPosition, endRotation))  // test edit
    }

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
        if (!field.inField(new, nearest.position)) {
            return
        }
        val node = Node(new, parent = nearest, informed = pathFound)
        tree.addNode(node)
        tree.optimize(node)
        val endDis = new.getDistance(information.endPosition)
        if (endDis < minGoalDistance && !(pathFound && endDis < path!!.first().pathLengthFromRoot)) {
            pathFound = true
            endNode = node
            println("path found = $path")
            information.update(path!!.last().pathLengthFromRoot)
        }
    }

    /**
     * Generates random point in the field
     * @return a valid position in the field
     */
    fun randomPoint(): Translation2d {
        var x: Double
        var y: Double
        do {
            x = random.nextDouble(field.width.meters)
            y = random.nextDouble(field.width.meters)
        } while(!field.inField(x, y))  // the convertions here might cause issues
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
    fun drawTreePath() {
        TreeIllustration(this).appear()
    }

    /**
     * Resets the pathing tree
     */
    private fun reset() {
        tree.vertices.clear()
        path?.clear()
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
        val PathPlanner = Pathfinder()
        // look @ informed RRT* and BIT*
        // current version in RRT
        start = Translation2d(0.meters, 0.meters)
        end = Translation2d(2.meters, 2.meters)
        for (i in 0..0) {
            val p = PathPlanner.randomPoint()
            val o = Obstacle(Pose2d(p, 0.degrees), 0.2, 0.2)
            if (o.contains(start) || o.contains(end)) continue
            PathPlanner.field.obstacles.add(o)
        }
//        val testO = Obstacle(Pose2d(3.feet, 5.feet, 0.degrees), 1.feet.meters, 1.feet.meters)
//        PathPlanner.field.obstacles.add(testO)
        println("field setup")
        PathPlanner.pathTo(Pose2d(start, 0.degrees), Pose2d(end, 0.degrees))
        println("tree loaded")
//        println(PathPlanner.path!!.size)
        PathPlanner.drawTreePath()
        println(PathPlanner.path)
        println(PathPlanner.path!!.map { PathPlanner.field.inField(it.position)})
        println(PathPlanner.field.obstacles)
    }
}