package frc.team6502.robot.Auto

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.team6502.kyberlib.math.units.extensions.feet
import frc.team6502.robot.RobotContainer
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.random.Random

/**
 * Object to manage pathfinding functions.
 * Implements RTT* in order to find the most efficient route
 * @author TateStaples
 */
object PathPlanner {
    val field = KField2d() //RobotContainer.navigation.field
    val tree = Tree()
    val random = Random(3)//Timer.getFPGATimestamp().toInt())

    var minGoalDistance = 0.5.feet.value  // margin of error for pathfinding node
    var pathFound = false  // whether the Planner currently has a working path
    var endNode: Node? = null  // the node the represents the end goal [robot position] (think about changing to growing 2 seperate trees)
    val path: ArrayList<Node>?  // the working path of points to get from robot position to target goal
        get() = endNode?.let { tree.trace(it) }

    val treeDepth = 800  // how many nodes the tree should create

    /**
     * Generates a trajectory to get from current estimated pose to a separate target
     * @param pose2d the pose that you want the robot to get to
     * @return a trajectory that will track your robot to the goal target
     */
    fun pathTo(pose2d: Pose2d): Trajectory {
        if (tree.nodeCount > 0)
            tree.pruneInformed()
        loadTree(pose2d.translation, RobotContainer.navigation.position)
        // TODO: figure out how to maintain rotation information
        return treeToTrajectory()
    }

    /**
     * Updates a trajectory when obstacles move
     * @param trajectory the old trajectory that may need correction
     * @return a trajectory that won't collide with any of the updated obstacles
     */
    fun updateTrajectory(trajectory: Trajectory): Trajectory {
        tree.pruneBlocked()
        if (tree.vertices.contains(endNode!!)) return trajectory
        return pathTo(trajectory.states.last().poseMeters)
    }

    /**
     * Converts a navigation tree into a path for the robot to follow
     * @return a trajectory that follows the Tree recommended path
     */
    private fun treeToTrajectory(): Trajectory {
        val positions = ArrayList<Translation2d>()
        for (node in path!!) positions.add(node.position)
        return RobotContainer.navigation.trajectory(positions)
    }

    /**
     * Creates the initial tree of nodes
     */
    internal fun loadTree(startPosition: Translation2d, endPosition: Translation2d) {  // to allow dynamic movement, maybe startPoint = goal and end is robot
        // look @ BIT*
        // current version is Informed RRT*
        pathFound = false
        tree.addNode(Node(startPosition))  // TODO: this may cause errors when loading tree several times
        for (i in tree.nodeCount..treeDepth) {
            val point = if (pathFound) informedPoint() else randomPoint()
            val nearest = tree.nearestNode(point)!!  // asserts not null
            var delta = point.minus(nearest.position)
            val magnitude = delta.getDistance(Translation2d(0.0, 0.0))
            if (magnitude > tree.maxBranchLength) {
                delta = delta.times(tree.maxBranchLength/magnitude)  // resize the vector
            }
            val new = nearest.position.plus(delta)
            if (!field.inField(new)) {
                continue
            }
            val node = Node(new, nearest)
            tree.addNode(node)
            tree.optimize(node)
            val endDis = new.getDistance(endPosition)
            if (endDis < minGoalDistance && !(pathFound && endDis < path!!.first().pathLengthFromRoot)) {
                pathFound = true
                endNode = node
//                break
            }
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
            x = random.nextDouble(field.width)
            y = random.nextDouble(field.width)
        } while(!field.inField(x, y))  // the convertions here might cause issues
        return Translation2d(x, y)
    }

    /**
     * Modified random sample that chooses from inside an oval.
     * This is done because once a rough path is found, no nodes outside the oval can improve the path
     */
    fun informedPoint(): Translation2d {
        // TODO: store these variables somewhere
        val startPosition = path!!.last().position
        val endPosition = path!!.first().position
        val center = startPosition.plus(endPosition).div(2.0)  // average
        val currentPathLength = path!![0].pathLengthFromRoot
        val dis = startPosition.getDistance(endPosition)
        val width = currentPathLength
        val height = sqrt(currentPathLength * currentPathLength - dis * dis)
        val shifted = endPosition.minus(startPosition)
        val rotation = Rotation2d(shifted.x, shifted.y)

        val theta = random.nextDouble(2*Math.PI)
        val rho = random.nextDouble(1.0)
        val x = cos(theta) * width/2 * rho
        val y = sin(theta) * height/2 * rho
        val point = Translation2d(x, y).rotateBy(rotation).plus(center)
        return point
    }

    /**
     * Illustrate to tree of values
     */
    private fun drawTreePath() {
        // TODO: update with field stuff and smartdashboard
        tree.draw()
    }

}