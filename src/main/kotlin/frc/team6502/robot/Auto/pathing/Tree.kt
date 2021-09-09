package frc.team6502.robot.Auto

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.kyberlib.math.units.extensions.radians
import frc.team6502.robot.Auto.pathing.PathPlanner
import java.awt.Color
import java.awt.Graphics
import java.awt.Graphics2D
import java.awt.geom.Ellipse2D
import java.util.function.Predicate
import javax.swing.JFrame
import javax.swing.JPanel
import kotlin.math.atan
import kotlin.math.sqrt
import kotlin.random.Random


/**
 * A node in the RRT tree
 * @author TateStaples
 */
class Node {
    var position: Translation2d
    var informed = false
    var pathLengthFromRoot: Double
    var nodeLengthFromRoot: Int
    val children = ArrayList<Node>()
    var parent: Node? = null
        /**
         * Should update the previous parent and change the pathLength to root
         */
        set(value) {
            if (parent != null) {
                parent!!.children.remove(this)
            }
            if (value != null && nodeLengthFromRoot != 0) {
                value.children.add(this)
                pathLengthFromRoot = value.pathLengthFromRoot + position.getDistance(value.position)  // TODO this might not be changed when optimizing
                nodeLengthFromRoot = value.nodeLengthFromRoot + 1
            }
            field = value
        }
    val orphaned
        get() = parent == null

    /**
     * This is the constructor for the root node of the tree.
     * This should only ever be used once per tree
     *
     * @param position where the tree root should be
     */
    constructor(position: Translation2d) {
        this.position = position
        nodeLengthFromRoot = 0
        pathLengthFromRoot = 0.0
        informed = false
    }

    /**
     * This is the standard constructor for Nodes.
     * @param position this is the location of the node
     * @param parent this is the node that connects this node back towards the root
     */
    constructor(position: Translation2d, parent: Node) {
        this.position = position
        pathLengthFromRoot = -10.0
        nodeLengthFromRoot = -10
        this.parent = parent // this should set the pathLengthFromRoot
        informed = PathPlanner.pathFound
    }

    fun breakBranch() {
        for (child in children) {
            child.parent = null
            child.breakBranch()
        }
        children.clear()
    }

    override fun toString(): String {
        return "($position) -- ($nodeLengthFromRoot, ${parent?.nodeLengthFromRoot})"
//        return hashCode().toString()
    }
}

/**
 * A tree class to represent to points and connections of the RRT pathfinder
 * @author TateStaples
 */
class Tree {
    /**
     * A way to test tree functionality
     */
    companion object Test {
        @JvmStatic
        fun main(args: Array<String>) {
            // look @ informed RRT* and BIT*
            // current version in RRT
            val obstacle = Obstacle(Pose2d(5.0, 5.0, Rotation2d(0.0)), 1.0, 1.0)
            PathPlanner.field.obstacles.add(obstacle)

            PathPlanner.loadTree(Translation2d(3.0, 3.0), Translation2d(9.0, 9.0))
            PathPlanner.tree.draw()
        }
        val random = Random(1)
        fun randomPoint(): Translation2d {
            var x: Double
            var y: Double
            do {
                x = random.nextDouble(10.0)
                y = random.nextDouble(10.0)
            } while(false)  // the convertions here might cause issues
            return Translation2d(x, y)
        }
    }
    val maxBranchLength = 1.0
    val vertices = ArrayList<Node>()

    /**
     * Adds a given node into the tree
     * @param n the node to be loaded
     */
    fun addNode(n: Node) {
        if (!vertices.contains(n)) {
            vertices.add(n)
        }
    }

    /**
     * Get the node in the tree that in closest to a given point
     * @param point the designated point
     * @return the closest Node to "point" or null if no nodes are in tree
     * @
     */
    fun nearestNode(point: Translation2d): Node? {
        var minDis = Double.POSITIVE_INFINITY
        var bestNode: Node? = null
        var dis: Double
        for (node in vertices) {
            dis = point.getDistance(node.position)
            if (dis < minDis) {
                minDis = dis
                bestNode = node
            }
        }
        return bestNode
    }

    /**
     * Checks to see if there is a way to take a shortcut to root rather than following current parent
     * @param n the node that is looking for a better parent
     */
    fun optimize(n: Node) {
        val nearby = nearNodes(n.position, maxBranchLength)
        var dis: Double
        for (node in nearby) {
            dis = node.position.getDistance(n.position)
            if (node.pathLengthFromRoot + dis < n.pathLengthFromRoot) n.parent = node
            else if (n.pathLengthFromRoot + dis < node.pathLengthFromRoot ) {
                node.parent = n
                optimize(node)  // this really might not work
            }   // this should work

        }
    }

    /**
     * Goes through the whole tree to make sure everything is as efficient as possible
     */
    fun optimize() {
        for (node in vertices) optimize(node)
    }

    /**
     * Returns all nodes in the tree within a certain range
     * @param point the location around which to search for nodes
     * @param range how far around "point" to search for nodes
     * @return ArrayList of Nodes that are with "range" of "point"
     */
    private fun nearNodes(point: Translation2d, range: Double): ArrayList<Node> {
        val nodes = ArrayList<Node>()
        for (node in vertices) {
            if (node.position.getDistance(point) < range)
                nodes.add(node)
        }
        return nodes
    }

    /**
     * Traces back the connection between a given nodes all the way back to the root
     * @param n the node to trace
     * @return ArrayList of all the nodes connecting "n" to the root
     */
    fun trace(n: Node): ArrayList<Node> {
        val nodes = ArrayList<Node>()
        nodes.add(n)
        var node = n
        while(node.parent != null) {
            nodes.add(node.parent!!)
            node = node.parent!!
        }
        return nodes
    }

    /**
     * Removes the nodes created for optimization.
     * This leaves only the breadth nodes
     */
    fun pruneInformed() {
        prune { it.informed }
    }

    /**
     * Prune the tree with updated obstacle information
     */
    fun pruneBlocked() {
        prune { PathPlanner.field.inField(it.position)}
    }

    /**
     * Removes node and nodes children from branch
     */
    fun prune(condition: Predicate<Node>) {
        // remove the offending nodes
        for (node in vertices) {
            if (condition.test(node)) {
                vertices.remove(node)
                node.breakBranch()
            }
        }

        // find which nodes have been cut off by the pruning
        val orphans = vertices.filter { it.orphaned } as ArrayList<Node>
        var adopted = 1

        // find ways to reconfigure the tree
        while (adopted > 0) {
            adopted = 0
            for (orphan in orphans) {
                val nearby = nearNodes(orphan.position, maxBranchLength)
                for (node in nearby) {
                    if (!node.orphaned) {
                        orphan.parent = node
                        adopted += 1
                        orphans.remove(orphan)
                        break
                    }
                }
            }
        }
        // get rid of nodes that can't reconnect to the tree
        vertices.removeAll(orphans)
        optimize()
    }

    private val mult = 40
    /**
     * Creates a TreeDrawing and then displays it
     */
    fun draw() {
        val frame = JFrame()
        frame.setSize(400, 400)
        frame.title = "Tree drawing"
        frame.layout = null
        frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
        val drawing = TreeDrawing(this)
        frame.contentPane = drawing
        frame.isVisible = true
    }

    /**
     * Puts the branches onto a graphics window
     */
    fun draw(graphics: Graphics2D) {
        graphics.color = Color.BLACK
        val mult = 40
        for (obstacle in PathPlanner.field.obstacles) {
            graphics.drawRect((obstacle.x-obstacle.width).toInt() * mult, (obstacle.y-obstacle.height).toInt() * mult, obstacle.width.toInt() * mult*2, obstacle.height.toInt() * mult*2)
        }
        drawBranch(vertices[0], graphics)
        if (PathPlanner.pathFound) drawPathOval(graphics)
    }

    /**
     * Draws the oval for Informed RRT*
     * Any point outside of this oval cannot be useful
     */
    private fun drawPathOval(graphics: Graphics2D) {
        val startPosition = PathPlanner.path!!.last().position
        val endPosition = PathPlanner.path!!.first().position
        val currentPathLength = PathPlanner.path!![0].pathLengthFromRoot
        val dis = startPosition.getDistance(endPosition)
        val center = startPosition.plus(endPosition).div(2.0)  // average
        val width = currentPathLength
        val height = sqrt(currentPathLength * currentPathLength - dis * dis)  // TODO: check this geometry
        val shifted = endPosition.minus(startPosition)
        val rotation = atan(shifted.y / shifted.x).radians

        val oval = Ellipse2D.Double((center.x-width/2) * mult, (center.y-height/2) * mult, width * mult, height * mult)
        graphics.color = Color.BLUE
        graphics.rotate(rotation.radians, center.x * mult, center.y * mult)
        graphics.draw(oval)
    }

    /**
     * Recursively draws each part of the branch
     */
    private fun drawBranch(n: Node, g: Graphics2D) {
        val mult = 40
        val x1 = n.position.x * mult
        val y1 = n.position.y * mult
        for (n2 in n.children) {
            val x2 = n2.position.x * mult
            val y2 = n2.position.y * mult
            if (PathPlanner.pathFound && PathPlanner.path!!.contains(n2))
                g.color = Color.RED
            else g.color =
                Color.BLACK
//            if (PathPlanner.pathFound && PathPlanner.path!!.contains(n2)) g.drawLine(x1.toInt(), y1.toInt(), x2.toInt(), y2.toInt())
            g.drawLine(x1.toInt(), y1.toInt(), x2.toInt(), y2.toInt())
            drawBranch(n2, g)
        }
    }

    val nodeCount: Int
        get() = vertices.size
}

/**
 * Draws a basic Tree without showing the obstacles
 */
class TreeDrawing(private val tree: Tree) : JPanel() { public override fun paintComponent(g: Graphics) { tree.draw(g as Graphics2D) } }


