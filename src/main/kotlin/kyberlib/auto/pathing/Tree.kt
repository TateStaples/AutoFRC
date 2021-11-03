package kyberlib.auto.pathing

import edu.wpi.first.wpilibj.geometry.Translation2d
import kyberlib.simulation.field.KField2d
import java.util.function.Predicate

/**
 * A node in the RRT tree
 * @author TateStaples
 */
class Node {
    var position: Translation2d
    var informed = false
    var pathLengthFromRoot: Double
        set(value) {
            field = value
            for (child in children)
                child.pathLengthFromRoot = pathLengthFromRoot + child.distance
        }
    private var nodeLengthFromRoot: Int
    val children = ArrayList<Node>()
    private var distance = 0.0

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
                distance = position.getDistance(value.position)
                pathLengthFromRoot = value.pathLengthFromRoot + distance
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
        distance = 0.0
        informed = false
    }

    /**
     * This is the standard constructor for Nodes.
     * @param position this is the location of the node
     * @param parent this is the node that connects this node back towards the root
     */
    constructor(position: Translation2d, parent: Node, informed: Boolean = false) {
        this.position = position
        pathLengthFromRoot = -10.0
        nodeLengthFromRoot = -10
        this.parent = parent // this should set the pathLengthFromRoot
        this.informed = informed
    }

    fun breakBranch() {
        for (child in children) {
            child.parent = null
            child.breakBranch()
        }
        children.clear()
    }

    override fun toString(): String {
        return "node($position)"
//        return hashCode().toString()
    }
}

/**
 * A tree class to represent to points and connections of the RRT pathfinder
 * @author TateStaples
 */
class Tree(private val field: KField2d) {
    val maxBranchLength = 0.5
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
        val nearby = nearNodes(n.position)
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
    private fun optimize() {
        for (node in vertices) optimize(node)
    }

    /**
     * Returns all nodes in the tree within a certain range
     * @param point the location around which to search for nodes
     * @return ArrayList of Nodes that are with "range" of "point"
     */
    private fun nearNodes(point: Translation2d): ArrayList<Node> {
        val nodes = ArrayList<Node>()
        for (node in vertices) {
            if (node.position.getDistance(point) < maxBranchLength && field.inField(point, node.position))
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
        prune { field.inField(it.position)}
    }

    /**
     * Removes node and nodes children from branch
     */
    private fun prune(condition: Predicate<Node>) {
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
                val nearby = nearNodes(orphan.position)
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

    val nodeCount: Int
        get() = vertices.size
}
