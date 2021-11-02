package kyberlib.auto.pathing


import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Translation2d

// https://github.com/ReadyPlayer2/TSP
typealias Waypoint = Translation2d
typealias Route = ArrayList<Waypoint>
// todo: add more algorithms just for fun

/**
 * Implementation of various Traveling Saleman Problem Solution.
 * This is about finding the optimal path that goes to a list of points
 */
class TravelingSalesman(val waypoints: MutableList<Translation2d>) {
    constructor(vararg waypoints: Translation2d) : this(waypoints.toMutableList())
    /**
     * Calculate the shortest route by trying every combination O(n!)
     */
    fun bruteForce(): Route {
        // Calculate
        permute(isBruteForce =  true)
        return findShortestPermutation(permutations)
    }
    /**
     * Calculates shortest route using nearest neighbour algorithm
     */
    fun nearestNeighbour(): Route {
        // New route with start as Stoke
        val availableCities = this.waypoints.toMutableList()
        val route = Route().apply { add(this@TravelingSalesman.waypoints.last()) }
        availableCities.remove(route[0])
        while (route.size < this.waypoints.size) {
            val currentCity = route.last()
            val nearestCity = availableCities.minByOrNull { it.getDistance(currentCity) }
            if (nearestCity != null) {
                // Update current location, add to route, set current as visited
                route.add(nearestCity)
                availableCities.remove(nearestCity)
            } else break
        }
        return route
    }
    /**
     * Calculates the shortest route using branch and bound algorithm
     */
    fun branchAndBound(): Route {
        // Calculate
        permute()
        return findShortestPermutation(permutations)
    }

    private var permutationMin = Double.MAX_VALUE
    private val permutations = ArrayList<Route>()
    /**
     * Generates all permutations in lexicographic order
     *
     * @param r: optional starting route, defaults to empty
     * @param isBruteForce: whether to try all options or only better options, defaults to false
     */
    private fun permute(r: Route = Route(), isBruteForce: Boolean = false) {
        if (r.size != waypoints.size) {
            for (city in waypoints) {
                if (r.contains(city)) continue
                // copy
                val newRoute = r.toMutableList() as Route

                // Add the first city from notVisited to the route
                newRoute.add(city)
                if (isBruteForce) {
                    // Recursive call
                    permute(newRoute, isBruteForce)
                }
                else {
                    // If a complete route has not yet been created keep permuting
                    if (permutations.isEmpty()) {
                        // Recursive call
                        permute(newRoute, isBruteForce)
                    }
                    else if (cost(newRoute) < permutationMin) {
                        // Current route cost is less than the best so far so keep permuting
                        permute(newRoute, isBruteForce)
                    }
                }
            }
        }
        else {
            permutations.add(r)
            if (!isBruteForce && cost(r) < permutationMin) {
                permutationMin = cost(r)
            }
        }
    }

    /**
     * find the shortest route in the list
     *
     * @param routeList: list of routes to Search
     */
    private fun findShortestPermutation(routeList: Collection<Route>): Route {
        // Loop through all the permutations
        return routeList.minByOrNull { cost(it) }!!
    }

    /**
     * Find the length of the route
     */
    private fun cost(route: Route): Double {
        var sum = 0.0
        for (i in 0 until route.size-1) {
            sum += route[i].getDistance(route[i+1])
        }
        return sum
    }
}


/**
 * Class to test various parts of the TravellingSalesman class
 */
internal object TravellingSalesmanTest {
    /**
     * Entry point
     */
    @JvmStatic
    fun main(args: Array<String>) {
        // Used to determine number of times the three algorithms should run
        val numIterations = 1
        val alg = TravelingSalesman(
            Translation2d(1.0, 1.0),
            Translation2d(4.0, 1.0),
            Translation2d(2.0, 4.0),
            Translation2d(1.0, 5.0),
            Translation2d(1.0, 10.0)
        )

        // Only individual algorithms should be run during profiling
        val timer = Timer()
        for (i in 0 until numIterations) {
            timer.start()
            // Run brute force
            alg.bruteForce()
            println("Brute Force: ${timer.get()} ms")
            timer.reset()
            // Run nearest neighbour
            alg.nearestNeighbour()
            println("NN: ${timer.get()} ms")
            timer.reset()
            // Run branch and bound
            alg.branchAndBound()
            println("B&B: ${timer.get()} ms")
            timer.stop()
        }
        // Output rough memory usage (profiler is more accurate)
        println(
            "KB: " + (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()).toDouble() / 1024
        )
    }
}