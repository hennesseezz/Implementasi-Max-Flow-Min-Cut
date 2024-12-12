/**
 * Implementation of the Capacity Scaling algorithm using a DFS as a method of finding augmenting
 * paths.
 *
 * <p>Time Complexity: O(E^2log(U)), where E = num edges, U = max capacity
 *
 * @author William Fiset, william.alexandre.fiset@gmail.com
 */


import static java.lang.Math.max;
import static java.lang.Math.min;

import java.util.List;

public class CapacityScalingSolverAdjacencyList extends NetworkFlowSolverBase {

    private long delta;

    /**
     * Creates an instance of a flow network solver. Use the {@link #addEdge(int, int, int)} method to
     * add edges to the graph.
     *
     * @param n - The number of nodes in the graph including source and sink nodes.
     * @param s - The index of the source node, 0 <= s < n
     * @param t - The index of the sink node, 0 <= t < n, t != s
     */
    public CapacityScalingSolverAdjacencyList(int n, int s, int t) {
        super(n, s, t);
    }

    /**
     * Adds a directed edge (and residual edge) to the flow graph.
     *
     * @param from - The index of the node the directed edge starts at.
     * @param to - The index of the node the directed edge end at.
     * @param capacity - The capacity of the edge.
     */

    public void addEdge(int from, int to, long capacity) {
        super.addEdge(from, to, capacity);
        delta = max(delta, capacity);
    }

    // Performs the Ford-Fulkerson method applying a depth first search as
    // a means of finding an augmenting path.
    @Override
    public void solve() {
        // Start delta at the largest power of 2 <= the largest capacity.
        // Equivalent of: delta = (long) pow(2, (int)floor(log(delta)/log(2)))
        delta = Long.highestOneBit(delta);

        // Repeatedly find augmenting paths from source to sink using only edges
        // with a remaining capacity >= delta. Half delta every time we become unable
        // to find an augmenting path from source to sink until the graph is saturated.
        for (long f = 0; delta > 0; delta /= 2) {
            do {
                markAllNodesAsUnvisited();
                f = dfs(s, INF);
                maxFlow += f;
            } while (f != 0);
        }

        // Find min cut.
        for (int i = 0; i < n; i++) if (visited(i)) minCut[i] = true;
    }

    private long dfs(int node, long flow) {
        // At sink node, return augmented path flow.
        if (node == t) return flow;

        List<Edge> edges = graph[node];
        visit(node);

        for (Edge edge : edges) {
            long cap = edge.remainingCapacity();
            if (cap >= delta && !visited(edge.to)) {

                long bottleNeck = dfs(edge.to, min(flow, cap));

                // Augment flow with bottle neck value
                if (bottleNeck > 0) {
                    edge.augment(bottleNeck);
                    return bottleNeck;
                }
            }
        }
        return 0;
    }

    /* Example */

    public static void main(String[] args) {
        WaterDistributionNetwork();
    }

    // Testing graph from:
    // http://crypto.cs.mcgill.ca/~crepeau/COMP251/KeyNoteSlides/07demo-maxflowCS-C.pdf
    private static void WaterDistributionNetwork() {
        int n = 6; // 1 source, 4 intermediate nodes, 1 sink
        int source = 0; // Stasiun pompa
        int sink = 5;   // Wilayah konsumen

        CapacityScalingSolverAdjacencyList solver;
        solver = new CapacityScalingSolverAdjacencyList(n, source, sink);

        // Tambahkan pipa (edges) dari stasiun pompa ke tangki penampungan
        solver.addEdge(source, 1, 15); // Stasiun ke tangki A
        solver.addEdge(source, 2, 10); // Stasiun ke tangki B

        // Tambahkan pipa antara tangki penampungan
        solver.addEdge(1, 2, 5); // Tangki A ke tangki B
        solver.addEdge(1, 3, 10); // Tangki A ke tangki C
        solver.addEdge(2, 4, 15); // Tangki B ke tangki D
        solver.addEdge(3, 4, 10); // Tangki C ke tangki D

        // Tambahkan pipa dari tangki penampungan ke wilayah konsumen
        solver.addEdge(3, sink, 10); // Tangki C ke Wilayah Konsumen
        solver.addEdge(4, sink, 10); // Tangki D ke Wilayah Konsumen

        // Hitung aliran maksimum
        System.out.println("Maximum water flow: " + solver.getMaxFlow());

        List<Edge>[] g = solver.getGraph();
        for (List<Edge> edges : g) {
            for (Edge e : edges) {
                System.out.println(e.toString(source, sink));
                // System.out.println(e.residual.toString(s, t));
            }
        }
    }
}
