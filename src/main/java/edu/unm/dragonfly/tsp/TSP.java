package edu.unm.dragonfly.tsp;

import edu.unm.dragonfly.ProjectedPoint;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.Stack;

/**
 * @author John Ericksen
 */
public class TSP {

    private static class Edge implements Comparable<Edge>{

        private final ProjectedPoint start, end;
        private final double distance;

        public Edge(ProjectedPoint one, KDTree.PointDistance pointDistance) {
            this.start = one;
            this.end = pointDistance.getPoint();
            this.distance = pointDistance.getDistance();
        }


        @Override
        public int compareTo(Edge o) {
            return Double.compare(this.distance, o.distance);
        }
    }

    /**
     * Prims algorithm for TSP optimization
     *
     * @param randomPoints
     * @return
     */
    public static List<ProjectedPoint> optimize(List<ProjectedPoint> randomPoints) {

        if(randomPoints.size() < 3) {
            return randomPoints;
        }

        PriorityQueue<Edge> heap = new PriorityQueue<>();
        Set<ProjectedPoint> included = new HashSet<>();
        Map<ProjectedPoint, List<ProjectedPoint>> minimumSpanningTree = new HashMap<>();

        ProjectedPoint first = randomPoints.get(0);

        KDTree nearestTree = new KDTree();
        for(ProjectedPoint point : randomPoints){
            nearestTree.insert(point);
        }

        nearestTree.remove(first);
        included.add(first);
        pushEdge(heap, first, nearestTree);

        while(!heap.isEmpty()) {
            Edge smallestEdge = heap.poll();

            if(!included.contains(smallestEdge.end)) {
                if(!minimumSpanningTree.containsKey(smallestEdge.start)) {
                    minimumSpanningTree.put(smallestEdge.start, new ArrayList<>());
                }
                minimumSpanningTree.get(smallestEdge.start).add(smallestEdge.end);
                included.add(smallestEdge.end);

                nearestTree.remove(smallestEdge.end);

                pushEdge(heap, smallestEdge.start, nearestTree);
                pushEdge(heap, smallestEdge.end, nearestTree);
            } else {
                pushEdge(heap, smallestEdge.start, nearestTree);
            }

        }

        return dft(first, minimumSpanningTree);
    }

    private static List<ProjectedPoint> dft(ProjectedPoint root, Map<ProjectedPoint, List<ProjectedPoint>> minimumSpanningTree) {
        List<ProjectedPoint> result = new ArrayList<>();

        Stack<ProjectedPoint> stack = new Stack<>();
        stack.push(root);

        while(!stack.isEmpty()) {
            ProjectedPoint localRoot = stack.pop();
            result.add(localRoot);
            if(minimumSpanningTree.containsKey(localRoot)) {
                for(ProjectedPoint point : minimumSpanningTree.get(localRoot)) {
                    stack.push(point);
                }
            }
        }

        return result;
    }

    private static void pushEdge(PriorityQueue<Edge> heap, ProjectedPoint point, KDTree nearestTree) {
        KDTree.PointDistance nearest = nearestTree.nearest(point);

        if(nearest.getDistance() != Double.POSITIVE_INFINITY) {
            heap.add(new Edge(point, nearest));
        }
    }
}
