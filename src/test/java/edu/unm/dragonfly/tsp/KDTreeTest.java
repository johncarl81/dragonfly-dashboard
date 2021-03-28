package edu.unm.dragonfly.tsp;

import com.esri.arcgisruntime.geometry.Point;
import edu.unm.dragonfly.ProjectedPoint;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;

/**
 * @author John Ericksen
 */
public class KDTreeTest {

    @Test
    public void testTreeLifecycle() {
        KDTree tree = new KDTree();

        List<ProjectedPoint> points = new ArrayList<>();

        points.add(new ProjectedPoint(new Point(0, 0)));
        points.add(new ProjectedPoint(new Point(0, 1)));
        points.add(new ProjectedPoint(new Point(0, 2)));
        points.add(new ProjectedPoint(new Point(2, 3)));
        points.add(new ProjectedPoint(new Point(-1, -1)));

        for(ProjectedPoint point : points) {
            tree.insert(point);
        }

        for(ProjectedPoint point : points) {
            assertNearest(tree, point, point);
        }

        assertNearest(tree, new ProjectedPoint(new Point(0, 0.6)), points.get(1));
        assertNearest(tree, new ProjectedPoint(new Point(-0.9, -0.5)), points.get(4));

        tree.remove(points.get(1));

        assertNearest(tree, new ProjectedPoint(new Point(0, 1)), points.get(2));
    }

    void assertNearest(KDTree tree, ProjectedPoint input, ProjectedPoint nearest) {
        KDTree.PointDistance output = tree.nearest(input);
        assertEquals(nearest, output.getPoint());
    }
}
