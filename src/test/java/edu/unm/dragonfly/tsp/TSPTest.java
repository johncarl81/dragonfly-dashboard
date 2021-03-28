package edu.unm.dragonfly.tsp;

import com.esri.arcgisruntime.geometry.Point;
import edu.unm.dragonfly.ProjectedPoint;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;

/**
 * @author John Ericksen
 */
public class TSPTest {

    private static final Random RAND = new Random(0);

    @Test
    public void testEmptyTSP() {
        TSP.optimize(Collections.emptyList());
    }

    @Test
    public void testSmallTSP() {
        TSP.optimize(Collections.singletonList(new ProjectedPoint(new Point(0, 0))));
    }


    @Test
    public void testTSPLifecycle() {

        List<ProjectedPoint> points = new ArrayList<>();

        points.add(new ProjectedPoint(new Point(12, 30)));
        points.add(new ProjectedPoint(new Point(44, 15)));
        points.add(new ProjectedPoint(new Point(32, 28)));
        points.add(new ProjectedPoint(new Point(95, 80)));

        List<ProjectedPoint> output = TSP.optimize(points);

        assertEquals(points.size(), output.size());
    }

    @Test
    public void testRandomTSP() {
        List<ProjectedPoint> points = new ArrayList<>();

        int size = 1000;
        for(int i = 0; i < size; i++) {
            points.add(new ProjectedPoint(new Point(RAND.nextDouble() * 6, RAND.nextDouble() * 6)));
        }
        List<ProjectedPoint> output = TSP.optimize(points);

        assertEquals(points.size(), output.size());
    }
}
