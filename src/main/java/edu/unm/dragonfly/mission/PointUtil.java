package edu.unm.dragonfly.mission;

import com.esri.arcgisruntime.geometry.Point;
import edu.unm.dragonfly.ProjectedPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * @author John Ericksen
 */
public class PointUtil {

    private static final Random RAND = new Random(System.currentTimeMillis());

    public static List<ProjectedPoint> createRandomPoints(List<Point> boundary, int pointSize, double minAltitude, double maxAltitude) {

        double xmax = Double.NEGATIVE_INFINITY;
        double xmin = Double.POSITIVE_INFINITY;;
        double ymax = Double.NEGATIVE_INFINITY;
        double ymin = Double.POSITIVE_INFINITY;;

        for (Point point : boundary) {
            xmax = Math.max(xmax, point.getX());
            xmin = Math.min(xmin, point.getX());
            ymax = Math.max(ymax, point.getY());
            ymin = Math.min(ymin, point.getY());
        }

        List<ProjectedPoint> points = new ArrayList<>();
        for(int i = 0; i < pointSize;) {
            Point randomPoint = new Point((RAND.nextDouble() * (xmax - xmin)) + xmin,
                    (RAND.nextDouble() * (ymax - ymin)) + ymin,
                    (RAND.nextDouble() * (maxAltitude - minAltitude)) + minAltitude);
            if(inside(randomPoint, boundary)) {
                points.add(new ProjectedPoint(randomPoint));
                i++;
            }
        }

        return points;
    }

    public static boolean inside(Point randomPoint, List<Point> boundaryPoints) {
        for(int i = 0; i < boundaryPoints.size() - 1; i++){
            Point a = boundaryPoints.get(i);
            Point b = boundaryPoints.get(i+1);
            if(!isLeft(a, b, randomPoint)) {
                return false;
            }
        }
        return isLeft(boundaryPoints.get(boundaryPoints.size() - 1), boundaryPoints.get(0), randomPoint);
    }

    public static boolean isLeft(Point a, Point b, Point c) {
        return ((b.getX() - a.getX()) * (c.getY() - a.getY()) - (b.getY() - a.getY()) * (c.getX() - a.getX())) > 0;
    }
}
