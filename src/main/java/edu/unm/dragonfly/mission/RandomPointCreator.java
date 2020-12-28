package edu.unm.dragonfly.mission;

import com.esri.arcgisruntime.geometry.Point;
import edu.unm.dragonfly.GeneticTSP;
import edu.unm.dragonfly.ProjectedPoint;
import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepNavigation;
import io.reactivex.Observable;
import io.reactivex.schedulers.Schedulers;
import javafx.collections.FXCollections;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.Callable;
import java.util.stream.Collectors;

/**
 * @author John Ericksen
 */
public class RandomPointCreator implements MissionStepCreator {

    private static final Random RAND = new Random(System.currentTimeMillis());

    private final List<Point> boundaryPoints;
    private final ComboBox<String> droneSelection;
    private TextField minAltitude;
    private TextField maxAltitude;
    private TextField size;
    private TextField iterations;
    private TextField population;
    private TextField waitTimeField;
    private TextField distanceThreshold;
    private ProgressBar progressBar;

    public RandomPointCreator(List<String> drones, List<Point> boundaryPoints) {
        this.droneSelection = new ComboBox<>(FXCollections.observableList(drones));
        this.boundaryPoints = boundaryPoints;
    }

    @Override
    public void create(GridPane grid) {

        minAltitude = new TextField();
        maxAltitude = new TextField();
        size = new TextField();
        iterations = new TextField();
        population = new TextField();
        waitTimeField = new TextField();
        distanceThreshold = new TextField();
        progressBar = new ProgressBar(0);

        // Set Defaults
        minAltitude.setText("10");
        maxAltitude.setText("20");
        size.setText("100");
        iterations.setText("100");
        population.setText("100");
        waitTimeField.setText("3");
        distanceThreshold.setText("1");


        grid.add(new Label("Drone:"), 1, 2);
        grid.add(droneSelection, 2, 2);
        grid.add(new Label("Min Altitude: "), 1, 3);
        grid.add(minAltitude, 2, 3);
        grid.add(new Label("Max Altitude: "), 1, 4);
        grid.add(maxAltitude, 2, 4);
        grid.add(new Label("Size: "), 1, 5);
        grid.add(size, 2, 5);
        grid.add(new Label("Iterations: "), 1, 6);
        grid.add(iterations, 2, 6);
        grid.add(new Label("Population: "), 1, 7);
        grid.add(population, 2, 7);
        grid.add(new Label("Wait Time: "), 1, 8);
        grid.add(waitTimeField, 2, 8);
        grid.add(new Label("Distance Threshold: "), 1, 9);
        grid.add(distanceThreshold, 2, 9);
        grid.add(progressBar, 1, 10, 2, 10);

    }



    @Override
    public MissionStep build() {

        List<Waypoint> waypoints = Observable.fromCallable(new Callable<GeneticTSP.Tour<ProjectedPoint>>() {
            @Override
            public GeneticTSP.Tour<ProjectedPoint> call() {
                double xmax = Double.NEGATIVE_INFINITY;
                double xmin = Double.POSITIVE_INFINITY;;
                double ymax = Double.NEGATIVE_INFINITY;
                double ymin = Double.POSITIVE_INFINITY;;

                for (Point point : boundaryPoints) {
                    if(xmax < point.getX()) {
                        xmax = point.getX();
                    }
                    if(xmin > point.getX()) {
                        xmin = point.getX();
                    }
                    if(ymax < point.getY()) {
                        ymax = point.getY();
                    }
                    if(ymin > point.getY()) {
                        ymin = point.getY();
                    }
                }

                List<ProjectedPoint> points = new ArrayList<>();
                int pointSize = Integer.parseInt(size.getText());
                double minAltitudeValue = Double.parseDouble(minAltitude.getText());
                double maxAltitudeValue = Double.parseDouble(maxAltitude.getText());
                for(int i = 0; i < pointSize;) {
                    Point randomPoint = new Point((RAND.nextDouble() * (xmax - xmin)) + xmin,
                            (RAND.nextDouble() * (ymax - ymin)) + ymin,
                            (RAND.nextDouble() * (maxAltitudeValue - minAltitudeValue)) + minAltitudeValue);
                    if(inside(randomPoint, boundaryPoints)) {
                        points.add(new ProjectedPoint(randomPoint));
                        i++;
                    }
                }

                GeneticTSP.Population<ProjectedPoint> chromosomes = GeneticTSP.Population.generate(points, new GeneticTSP.DistanceMetric<ProjectedPoint>() {
                    @Override
                    public double distance(List<ProjectedPoint> points) {
                        double distance = 0;
                        for(int i = 0; i < points.size() - 1; i++) {
                            double deltax = points.get(i).getX() - points.get(i + 1).getX();
                            double deltay = points.get(i).getY() - points.get(i + 1).getY();
                            double deltaz = points.get(i).getZ() - points.get(i + 1).getZ();
                            distance += Math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz));
                        }
                        return distance;
                    }
                }, Integer.parseInt(population.getText()));

                int iterationsValue = Integer.parseInt(iterations.getText());
                for(int i = 0; i < iterationsValue; i++) {
                    long start = System.currentTimeMillis();
                    chromosomes = GeneticTSP.evolve(chromosomes);

                    System.out.println("Evolution " + i + " " +
                            "took: " + (System.currentTimeMillis() - start) + "ms, " +
                            "distance: " + chromosomes.getMostFit().getDistance());
                }

                return chromosomes.getMostFit();
            }
        })
                .subscribeOn(Schedulers.computation())
                .map(tour -> tour.getPoints().stream().map(ProjectedPoint::toWaypoint).collect(Collectors.toList()))
                .blockingFirst();

        System.out.println("Waypoint size: " + waypoints.size());

        return new MissionStepNavigation(droneSelection.getSelectionModel().getSelectedItem(),
                waypoints,
                Float.parseFloat(waitTimeField.getText()),
                Float.parseFloat(distanceThreshold.getText()));
    }

    private boolean inside(Point randomPoint, List<Point> boundaryPoints) {
        for(int i = 0; i < boundaryPoints.size() - 1; i++){
            Point a = boundaryPoints.get(i);
            Point b = boundaryPoints.get(i+1);
            if(!isLeft(a, b, randomPoint)) {
                return false;
            }
        }
        return isLeft(boundaryPoints.get(boundaryPoints.size() - 1), boundaryPoints.get(0), randomPoint);
    }



    private boolean isLeft(Point a, Point b, Point c) {
        return ((b.getX() - a.getX()) * (c.getY() - a.getY()) - (b.getY() - a.getY()) * (c.getX() - a.getX())) > 0;
    }

}
