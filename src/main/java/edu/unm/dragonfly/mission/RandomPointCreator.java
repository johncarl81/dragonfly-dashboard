package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.ProjectedPoint;
import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepNavigation;
import edu.unm.dragonfly.tsp.TSP;
import io.reactivex.Observable;
import io.reactivex.schedulers.Schedulers;
import javafx.collections.FXCollections;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.stream.Collectors;

/**
 * @author John Ericksen
 */
public class RandomPointCreator implements MissionStepCreator {

    private final Map<String, List<Waypoint>> boundaries;
    private final ComboBox<String> droneSelection;
    private final ComboBox<String> boundarySelection;
    private TextField minAltitude;
    private TextField maxAltitude;
    private TextField size;
    private TextField waitTimeField;
    private TextField distanceThreshold;

    public RandomPointCreator(List<String> drones, Map<String, List<Waypoint>> boundaries) {
        this.droneSelection = new ComboBox<>(FXCollections.observableList(drones));
        this.boundarySelection = new ComboBox<>(FXCollections.observableList(boundaries.keySet().stream().sorted().collect(Collectors.toList())));
        this.boundaries = boundaries;
    }

    @Override
    public void create(GridPane grid) {

        minAltitude = new TextField();
        maxAltitude = new TextField();
        size = new TextField();
        waitTimeField = new TextField();
        distanceThreshold = new TextField();

        // Set Defaults
        minAltitude.setText("10");
        maxAltitude.setText("20");
        size.setText("100");
        waitTimeField.setText("3");
        distanceThreshold.setText("1");

        GridUtil.builder(grid).increment()
                .add("Drone:", droneSelection)
                .add("Boundary:", boundarySelection)
                .add("Min Altitude:", minAltitude)
                .add("Max Altitude:", maxAltitude)
                .add("Size:", size)
                .add("Wait Time:", waitTimeField)
                .add("Distance Threshold:", distanceThreshold);
    }



    @Override
    public MissionStep build() {

        List<Waypoint> waypoints = Observable.fromCallable(new Callable<List<ProjectedPoint>>() {
            @Override
            public List<ProjectedPoint> call() {
                List<ProjectedPoint> randomPoints = PointUtil.createRandomPoints(
                        boundaries.get(boundarySelection.getValue()).stream().map(Waypoint::toPoint).collect(Collectors.toList()),
                        Integer.parseInt(size.getText()),
                        Double.parseDouble(minAltitude.getText()),
                        Double.parseDouble(maxAltitude.getText()));
                return TSP.optimize(randomPoints);
            }
        })
                .subscribeOn(Schedulers.computation())
                .map(tour -> tour.stream().map(ProjectedPoint::toWaypoint).collect(Collectors.toList()))
                .blockingFirst();

        System.out.println("Waypoint size: " + waypoints.size());

        return new MissionStepNavigation(droneSelection.getSelectionModel().getSelectedItem(),
                waypoints,
                Float.parseFloat(waitTimeField.getText()),
                Float.parseFloat(distanceThreshold.getText()));
    }

}
