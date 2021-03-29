package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepCurtain;
import javafx.collections.FXCollections;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class CurtainCreator implements MissionStepCreator {

    private final ComboBox<String> droneSelection;
    private final ComboBox<String> waypointStartSelection;
    private final ComboBox<String> waypointEndSelection;
    private TextField altitudeField;
    private TextField stacksField;
    private TextField distanceThreshold;

    public CurtainCreator(List<String> drones, List<String> waypoints) {
        this.droneSelection = new ComboBox<>(FXCollections.observableList(drones));
        this.waypointStartSelection = new ComboBox<>(FXCollections.observableList(waypoints));
        this.waypointEndSelection = new ComboBox<>(FXCollections.observableList(waypoints));
    }

    @Override
    public void create(GridPane grid) {

        stacksField = new TextField();
        altitudeField = new TextField();
        distanceThreshold = new TextField();

        // Set Defaults
        stacksField.setText("1");
        altitudeField.setText("10");
        distanceThreshold.setText("1");

        GridUtil.builder(grid).increment()
                .add("Drone:", droneSelection)
                .add("Start:", waypointStartSelection)
                .add("End:", waypointEndSelection)
                .add("Altitude:", altitudeField)
                .add("Stacks:", stacksField)
                .add("Distance Threshold:", distanceThreshold);
    }

    @Override
    public MissionStep build() {
        return new MissionStepCurtain(
                droneSelection.getSelectionModel().getSelectedItem(),
                waypointStartSelection.getSelectionModel().getSelectedItem(),
                waypointEndSelection.getSelectionModel().getSelectedItem(),
                Float.parseFloat(altitudeField.getText()),
                Integer.parseInt(stacksField.getText()),
                Float.parseFloat(distanceThreshold.getText()));
    }
}
