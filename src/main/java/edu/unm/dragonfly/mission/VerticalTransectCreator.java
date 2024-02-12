package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.*;
import javafx.collections.*;
import javafx.scene.control.*;
import javafx.scene.layout.*;

import java.util.*;

/**
 * @author John Ericksen
 */
public class VerticalTransectCreator implements MissionStepCreator {

    private final ComboBox<String> droneSelection;
    private final ComboBox<String> waypointSelection;
    private TextField minAltitudeField;
    private TextField maxAltitudeField;

    public VerticalTransectCreator(List<String> drones, List<String> waypoints) {
        this.droneSelection = new ComboBox<>(FXCollections.observableList(drones));
        this.waypointSelection = new ComboBox<>(FXCollections.observableList(waypoints));
    }

    @Override
    public void create(GridPane grid) {
        minAltitudeField = new TextField();
        minAltitudeField.setText("30");
        maxAltitudeField = new TextField();
        maxAltitudeField.setText("100");

        GridUtil.builder(grid).increment()
                .add("Drone:", droneSelection)
                .add("Waypoint:", waypointSelection)
                .add("Min Alt:", minAltitudeField)
                .add("Max Alt:", maxAltitudeField);
    }

    @Override
    public MissionStep build() {
        return new MissionStepVerticalTransect(droneSelection.getSelectionModel().getSelectedItem(),
                waypointSelection.getSelectionModel().getSelectedItem(),
                Double.parseDouble(minAltitudeField.getText()),
                Double.parseDouble(maxAltitudeField.getText()));
    }
}
