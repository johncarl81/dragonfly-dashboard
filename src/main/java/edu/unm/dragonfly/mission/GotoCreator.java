package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepGoto;
import javafx.collections.FXCollections;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class GotoCreator implements MissionStepCreator {

    private final ComboBox<String> droneSelection;
    private final ComboBox<String> waypointSelection;

    public GotoCreator(List<String> drones, List<String> waypoints) {
        this.droneSelection = new ComboBox<>(FXCollections.observableList(drones));
        this.waypointSelection = new ComboBox<>(FXCollections.observableList(waypoints));
    }

    @Override
    public void create(GridPane grid) {

        grid.add(new Label("Drone:"), 1, 2);
        grid.add(droneSelection, 2, 2);
        grid.add(new Label("Waypoint:"), 1, 3);
        grid.add(waypointSelection, 2, 3);
    }

    @Override
    public MissionStep build() {
        return new MissionStepGoto(droneSelection.getSelectionModel().getSelectedItem(), waypointSelection.getSelectionModel().getSelectedItem());
    }
}
