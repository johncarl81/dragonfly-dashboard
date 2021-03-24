package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepGoto;
import javafx.collections.FXCollections;
import javafx.scene.control.ComboBox;
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

        GridUtil.builder(grid).increment()
                .add("Drone:", droneSelection)
                .add("Waypoint:", waypointSelection);
    }

    @Override
    public MissionStep build() {
        return new MissionStepGoto(droneSelection.getSelectionModel().getSelectedItem(), waypointSelection.getSelectionModel().getSelectedItem());
    }
}
