package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStart;
import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepFlock;
import edu.unm.dragonfly.mission.step.MissionStepGoto;
import javafx.collections.FXCollections;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class FlockCreator implements MissionStepCreator {

    private final ComboBox<String> droneSelection;
    private final ComboBox<String> leaderSelection;
    private TextField xoffset;
    private TextField yoffset;

    public FlockCreator(List<String> drones) {
        this.droneSelection = new ComboBox<>(FXCollections.observableList(drones));
        this.leaderSelection = new ComboBox<>(FXCollections.observableList(drones));
    }

    @Override
    public void create(GridPane grid) {

        xoffset = new TextField();
        yoffset = new TextField();

        xoffset.setText("10");
        yoffset.setText("0");

        grid.add(new Label("Drone:"), 1, 2);
        grid.add(droneSelection, 2, 2);
        grid.add(new Label("Leader:"), 1, 3);
        grid.add(leaderSelection, 2, 3);
        grid.add(new Label("x offset:"), 1, 4);
        grid.add(xoffset, 2, 4);
        grid.add(new Label("y offset:"), 1, 5);
        grid.add(yoffset, 2, 5);
    }

    @Override
    public MissionStep build() {
        return new MissionStepFlock(droneSelection.getSelectionModel().getSelectedItem(),
                leaderSelection.getSelectionModel().getSelectedItem(),
                Double.parseDouble(xoffset.getText()),
                Double.parseDouble(yoffset.getText()));
    }
}
