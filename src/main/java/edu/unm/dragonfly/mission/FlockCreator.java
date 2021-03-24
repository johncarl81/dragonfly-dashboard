package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepFlock;
import javafx.collections.FXCollections;
import javafx.scene.control.ComboBox;
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

        GridUtil.builder(grid).increment()
                .add("Drone:", droneSelection)
                .add("Leader:", leaderSelection)
                .add("X offset:", xoffset)
                .add("Y offset:", yoffset);
    }

    @Override
    public MissionStep build() {
        return new MissionStepFlock(droneSelection.getSelectionModel().getSelectedItem(),
                leaderSelection.getSelectionModel().getSelectedItem(),
                Double.parseDouble(xoffset.getText()),
                Double.parseDouble(yoffset.getText()));
    }
}
