package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepGradient;
import javafx.collections.FXCollections;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class GradientCreator implements MissionStepCreator {

    private final ComboBox<String> targetSelection;
    private final SelectableDronesCreator droneSelection;

    public GradientCreator(List<String> drones) {
        targetSelection = new ComboBox<>(FXCollections.observableList(drones));
        droneSelection = new SelectableDronesCreator(drones);
    }

    @Override
    public void create(GridPane grid) {
        grid.add(new Label("Target:"), 1, 2);
        grid.add(targetSelection, 2, 2);
        droneSelection.create(grid, 3);
    }

    @Override
    public MissionStep build() {
        return new MissionStepGradient(targetSelection.getSelectionModel().getSelectedItem(), droneSelection.getSelectedDrones());
    }
}
