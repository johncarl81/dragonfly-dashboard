package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepCalibration;
import edu.unm.dragonfly.mission.step.MissionStepGradient;
import javafx.collections.FXCollections;
import javafx.scene.control.ComboBox;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class CalibrationCreator implements MissionStepCreator {

    private final ComboBox<String> targetSelection;
    private final SelectableDronesCreator droneSelection;

    public CalibrationCreator(List<String> drones) {
        targetSelection = new ComboBox<>(FXCollections.observableList(drones));
        droneSelection = new SelectableDronesCreator(drones);
    }

    @Override
    public void create(GridPane grid) {
        GridUtil.builder(grid).increment()
                .add("Target:", targetSelection)
                .add(droneSelection);
    }

    @Override
    public MissionStep build() {
        return new MissionStepCalibration(targetSelection.getSelectionModel().getSelectedItem(), droneSelection.getSelectedDrones());
    }
}
