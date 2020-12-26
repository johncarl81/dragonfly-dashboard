package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepSleep;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class SleepCreator implements MissionStepCreator {

    private final SelectableDronesCreator droneSelection;
    private TextField durationField;

    public SleepCreator(List<String> drones) {
        droneSelection = new SelectableDronesCreator(drones);
    }

    @Override
    public void create(GridPane grid) {
        durationField = new TextField();

        durationField.setText("0");
        droneSelection.create(grid, 2);
        grid.add(new Label("Duration:"), 1, 3);
        grid.add(durationField, 2, 3);
    }

    @Override
    public MissionStep build() {
        return new MissionStepSleep(droneSelection.getSelectedDrones(), Float.parseFloat(durationField.getText()));
    }

}
