package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepSleep;
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

        GridUtil.builder(grid).increment()
                .add(droneSelection)
                .add("Duration:", durationField);
    }

    @Override
    public MissionStep build() {
        return new MissionStepSleep(droneSelection.getSelectedDrones(), Float.parseFloat(durationField.getText()));
    }

}
