package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepTakeoff;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class TakeoffCreator implements MissionStepCreator {

    private final SelectableDronesCreator droneSelection;
    private TextField altitudeField;

    public TakeoffCreator(List<String> drones) {
        droneSelection = new SelectableDronesCreator(drones);
    }

    @Override
    public void create(GridPane grid) {
        altitudeField = new TextField();

        altitudeField.setText("10");

        droneSelection.create(grid, 2);
        grid.add(new Label("Altitude:"), 1, 3);
        grid.add(altitudeField, 2, 3);
    }

    @Override
    public MissionStep build() {
        return new MissionStepTakeoff(droneSelection.getSelectedDrones(), Double.parseDouble(altitudeField.getText()));
    }
}
