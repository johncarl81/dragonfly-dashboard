package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepPump;
import edu.unm.dragonfly.mission.step.MissionStepSleep;
import javafx.collections.FXCollections;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.Arrays;
import java.util.List;

/**
 * @author John Ericksen
 */
public class PumpCreator implements MissionStepCreator {

    private final SelectableDronesCreator droneSelection;
    private ComboBox<Integer> pumpNumber;

    public PumpCreator(List<String> drones) {
        droneSelection = new SelectableDronesCreator(drones);
    }

    @Override
    public void create(GridPane grid) {
        pumpNumber = new ComboBox<>(FXCollections.observableList(Arrays.asList(0, 1, 2, 3)));

        GridUtil.builder(grid).increment()
                .add(droneSelection)
                .add("Pump Number:", pumpNumber);
    }

    @Override
    public MissionStep build() {
        return new MissionStepPump(droneSelection.getSelectedDrones(), pumpNumber.getSelectionModel().getSelectedItem());
    }

}
