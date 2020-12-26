package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepSemaphore;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class SemaphoreCreator implements MissionStepCreator {

    private final SelectableDronesCreator droneSelection;

    public SemaphoreCreator(List<String> drones) {
        droneSelection = new SelectableDronesCreator(drones);
    }

    @Override
    public void create(GridPane grid) {
        droneSelection.create(grid, 2);
    }

    @Override
    public MissionStep build() {
        return new MissionStepSemaphore(droneSelection.getSelectedDrones());
    }
}
