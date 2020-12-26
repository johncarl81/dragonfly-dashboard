package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepLand;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class LandCreator implements MissionStepCreator {

    private final SelectableDronesCreator droneSelection;

    public LandCreator(List<String> drones) {
        droneSelection = new SelectableDronesCreator(drones);
    }

    @Override
    public void create(GridPane grid) {
        droneSelection.create(grid, 2);
    }

    @Override
    public MissionStep build() {
        return new MissionStepLand(droneSelection.getSelectedDrones());
    }
}
