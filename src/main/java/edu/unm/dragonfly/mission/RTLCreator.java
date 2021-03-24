package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import edu.unm.dragonfly.mission.step.MissionStepRTL;
import javafx.scene.layout.GridPane;

import java.util.List;

/**
 * @author John Ericksen
 */
public class RTLCreator implements MissionStepCreator {


    private final SelectableDronesCreator droneSelection;

    public RTLCreator(List<String> drones) {
        droneSelection = new SelectableDronesCreator(drones);
    }

    @Override
    public void create(GridPane grid) {
        droneSelection.create(grid, 2);

    }

    @Override
    public MissionStep build() {
        return new MissionStepRTL(droneSelection.getSelectedDrones());
    }
}
