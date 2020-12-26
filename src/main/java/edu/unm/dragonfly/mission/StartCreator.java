package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStart;
import edu.unm.dragonfly.mission.step.MissionStep;
import javafx.scene.layout.GridPane;

/**
 * @author John Ericksen
 */
public class StartCreator implements MissionStepCreator {
    @Override
    public void create(GridPane grid) {

    }

    @Override
    public MissionStep build() {
        return new MissionStart();
    }

    @Override
    public String toString() {
        return "Mission Start";
    }
}
