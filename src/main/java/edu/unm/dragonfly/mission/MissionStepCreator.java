package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import javafx.scene.layout.GridPane;

/**
 * @author John Ericksen
 */
public interface MissionStepCreator {

    void create(GridPane grid);

    MissionStep build();
}
