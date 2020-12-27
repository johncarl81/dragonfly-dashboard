package edu.unm.dragonfly.mission;

import edu.unm.dragonfly.mission.step.MissionStep;
import javafx.collections.ObservableList;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionDataHolder {
    private List<MissionStep> steps;

    public MissionDataHolder() {
        // Empty Bean constructor
    }

    public MissionDataHolder(List<MissionStep> steps) {
        this.steps = steps;
    }

    public List<MissionStep> getSteps() {
        return steps;
    }
}
