package edu.unm.dragonfly.mission.step;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepRTL implements MissionStep {

    private final List<String> drones;

    public MissionStepRTL(List<String> drones) {
        this.drones = drones;
    }

    @Override
    public String toString() {
        return "RTL{" + drones + '}';
    }
}
