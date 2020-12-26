package edu.unm.dragonfly.mission.step;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepLand implements MissionStep {

    private final List<String> drones;

    public MissionStepLand(List<String> drones) {
        this.drones = drones;
    }

    @Override
    public String toString() {
        return "Land{" +
                "drones=" + drones +
                '}';
    }
}
