package edu.unm.dragonfly.mission.step;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepSleep implements MissionStep {
    private final List<String> drones;
    private final double duration;

    public MissionStepSleep(List<String> drones, double duration) {
        this.drones = drones;
        this.duration = duration;
    }

    @Override
    public String toString() {
        return "MissionStepSleep{" +
                "drones=" + drones +
                ", duration=" + duration +
                '}';
    }
}
