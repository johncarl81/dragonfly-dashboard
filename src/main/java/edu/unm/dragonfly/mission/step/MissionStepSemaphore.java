package edu.unm.dragonfly.mission.step;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepSemaphore implements MissionStep {

    private final List<String> drones;

    public MissionStepSemaphore(List<String> drones) {
        this.drones = drones;
    }

    @Override
    public String toString() {
        return "Semaphore{" + drones + '}';
    }
}
