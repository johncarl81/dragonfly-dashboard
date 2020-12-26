package edu.unm.dragonfly.mission.step;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepTakeoff implements MissionStep {

    private final List<String> drones;
    private final double altitude;

    public MissionStepTakeoff(List<String> drones, double altitude) {
        this.drones = drones;
        this.altitude = altitude;
    }

    @Override
    public String toString() {
        return "Takeoff{" +
                "drones=" + drones +
                ", altitude=" + altitude +
                '}';
    }
}
