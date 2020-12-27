package edu.unm.dragonfly.mission.step;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepLand implements MissionStep {

    private List<String> drones;

    public MissionStepLand() {
        // Empty Bean Constructor
    }

    public MissionStepLand(List<String> drones) {
        this.drones = drones;
    }

    public List<String> getDrones() {
        return drones;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepLand that = (MissionStepLand) o;

        return drones != null ? drones.equals(that.drones) : that.drones == null;
    }

    @Override
    public int hashCode() {
        return drones != null ? drones.hashCode() : 0;
    }

    @Override
    public String toString() {
        return "Land{" +
                "drones=" + drones +
                '}';
    }
}
