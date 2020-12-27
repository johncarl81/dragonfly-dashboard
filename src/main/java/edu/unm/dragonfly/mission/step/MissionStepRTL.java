package edu.unm.dragonfly.mission.step;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepRTL implements MissionStep {

    private List<String> drones;

    public MissionStepRTL() {
        // Empty bean constructor
    }

    public List<String> getDrones() {
        return drones;
    }

    public MissionStepRTL(List<String> drones) {
        this.drones = drones;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepRTL that = (MissionStepRTL) o;

        return drones != null ? drones.equals(that.drones) : that.drones == null;
    }

    @Override
    public int hashCode() {
        return drones != null ? drones.hashCode() : 0;
    }

    @Override
    public String toString() {
        return "RTL{" + drones + '}';
    }
}
