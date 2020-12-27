package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepRTL implements MissionStep {

    private final List<String> drones;

    @JsonCreator
    public MissionStepRTL(@JsonProperty("drones") List<String> drones) {
        this.drones = drones;
    }

    public List<String> getDrones() {
        return drones;
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
