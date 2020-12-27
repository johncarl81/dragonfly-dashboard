package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepSleep implements MissionStep {
    private final List<String> drones;
    private final double duration;

    @JsonCreator
    public MissionStepSleep(@JsonProperty("drones") List<String> drones, @JsonProperty("duration") double duration) {
        this.drones = drones;
        this.duration = duration;
    }

    public List<String> getDrones() {
        return drones;
    }

    public double getDuration() {
        return duration;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepSleep that = (MissionStepSleep) o;

        if (Double.compare(that.duration, duration) != 0) return false;
        return drones != null ? drones.equals(that.drones) : that.drones == null;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = drones != null ? drones.hashCode() : 0;
        temp = Double.doubleToLongBits(duration);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "MissionStepSleep{" +
                "drones=" + drones +
                ", duration=" + duration +
                '}';
    }
}
