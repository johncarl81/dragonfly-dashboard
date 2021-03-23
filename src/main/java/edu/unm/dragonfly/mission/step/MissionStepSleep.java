package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

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
    public boolean appliesTo(String name) {
        return this.drones.contains(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper, String droneName) {
        ObjectNode sleep = mapper.createObjectNode();

        sleep.put("msg_type", MissionStepType.SLEEP.getMission_type());
        ObjectNode data = sleep.putObject("sleep");

        data.put("duration", duration);

        return sleep;
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
        return "Sleep{" +
                "drones=" + drones +
                ", duration=" + duration +
                '}';
    }
}
