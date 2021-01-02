package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepFlock implements MissionStep {

    private final String drone;
    private final String leader;
    private final double x;
    private final double y;

    @JsonCreator
    public MissionStepFlock(@JsonProperty("drone") String drone, @JsonProperty("leader") String leader, @JsonProperty("x") double x, @JsonProperty("y") double y) {
        this.drone = drone;
        this.leader = leader;
        this.x = x;
        this.y = y;
    }


    public String getLeader() {
        return leader;
    }

    public String getDrone() {
        return drone;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    @Override
    public boolean appliesTo(String name) {
        return this.drone.equals(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper) {
        ObjectNode flock = mapper.createObjectNode();

        flock.put("msg_type", MissionStepType.FLOCK.getMission_type());
        ObjectNode data = flock.putObject("flock");

        data.put("x", x);
        data.put("y", y);
        data.put("leader", leader);

        return flock;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepFlock that = (MissionStepFlock) o;

        if (Double.compare(that.x, x) != 0) return false;
        if (Double.compare(that.y, y) != 0) return false;
        if (drone != null ? !drone.equals(that.drone) : that.drone != null) return false;
        return leader != null ? leader.equals(that.leader) : that.leader == null;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = drone != null ? drone.hashCode() : 0;
        result = 31 * result + (leader != null ? leader.hashCode() : 0);
        temp = Double.doubleToLongBits(x);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public String toString() {
        return "MissionStepFlock{" +
                "drone='" + drone + '\'' +
                ", leader='" + leader + '\'' +
                ", x=" + x +
                ", y=" + y +
                '}';
    }
}
