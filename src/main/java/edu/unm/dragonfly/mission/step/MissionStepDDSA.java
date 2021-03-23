package edu.unm.dragonfly.mission.step;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.unm.dragonfly.Walk;

import java.util.List;

/**
 * @author John Ericksen
 */
public class MissionStepDDSA implements MissionStep{

    private final List<String> drones;
    private final String waypoint;
    private final float radius;
    private final float stepLength;
    private final float altitude;
    private final int loops;
    private final int stacks;
    private final Walk walk;
    private final float waitTime;
    private final float distanceThreshold;
    private final boolean uniqueAltitudes;

    @JsonCreator
    public MissionStepDDSA(@JsonProperty("drone") List<String> drones,
                           @JsonProperty("waypoint") String waypoint,
                           @JsonProperty("radius") float radius,
                           @JsonProperty("stepLength") float stepLength,
                           @JsonProperty("altitude") float altitude,
                           @JsonProperty("loops") int loops,
                           @JsonProperty("stacks") int stacks,
                           @JsonProperty("walk") Walk walk,
                           @JsonProperty("waitTime") float waitTime,
                           @JsonProperty("distanceThreshold") float distanceThreshold,
                           @JsonProperty("uniqueAltitudes") boolean uniqueAltitudes) {
        this.drones = drones;
        this.waypoint = waypoint;
        this.radius = radius;
        this.stepLength = stepLength;
        this.altitude = altitude;
        this.loops = loops;
        this.stacks = stacks;
        this.walk = walk;
        this.waitTime = waitTime;
        this.distanceThreshold = distanceThreshold;
        this.uniqueAltitudes = uniqueAltitudes;
    }

    public List<String> getDrones() {
        return drones;
    }

    public String getWaypoint() {
        return waypoint;
    }

    public float getRadius() {
        return radius;
    }

    public float getStepLength() {
        return stepLength;
    }

    public float getAltitude() {
        return altitude;
    }

    public int getLoops() {
        return loops;
    }

    public int getStacks() {
        return stacks;
    }

    public Walk getWalk() {
        return walk;
    }

    public float getWaitTime() {
        return waitTime;
    }

    public float getDistanceThreshold() {
        return distanceThreshold;
    }

    public boolean isUniqueAltitudes() {
        return uniqueAltitudes;
    }

    @Override
    public boolean appliesTo(String name) {
        return this.drones.contains(name);
    }

    @Override
    public ObjectNode toROSJson(ObjectMapper mapper, String droneName) {
        ObjectNode ddsa = mapper.createObjectNode();

        ddsa.put("msg_type", MissionStepType.DDSA.getMission_type());

        ObjectNode data = ddsa.putObject("ddsa");
        data.put("waypoint", waypoint);
        data.put("radius", radius);
        data.put("stepLength", stepLength);
        data.put("walk", walk.id);
        data.put("swarm_size", drones.size());
        data.put("swarm_index", drones.indexOf(droneName));
        data.put("stacks", stacks);
        data.put("loops", loops);
        if(uniqueAltitudes) {
            data.put("altitude", altitude + (radius * drones.indexOf(droneName)));
        } else {
            data.put("altitude", altitude);
        }
        data.put("waitTime", waitTime);
        data.put("distanceThreshold", distanceThreshold);


        return ddsa;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        MissionStepDDSA that = (MissionStepDDSA) o;

        if (Float.compare(that.radius, radius) != 0) return false;
        if (Float.compare(that.stepLength, stepLength) != 0) return false;
        if (Float.compare(that.altitude, altitude) != 0) return false;
        if (loops != that.loops) return false;
        if (stacks != that.stacks) return false;
        if (Float.compare(that.waitTime, waitTime) != 0) return false;
        if (Float.compare(that.distanceThreshold, distanceThreshold) != 0) return false;
        if (uniqueAltitudes != that.uniqueAltitudes) return false;
        if (drones != null ? !drones.equals(that.drones) : that.drones != null) return false;
        if (waypoint != null ? !waypoint.equals(that.waypoint) : that.waypoint != null) return false;
        return walk == that.walk;
    }

    @Override
    public int hashCode() {
        int result = drones != null ? drones.hashCode() : 0;
        result = 31 * result + (waypoint != null ? waypoint.hashCode() : 0);
        result = 31 * result + (radius != +0.0f ? Float.floatToIntBits(radius) : 0);
        result = 31 * result + (stepLength != +0.0f ? Float.floatToIntBits(stepLength) : 0);
        result = 31 * result + (altitude != +0.0f ? Float.floatToIntBits(altitude) : 0);
        result = 31 * result + loops;
        result = 31 * result + stacks;
        result = 31 * result + (walk != null ? walk.hashCode() : 0);
        result = 31 * result + (waitTime != +0.0f ? Float.floatToIntBits(waitTime) : 0);
        result = 31 * result + (distanceThreshold != +0.0f ? Float.floatToIntBits(distanceThreshold) : 0);
        result = 31 * result + (uniqueAltitudes ? 1 : 0);
        return result;
    }

    @Override
    public String toString() {
        return "DDSA{" +
                "drones=" + drones +
                ", waypoint='" + waypoint + '\'' +
                ", radius=" + radius +
                ", stepLength=" + stepLength +
                ", altitude=" + altitude +
                ", loops=" + loops +
                ", stacks=" + stacks +
                ", walk=" + walk +
                ", waitTime=" + waitTime +
                ", distanceThreshold=" + distanceThreshold +
                ", uniqueAltitudes=" + uniqueAltitudes +
                '}';
    }
}
