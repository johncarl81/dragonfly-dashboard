package edu.unm.dragonfly;

import edu.unm.dragonfly.msgs.MavrosState;

/**
 * @author John Ericksen
 */
public class DroneStatus {

    private final Drone drone;
    private final MavrosState.SystemStatus status;

    public DroneStatus(Drone drone, MavrosState.SystemStatus status) {
        this.drone = drone;
        this.status = status;
    }

    public Drone getDrone() {
        return drone;
    }

    public MavrosState.SystemStatus getStatus() {
        return status;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        DroneStatus that = (DroneStatus) o;

        return drone.equals(that.drone);
    }

    @Override
    public int hashCode() {
        return drone.hashCode();
    }
}
