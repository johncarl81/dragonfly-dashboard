package edu.unm.dragonfly.msgs;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonValue;
import com.google.auto.value.AutoValue;
import ros.msgs.std_msgs.Header;

import java.util.HashMap;
import java.util.Map;

/**
 * @author John Ericksen
 */
@AutoValue
public abstract class MavrosState {

    public enum SystemStatus {
        MAV_STATE_UNINIT(0, "Uninitialized", "Uninitialized system, state is unknown."),
        MAV_STATE_BOOT(1, "Booting", "System is booting up."),
        MAV_STATE_CALIBRATING(2, "Calibrating", "System is calibrating and not flight-ready."),
        MAV_STATE_STANDBY(3, "Ready", "System is grounded and on standby. It can be launched any time."),
        MAV_STATE_ACTIVE(4, "Active", "System is active and might be already airborne. Motors are engaged."),
        MAV_STATE_CRITICAL(5, "Critical", "System is in a non-normal flight mode. It can however still navigate."),
        MAV_STATE_EMERGENCY(6, "Emergency", "System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down."),
        MAV_STATE_POWEROFF(7, "Power-Down", "System just initialized its power-down sequence, will shut down now."),
        MAV_STATE_FLIGHT_TERMINATION(8, "Termination", "System is terminating itself.");

        private static final Map<Integer, SystemStatus> STATE_MAP = new HashMap<>();

        static {
            for(SystemStatus state : SystemStatus.values()) {
                STATE_MAP.put(state.value, state);
            }
        }

        private final int value;
        private final String name;
        private final String description;

        SystemStatus(int value, String name, String description) {
            this.value = value;
            this.name = name;
            this.description = description;
        }

        @JsonValue
        public int getValue() {
            return value;
        }

        @JsonCreator
        public static SystemStatus forValue(int value) {
            return STATE_MAP.get(value);
        }

        public String getName() {
            return name;
        }

        public String getDescription() {
            return description;
        }

        public String toString() {
            return name;
        }
    }

    @JsonProperty
    public abstract Header header();
    @JsonProperty
    public abstract boolean connected();
    @JsonProperty
    public abstract boolean armed();
    @JsonProperty
    public abstract boolean guided();
    @JsonProperty("manual_input")
    public abstract boolean manualInput();
    @JsonProperty
    public abstract String mode();
    @JsonProperty("system_status")
    public abstract SystemStatus systemStatus();

    @JsonCreator
    public static MavrosState create(@JsonProperty("header") Header header,
                                     @JsonProperty("connected") boolean connected,
                                     @JsonProperty("armed") boolean armed,
                                     @JsonProperty("guided") boolean guided,
                                     @JsonProperty("manual_input") boolean manualInput,
                                     @JsonProperty("mode") String mode,
                                     @JsonProperty("system_status") SystemStatus systemStatus) {
        return new AutoValue_MavrosState(header, connected, armed, guided, manualInput, mode, systemStatus);
    }
}
