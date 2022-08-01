package edu.unm.dragonfly;

import edu.unm.dragonfly.msgs.MavrosState;
import javafx.scene.control.ListCell;
import javafx.scene.control.ListView;
import javafx.scene.control.Tooltip;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.util.Callback;

import java.util.EnumMap;
import java.util.Map;

/**
 * @author John Ericksen
 */
public class DroneCellFactory implements Callback<ListView<DroneStatus>, ListCell<DroneStatus>> {

    private enum StatusIcon {
        NONE("images/bullet_black.png"),
        PREPARE("images/bullet_yellow.png"),
        READY("images/bullet_green.png"),
        ACTIVE("images/bullet_blue.png"),
        ERROR("images/bullet_red.png");

        private static final Map<MavrosState.SystemStatus, StatusIcon> STATUS_MAP = new EnumMap<>(MavrosState.SystemStatus.class);

        static {
            STATUS_MAP.put(MavrosState.SystemStatus.MAV_STATE_UNINIT, NONE);
            STATUS_MAP.put(MavrosState.SystemStatus.MAV_STATE_BOOT, PREPARE);
            STATUS_MAP.put(MavrosState.SystemStatus.MAV_STATE_CALIBRATING, PREPARE);
            STATUS_MAP.put(MavrosState.SystemStatus.MAV_STATE_STANDBY, READY);
            STATUS_MAP.put(MavrosState.SystemStatus.MAV_STATE_ACTIVE, ACTIVE);
            STATUS_MAP.put(MavrosState.SystemStatus.MAV_STATE_CRITICAL, ERROR);
            STATUS_MAP.put(MavrosState.SystemStatus.MAV_STATE_EMERGENCY, ERROR);
            STATUS_MAP.put(MavrosState.SystemStatus.MAV_STATE_POWEROFF, NONE);
            STATUS_MAP.put(MavrosState.SystemStatus.MAV_STATE_FLIGHT_TERMINATION, ERROR);

            for(MavrosState.SystemStatus status : MavrosState.SystemStatus.values()) {
                if(!STATUS_MAP.containsKey(status)) {
                    throw new IllegalArgumentException("System Status " + status + " not associated with icon.");
                }
            }
        }

        private final String icon;

        StatusIcon(String icon) {
            this.icon = icon;
        }

        public static StatusIcon get(MavrosState.SystemStatus systemStatus) {
            return STATUS_MAP.get(systemStatus);
        }

        String getIcon() {
            return icon;
        }
    }

    @Override
    public ListCell<DroneStatus> call(ListView<DroneStatus> param) {
        return new ListCell<>() {
            final Tooltip tooltip = new Tooltip();
            final ImageView imageView = new ImageView();
            @Override
            protected void updateItem(DroneStatus droneStatus, boolean empty) {

                super.updateItem(droneStatus, empty);

                if (droneStatus == null || empty) {
                    setGraphic(null);
                    setText(null);
                    setTooltip(null);
                } else {
                    String name = droneStatus.getDrone().getName();
                    MavrosState.SystemStatus status = droneStatus.getStatus();

                    setText(name + "(" + status.getName() + ")");

                    imageView.setFitHeight(16);
                    imageView.setFitWidth(16);
                    imageView.setImage(new Image(StatusIcon.get(status).getIcon()));
                    setGraphic(imageView);

                    tooltip.setText(status.getDescription());
                    setTooltip(tooltip);
                }
            }
        };
    }
}
