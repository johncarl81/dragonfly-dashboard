package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;
import edu.unm.dragonfly.mission.GridUtil;
import javafx.collections.FXCollections;
import javafx.scene.control.ButtonType;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Dialog;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;

import java.util.Map;
import java.util.Optional;

public class AddWaypointDialogFactory {

    public interface AddWaypointCallback {
        void call(String name, double latitude, double longitude, double altitude, float distanceThreshold);
    }

    public interface NavigateWaypointCallback {
        void call(NavigateWaypoint waypoint);
    }

    public static void create(Point point, AddWaypointCallback callback) {

        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("Add Waypoint");

        GridPane grid = new GridPane();

        TextField nameField = new TextField();
        TextField latitudeField = new TextField();
        TextField longitudeField = new TextField();
        TextField altitudeField = new TextField();
        TextField distanceThreshold = new TextField();

        // Set Defaults
        nameField.setText("");
        latitudeField.setText(Double.toString(point.getY()));
        longitudeField.setText(Double.toString(point.getX()));
        altitudeField.setText("10");
        distanceThreshold.setText("1");

        GridUtil.builder(grid)
                .add("Name:", nameField)
                .add("Latitude:", latitudeField)
                .add("Longitude:", longitudeField)
                .add("Altitude:", altitudeField)
                .add("Distance Threshold:", distanceThreshold);

        dialog.getDialogPane().setContent(grid);

        dialog.getDialogPane().getButtonTypes().add(ButtonType.CANCEL);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.OK);

        Optional<ButtonType> result = dialog.showAndWait();

        if(result.isPresent() && result.get() == ButtonType.OK) {
            callback.call(nameField.getText(),
                    Float.parseFloat(latitudeField.getText()),
                    Float.parseFloat(longitudeField.getText()),
                    Float.parseFloat(altitudeField.getText()),
                    Float.parseFloat(distanceThreshold.getText()));
        }
    }

    public static void createSelect(Map<String, NavigateWaypoint> waypoints, Drone selected, NavigateWaypointCallback callback) {

        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("Add Waypoint");

        GridPane grid = new GridPane();

        ComboBox<String> waypointComboBox = new ComboBox<>(FXCollections.observableArrayList(waypoints.keySet()));

        grid.add(new Label("Name: "), 1, 1);
        grid.add(new Label(selected.getName()), 2, 1);
        grid.add(new Label("Waypoint: "), 1, 2);
        grid.add(waypointComboBox, 2, 2);

        dialog.getDialogPane().setContent(grid);

        dialog.getDialogPane().getButtonTypes().add(ButtonType.CANCEL);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.OK);

        Optional<ButtonType> result = dialog.showAndWait();

        if(result.isPresent() && result.get() == ButtonType.OK) {
            callback.call(waypoints.get(waypointComboBox.getValue()));
        }
    }
}
