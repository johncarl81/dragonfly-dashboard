package edu.unm.dragonfly;

import edu.unm.dragonfly.mission.*;
import edu.unm.dragonfly.msgs.*;
import javafx.collections.*;
import javafx.scene.control.*;
import javafx.scene.layout.*;

import java.util.*;
import java.util.stream.*;

/**
 * @author John Ericksen
 */
public class PlumeDialog {

    public interface PlumeFactoryCallback {
        void call(String name, Plume plume);
    }

    public static void create(Map<String, NavigateWaypoint> waypoints, PlumeFactoryCallback callback) {
        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("Plume Parameters");

        ComboBox<String> waypointComboBox = new ComboBox<>(FXCollections.observableArrayList(waypoints.keySet()));
        TextField name = new TextField();
        TextField windDirection = new TextField();
        TextField flux = new TextField();
        TextField diffusion = new TextField();
        TextField windSpeed = new TextField();

        // Set Defaults
        name.setText("");
        windDirection.setText("0");
        flux.setText("5000");
        diffusion.setText("2");
        windSpeed.setText("1");

        GridPane grid = new GridPane();
        ColumnConstraints constraints1 = new ColumnConstraints(100);
        ColumnConstraints constraints2 = new ColumnConstraints();
        constraints2.setHgrow(Priority.ALWAYS);
        grid.getColumnConstraints().addAll(constraints1, constraints2);

        GridUtil.builder(grid)
                .add("Source:", waypointComboBox)
                .add("Name:", name)
                .add("Wind Direction (° from N)", windDirection)
                .add("Flux (q kg/s)", flux)
                .add("Diffusion (k)", diffusion)
                .add("Wind Speed (u m/s)", windSpeed);

        dialog.getDialogPane().setContent(grid);

        dialog.getDialogPane().getButtonTypes().add(ButtonType.CANCEL);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.OK);

        Optional<ButtonType> result = dialog.showAndWait();

        if(result.isPresent() && result.get() == ButtonType.OK) {
            callback.call(name.getText(),
                    Plume.create(waypoints.get(waypointComboBox.getSelectionModel().getSelectedItem()).getWaypoint().toLatLon(),
                    Float.parseFloat(windDirection.getText()),
                    Float.parseFloat(flux.getText()),
                    Float.parseFloat(diffusion.getText()),
                    Float.parseFloat(windSpeed.getText())));
        }
    }
}
