package edu.unm.dragonfly;

import edu.unm.dragonfly.mission.GridUtil;
import edu.unm.dragonfly.mission.Waypoint;
import javafx.collections.FXCollections;
import javafx.scene.control.ButtonType;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Dialog;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.TextField;
import javafx.scene.layout.ColumnConstraints;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.Priority;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

public class PumpDialogFactory {

    public interface DialogCallback {
        void call(int pump_num);
    }

    public static void create(DialogCallback callback) {

        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("Pump");

        GridPane grid = new GridPane();
        ColumnConstraints constraints1 = new ColumnConstraints(100);
        ColumnConstraints constraints2 = new ColumnConstraints();
        constraints2.setHgrow(Priority.ALWAYS);
        grid.getColumnConstraints().addAll(constraints1, constraints2);

        // Set Defaults
        ComboBox<Integer> pumpNumberSelection = new ComboBox<>(FXCollections.observableList(Arrays.asList(0, 1, 2, 3)));

        GridUtil.builder(grid)
                .add("Pump Number:", pumpNumberSelection);

        dialog.getDialogPane().setContent(grid);

        dialog.getDialogPane().getButtonTypes().add(ButtonType.CANCEL);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.OK);

        Optional<ButtonType> result = dialog.showAndWait();

        if(result.isPresent() && result.get() == ButtonType.OK) {
            callback.call(pumpNumberSelection.getSelectionModel().getSelectedItem());
        }
    }
}
