package edu.unm.dragonfly;

import javafx.scene.control.ListCell;
import javafx.scene.control.ListView;
import javafx.scene.control.Tooltip;
import javafx.util.Callback;

/**
 * @author John Ericksen
 */
public class TooltipCellFactory<T> implements Callback<ListView<T>, ListCell<T>> {

    @Override
    public ListCell<T> call(ListView<T> param) {
        return new ListCell<T>() {
            final Tooltip tooltip = new Tooltip();
            @Override
            protected void updateItem(T item, boolean empty) {
                super.updateItem(item, empty);

                if (item == null || empty) {
                    setText(null);
                    setTooltip(null);
                } else {
                    setText(item.toString());
                    tooltip.setText(item.toString());
                    setTooltip(tooltip);
                }
            }
        };
    }
}
