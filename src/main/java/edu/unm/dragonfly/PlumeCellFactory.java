package edu.unm.dragonfly;

import javafx.scene.control.*;
import javafx.util.*;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

/**
 * @author John Ericksen
 */
public class PlumeCellFactory implements Callback<ListView<NamedPlume>, ListCell<NamedPlume>> {

    @Override
    public ListCell<NamedPlume> call(ListView<NamedPlume> param) {
        return new ListCell<>() {
            final Tooltip tooltip = new Tooltip();
            final ImageView imageView = new ImageView();
            @Override
            protected void updateItem(NamedPlume item, boolean empty) {

                super.updateItem(item, empty);

                if (item == null || empty) {
                    setGraphic(null);
                    setText(null);
                    setTooltip(null);
                } else {
                    imageView.setImage(new Image("images/clouds.png"));
                    imageView.setFitHeight(24);
                    imageView.setFitWidth(24);
                    setGraphic(imageView);
                    setText(item.getName());
                    tooltip.setText(item.getName());
                    setTooltip(tooltip);
                }
            }
        };
    }
}
