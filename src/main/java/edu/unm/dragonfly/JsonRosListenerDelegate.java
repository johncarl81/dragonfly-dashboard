package edu.unm.dragonfly;

import com.fasterxml.jackson.databind.JsonNode;
import ros.RosListenDelegate;
import ros.tools.MessageUnpacker;

/**
 * @author John Ericksen
 */
public class JsonRosListenerDelegate<T> implements RosListenDelegate {

    private final MessageUnpacker<T> unpacker;
    private final Receiver<T> receiver;

    public interface Receiver<T>{
        void receive(T value);
    }

    public JsonRosListenerDelegate(Class<? super T> clazz, Receiver<T> receiver) {
        this.unpacker = new MessageUnpacker<>(clazz);
        this.receiver = receiver;
    }

    @Override
    public void receive(JsonNode data, String stringRep) {
        receiver.receive(unpacker.unpackRosMessage(data));
    }
}
