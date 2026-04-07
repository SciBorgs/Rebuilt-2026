package org.sciborgs1155.robot.indexer;

public class NoIndexer implements IndexerIO {

    public NoIndexer() {}

    @Override
    public void setVoltage(double volt) {}

    @Override
    public double velocity() {
        return 0.0;
    }

    @Override
    public void close() {}
}
