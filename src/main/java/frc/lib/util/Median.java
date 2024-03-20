package frc.lib.util;

import java.util.LinkedList;

public class Median {
    private final LinkedList<Double> observations = new LinkedList<>();
    private int numberOfObservations;

    /**
     * Super simple median calculator. Not very efficient!!!
     * For one-shot tasks, will probably kill loop times.
     * @param numberOfObservations the number of observations to keep track of
     */
    public Median(int numberOfObservations){
        this.numberOfObservations = numberOfObservations;
    }

    public void add(double observation){
        observations.add(observation);
        if(observations.size() > numberOfObservations){
            observations.removeFirst();
        }
    }

    public double get(){
        if(observations.size() == 0){
            return 0;
        }
        double[] sortedObservations = observations.stream().mapToDouble(Double::doubleValue).sorted().toArray();
        if(sortedObservations.length % 2 == 0){
            return (sortedObservations[sortedObservations.length/2] + sortedObservations[sortedObservations.length/2 - 1])/2;
        }
        return sortedObservations[sortedObservations.length/2];
    }
}
