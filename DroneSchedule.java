import java.util.List;

public class DroneSchedule {
    public List<List<Integer>> sequences;
    public double makespan;
    public double maxStartingTime;

    public DroneSchedule(List<List<Integer>> sequences) {
        this.sequences = sequences;
        this.makespan = -1;
        this.maxStartingTime = -1;
    }

    public int getNumDrone() { return this.sequences.size(); }
}
