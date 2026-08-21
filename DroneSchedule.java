import java.util.List;

public class DroneSchedule {
    public int src;
    public List<List<Integer>> sequences;
    public double makespan;
    public double maxStartingTime;

    public DroneSchedule(int src, List<List<Integer>> sequences) {
        this.src = src;
        this.sequences = sequences;
        this.makespan = -1;
        this.maxStartingTime = -1;
    }

    public int getNumDrone() { return this.sequences.size(); }

    public double makespan() {
        if(this.makespan != -1) { return this.makespan; }
        
        double totalTime = Double.MIN_VALUE;
        for(var droneSequence : sequences) {
            double singleTime = 0;
            for(int customer : droneSequence) {
                singleTime += Constant.DRONE_SETUP_TIME
                            + VRPInstance.distMatrix[src][customer] / Constant.DRONE_SPEED
                            + VRPInstance.nodes.get(customer).servingTime / 2
                            + VRPInstance.distMatrix[src][customer] / Constant.DRONE_SPEED;
            }
            if(singleTime > totalTime) totalTime = singleTime;
        }

        return totalTime;
    }

    
}
