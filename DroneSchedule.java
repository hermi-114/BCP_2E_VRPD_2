import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class DroneSchedule {
    public int src;
    public List<List<Integer>> sequences;
    public double makespan;
    public double maxStartingTime;
    public Set<Integer> customerServed;

    public DroneSchedule(int src, List<List<Integer>> sequences) {
        this.src = src;
        this.sequences = sequences;
        this.makespan = -1;
        this.maxStartingTime = Double.MAX_VALUE;
        this.customerServed = new HashSet<>();

        for(var droneSequence : sequences) {
            double singleTime = 0;
            for(int customer : droneSequence) {
                if(customerServed.contains(customer)) throw new IllegalArgumentException("Customer(s) served more than once");
                customerServed.add(customer);

                singleTime += Constant.DRONE_SETUP_TIME
                            + VRPInstance.distMatrix[src][customer] / Constant.DRONE_SPEED
                            + VRPInstance.nodes.get(customer).servingTime / 2
                            + VRPInstance.distMatrix[src][customer] / Constant.DRONE_SPEED;
                
                double startTime = VRPInstance.nodes.get(customer).tw_b - singleTime; // starting time enough to serve customer before customer's deadline

                if(startTime < this.maxStartingTime) this.maxStartingTime = startTime; 
            }
            if(singleTime > this.makespan) this.makespan = singleTime;
        }

    }

    public DroneSchedule(int src, List<List<Integer>> sequences, double makespan, double maxStartingTime, Set<Integer> customerServed) {
        this.src = src;
        this.sequences = sequences;
        this.makespan = makespan;
        this.maxStartingTime = maxStartingTime;
        this.customerServed = customerServed;
    }

    public int getNumDrone() { return this.sequences.size(); }

    public boolean equals(DroneSchedule other) {
        return this.makespan == other.makespan
            && this.maxStartingTime == other.maxStartingTime;
    }

    public boolean dominates(DroneSchedule other) {
        return !this.equals(other)
            && this.makespan <= other.makespan 
            && this.maxStartingTime >= other.maxStartingTime;
    }
    
}
