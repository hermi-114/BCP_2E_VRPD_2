import java.math.BigInteger;
import java.util.List;

public class DroneSchedule {
    public int src;
    public List<List<Integer>> sequences;
    public double makespan;
    public double maxStartingTime;
    public BigInteger customerServed;

    public DroneSchedule(int src, List<List<Integer>> sequences) {
        this.src = src;
        this.sequences = sequences;
        this.makespan = -1;
        this.maxStartingTime = Double.MAX_VALUE;
        this.customerServed = BigInteger.ZERO;

        for(List<Integer> sequence : sequences) {
            double singleTime = 0;
            for(int customer : sequence) {
                if(this.customerServed.testBit(customer)) throw new IllegalArgumentException("Customer(s) served more than once");
                this.customerServed = this.customerServed.setBit(customer);

                singleTime += Constant.DRONE_SETUP_TIME
                            + VRPInstance.distMatrix[src][customer] / Constant.DRONE_SPEED
                            + VRPInstance.nodes.get(customer).servingTime / 2
                            + VRPInstance.distMatrix[src][customer] / Constant.DRONE_SPEED;
                
                double startTime = VRPInstance.nodes.get(customer).tw_b - singleTime; // starting time enough to serve customer before customer's deadline

                if(startTime < this.maxStartingTime) this.maxStartingTime = Math.min(startTime, VRPInstance.nodes.get(customer).tw_a); 
            }
            if(singleTime > this.makespan) this.makespan = singleTime;
        }

        if(this.maxStartingTime < 0) throw new IllegalArgumentException("MaxStartingTime must not be negative");

    }

    public DroneSchedule(int src, List<List<Integer>> sequences, double makespan, double maxStartingTime, BigInteger customerServed) {
        this.src = src;
        this.sequences = sequences;
        this.makespan = makespan;
        this.maxStartingTime = maxStartingTime;
        this.customerServed = customerServed;
    }

    public int getNumDrone() { return this.sequences.size(); }

    public boolean equals(DroneSchedule other) {
        return Math.abs(this.makespan - other.makespan) < Constant.EPSILON
            && Math.abs(this.maxStartingTime - other.maxStartingTime) <= Constant.EPSILON;
    }

    public boolean dominates(DroneSchedule other) {
        return !this.equals(other)
            && this.makespan <= other.makespan 
            && this.maxStartingTime >= other.maxStartingTime;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("DroneSchedule: [");

        sb.append("numDrones: ").append(sequences.size())
        .append(", makespan: ").append(String.format("%7.3f", makespan))
        .append(", maxStartingTime: ").append(String.format("%7.3f", maxStartingTime));

        sb.append("]");
        return sb.toString();
    }
    
}
