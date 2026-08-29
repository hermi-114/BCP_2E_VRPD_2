import java.math.BigInteger;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;


public class DroneSchedule {
    public int src;
    public List<List<Integer>> sequences;
    public double makespan;
    public double maxStartingTime;
    public Set<Integer> customerServed;
    public BigInteger customerServedHashed;
    public double reducedCost = 1;

    public DroneSchedule(int src, List<List<Integer>> sequences) {
        this.src = src;
        this.sequences = sequences;
        this.makespan = -1;
        this.maxStartingTime = Double.MAX_VALUE;
        this.customerServed = new HashSet<>();
        this.customerServedHashed = BigInteger.ZERO;

        for(List<Integer> sequence : sequences) {
            double singleTime = 0;
            for(int customer : sequence) {
                if(this.customerServed.contains(customer))
                    throw new IllegalArgumentException("Customer(s) served more than once");
                
                this.customerServed.add(customer);
                this.customerServedHashed = this.customerServedHashed.setBit(customer);

                double timeEnoughToServingCompleted = singleTime 
                                       + Constant.DRONE_SETUP_TIME
                                       + VRPInstance.distMatrix[src][customer] / Constant.DRONE_SPEED
                                       + VRPInstance.nodes.get(customer).servingTime/2;
                        
                double latestStart = VRPInstance.nodes.get(customer).tw_b - timeEnoughToServingCompleted;

                if(latestStart < this.maxStartingTime) this.maxStartingTime = latestStart;
                
                singleTime = timeEnoughToServingCompleted + VRPInstance.distMatrix[customer][src] / Constant.DRONE_SPEED;
                
            }
            if(singleTime > this.makespan) this.makespan = singleTime;
        }

        if(this.maxStartingTime < 0) throw new IllegalArgumentException("Max starting time must not be negative");

    }

    public DroneSchedule(int src, List<List<Integer>> sequences, double makespan, double maxStartingTime, BigInteger customerServedHashed) {
        this.src = src;
        this.sequences = sequences;
        this.makespan = makespan;
        this.maxStartingTime = maxStartingTime;
        this.customerServed = new HashSet<>();
        this.customerServedHashed = customerServedHashed;

        for(int i = 0; i < customerServedHashed.bitLength(); i++) {
            if(customerServedHashed.testBit(i)) this.customerServed.add(i);
        }
    }

    public DroneSchedule(int src, List<List<Integer>> sequences, double makespan, double maxStartingTime, Set<Integer> customerServed) {
        this.src = src;
        this.sequences = sequences;
        this.makespan = makespan;
        this.maxStartingTime = maxStartingTime;
        this.customerServed = customerServed;

        for(int cust : customerServed) {
            this.customerServedHashed = this.customerServedHashed.setBit(cust);
        }
    }

    public DroneSchedule(int src) {  // empty drone schedule, using in pricing, not in drone schedule enumeration
        this.src = src;
        this.sequences = Collections.emptyList();
        this.makespan = 0.0;
        this.maxStartingTime = Double.MAX_VALUE;
        this.customerServed = Collections.emptySet();
        this.customerServedHashed = BigInteger.ZERO;
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
