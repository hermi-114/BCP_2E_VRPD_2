import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class Route {
    private static int routeCount = 0; 

    public int id;
    public List<Node> sequence;
    public Map<Integer, DroneSchedule> customerDroneSchedule;
    public double totalTime;
    public double reducedCost = Double.POSITIVE_INFINITY;
    public Set<Integer> customerServed;
    public BigInteger customerServedHashed = BigInteger.ZERO;

    public Route() {}
    
    public Route(List<Node> sequence, Map<Integer, DroneSchedule> customerDroneSchedule) {
        this.id = routeCount++;
        this.sequence = sequence;
        this.customerDroneSchedule = customerDroneSchedule;
        this.totalTime = 0.0;
        this.customerServed = new HashSet<>();

        for(Node cust : sequence) {
            this.customerServed.add(cust.id);
            this.customerServedHashed = this.customerServedHashed.setBit(cust.id);
        }

        for(var schedule : customerDroneSchedule.entrySet()) {
            this.customerServed.addAll(schedule.getValue().customerServed);
            this.customerServedHashed = this.customerServedHashed.or(schedule.getValue().customerServedHashed);
        }

        // 0 - 1 - 2 - 3 - 4 - 0
        sequence.add(0, new Node(0));
        sequence.add(new Node(0));

        for(int i = 1; i < sequence.size(); i++) {
            int curr = sequence.get(i).id;
            int prev = sequence.get(i-1).id;

            double drivingTime = VRPInstance.distMatrix[prev][curr]/Constant.TRUCK_SPEED;
            double servingTime = sequence.get(i).servingTime;
            DroneSchedule schedule = customerDroneSchedule.getOrDefault(curr, null);
            if(schedule != null) servingTime = Math.max(servingTime, schedule.makespan);

            totalTime += drivingTime + servingTime;

        }
    }

    public int getNumDrone() {
        int max = 0;
        for(var customer : sequence) {
            DroneSchedule schedule = customerDroneSchedule.get(customer.id);
            if(schedule == null) continue;
            if(schedule.getNumDrone() > max)
                max = schedule.getNumDrone();
        }

        return max;
    }

    public void setReducedCost(double reducedCost) { this.reducedCost = reducedCost; }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("Route: id=%-4d | sequence=%-15s", id, getSequence())).append(String.format(" | time = %.2f ", totalTime));
        sb.append("\t|| Drones: ");
        for(var schedule : customerDroneSchedule.entrySet()) {
            sb.append("\t").append(schedule.getKey()).append("-").append(schedule.getValue().sequences.toString());
        }
        
        return sb.toString();

    }

    public String getSequence() {
        StringBuilder sb = new StringBuilder();
        sb.append("[0");
        for(int i = 1; i < sequence.size(); i++) sb.append("-").append(sequence.get(i).id);
        sb.append("]");
        return sb.toString();
    }

    public String getSignature() {
        StringBuilder sb = new StringBuilder();
        
        // 1. Truck sequence (including depot at both ends)
        for (Node node : sequence) {
            sb.append(node.id).append(",");
        }
        sb.append("|");
        
        // 2. Drone schedules – sorted by parking node
        List<Integer> keys = new ArrayList<>(customerDroneSchedule.keySet());
        Collections.sort(keys);
        
        for (int key : keys) {
            DroneSchedule schedule = customerDroneSchedule.get(key);
            sb.append(key).append(":");  // parking node
            
            // Sort the sequences lexicographically to ensure order independence
            // for multiple drones, but preserve the order of round‑trips within each drone.
            List<List<Integer>> sortedSequences = new ArrayList<>(schedule.sequences);
            sortedSequences.sort((a, b) -> {
                int min = Math.min(a.size(), b.size());
                for (int i = 0; i < min; i++) {
                    int cmp = Integer.compare(a.get(i), b.get(i));
                    if (cmp != 0) return cmp;
                }
                return Integer.compare(a.size(), b.size());
            });
            
            for (List<Integer> seq : sortedSequences) {
                sb.append("[");
                for (int i = 0; i < seq.size(); i++) {
                    sb.append(seq.get(i));
                    if (i < seq.size() - 1) sb.append(",");
                }
                sb.append("]");
            }
            sb.append("|");
        }
        
        return sb.toString();
    }
    
}
