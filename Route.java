import java.math.BigInteger;
import java.util.*;

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

        for (Node cust : sequence) {
            this.customerServed.add(cust.id);
            this.customerServedHashed = this.customerServedHashed.setBit(cust.id);
        }
        for (var s : customerDroneSchedule.entrySet()) {
            this.customerServed.addAll(s.getValue().customerServed);
            this.customerServedHashed = this.customerServedHashed.or(s.getValue().customerServedHashed);
        }

        sequence.add(0, new Node(0));
        sequence.add(new Node(0));

        for (int i = 1; i < sequence.size(); i++) {
            int curr = sequence.get(i).id;
            int prev = sequence.get(i - 1).id;
            double drivingTime = VRPInstance.distMatrix[prev][curr] / Constant.TRUCK_SPEED;
            double servingTime = sequence.get(i).servingTime;
            DroneSchedule sch = customerDroneSchedule.getOrDefault(curr, null);
            if (sch != null) servingTime = Math.max(servingTime, sch.makespan);
            totalTime += drivingTime + servingTime;
        }
    }

    public int getNumDrone() {
        int max = 0;
        for (var customer : sequence) {
            DroneSchedule s = customerDroneSchedule.get(customer.id);
            if (s == null) continue;
            if (s.getNumDrone() > max) max = s.getNumDrone();
        }
        return max;
    }

    public void setReducedCost(double reducedCost) { this.reducedCost = reducedCost; }

    // ---- branching helpers ----

    public boolean usesTruckArc(int i, int j) {
        for (int k = 1; k < sequence.size(); k++)
            if (sequence.get(k - 1).id == i && sequence.get(k).id == j) return true;
        return false;
    }

    public double coefficientTotalTrucks()  { return 1.0; }
    public double coefficientTotalDrones()  { return getNumDrone(); }
    public double coefficientTrucksWithD(int d) { return getNumDrone() == d ? 1.0 : 0.0; }
    public double coefficientTruckArc(int i, int j) { return usesTruckArc(i, j) ? 1.0 : 0.0; }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("Route: id=%-4d | seq=%-15s | time=%.2f ",
                id, getSequence(), totalTime));
        sb.append("\t|| Drones: ");
        for (var s : customerDroneSchedule.entrySet())
            sb.append("\t").append(s.getKey()).append("-").append(s.getValue().sequences);
        return sb.toString();
    }

    public String getSequence() {
        StringBuilder sb = new StringBuilder("[0");
        for (int i = 1; i < sequence.size(); i++) sb.append("-").append(sequence.get(i).id);
        sb.append("]");
        return sb.toString();
    }

    public String getSignature() {
        StringBuilder sb = new StringBuilder();
        for (Node n : sequence) sb.append(n.id).append(",");
        sb.append("|");
        List<Integer> keys = new ArrayList<>(customerDroneSchedule.keySet());
        Collections.sort(keys);
        for (int key : keys) {
            DroneSchedule s = customerDroneSchedule.get(key);
            sb.append(key).append(":");
            List<List<Integer>> sorted = new ArrayList<>(s.sequences);
            sorted.sort((a, b) -> {
                int m = Math.min(a.size(), b.size());
                for (int i = 0; i < m; i++) {
                    int c = Integer.compare(a.get(i), b.get(i));
                    if (c != 0) return c;
                }
                return Integer.compare(a.size(), b.size());
            });
            for (List<Integer> seq : sorted) {
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