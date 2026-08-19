import java.util.List;
import java.util.Map;

public class Route {
    private static int routeCount = 0; 

    public int id;
    public final List<Node> sequence;
    public final Map<Integer, DroneSchedule> customerDroneSchedule;
    public double totalTime;
    
    public Route(List<Node> sequence, Map<Integer, DroneSchedule> customerDroneSchedule) {
        this.id = routeCount++;
        this.sequence = sequence;
        this.customerDroneSchedule = customerDroneSchedule;
        totalTime = -1;

        // 0 - 1 - 2 - 3 - 4 - 0
        sequence.add(0, new Node(0));
        sequence.add(new Node(0));
    }

    public int getNumDrone() {
        int max = 0;
        for(var customer : sequence) {
            DroneSchedule schedule = customerDroneSchedule.get(customer.id);
            if(schedule.getNumDrone() > max)
                max = schedule.getNumDrone();
        }

        return max;
    }

    
}
