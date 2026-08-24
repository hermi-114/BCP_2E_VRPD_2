public class Node {
    public int id;
    public double x,y;
    public int demand;
    public boolean canServedByDrone;
    public double tw_a;
    public double tw_b;
    public double servingTime = 1.5; // h 1.5h (1h30) for truck, 0.75h (45m) for drone

    public Node(int id) { // depot;
        this.id = id;
    }

    public Node(int id, double x, double y, int demand, double tw_a, double tw_b) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.demand = demand;
        this.tw_a = tw_a;
        this.tw_b = tw_b;
        canServedByDrone = true;
    }
}
