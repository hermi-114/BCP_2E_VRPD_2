public class Node {
    public int id;
    public double x,y;
    public int demand;
    public boolean canServedByDrone;
    public double tw_a;
    public double tw_b;

    public Node(int id, double x, double y, int demand) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.demand = demand;
        canServedByDrone = true;
    }
}
