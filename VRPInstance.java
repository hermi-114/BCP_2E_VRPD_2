import java.util.ArrayList;
import java.util.List;


public class VRPInstance {
    public Node depot;
    public List<Node> nodes;
    public double[][] distMatrix;

    public VRPInstance() {
        nodes = new ArrayList<>();
        distMatrix = new double[Constant.MAX_VEHICLE][Constant.MAX_VEHICLE];

    }

    public void setDepot(Node depot) { this.depot = depot; }

    public void addNode(Node node) {
        if(nodes.contains(node)) return;

        nodes.add(node);
    }

    public void calculateDistance() {
        for(int i = 0; i < distMatrix.length; i++) {
            for(int j = 0; j < distMatrix.length; j++) {
                if(i == j) continue;

                Node src = nodes.get(i);
                Node dst = nodes.get(j);

                double x = src.x - dst.x;
                double y = src.y - dst.y;

                double dist = Math.sqrt(x*x + y*y);
                distMatrix[i][j] = dist;
            }
        }
    }
}
