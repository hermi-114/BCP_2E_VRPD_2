import java.util.ArrayList;
import java.util.List;

public class VRPInstance {
    
    public List<Node> nodes;
    public static double[][] distMatrix;
    public List<Route> routePool;

    public VRPInstance() {
        nodes = new ArrayList<>();
        distMatrix = new double[Constant.TOTAL_CUSTOMER + 1][Constant.TOTAL_CUSTOMER + 1];
        routePool = new ArrayList<>();
    }

    public void addNode(Node node) {
        nodes.add(node);
    }

    public void calculateDistance() {
        for(int i = 0; i < distMatrix.length; i++) {
            for(int j = 0; j < distMatrix.length; j++) {
                if(i == j) distMatrix[i][j] = 1e8;

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
