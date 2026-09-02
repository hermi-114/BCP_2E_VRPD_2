import java.util.ArrayList;
import java.util.List;


public class VRPInstance {
    
    public static List<Node> nodes = new ArrayList<>();
    public static double[][] distMatrix;
    public static List<Route> routePool = new ArrayList<>();

    public static void calculateDistance() {
        distMatrix = new double[Constant.TOTAL_CUSTOMER + 1][Constant.TOTAL_CUSTOMER + 1];

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
