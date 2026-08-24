import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.Map;

public class DataLoader {
    
    public static int truckNum;
    public static int droneNum;
    public static int nodeNum;
    public static String instanceName;
    public static Map<Integer, Route> routeMap;


    public static void load(String path) {
        System.out.println("Start reading file...");

        int counter = 0;

        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
    
            String line;
    
            while((line=br.readLine()) != null) {

                if (line.trim().isEmpty()) {
                    continue;
                }

                counter++;

                String[] attrs = line.trim().split("\\s+");

                int id = Integer.parseInt(attrs[0]);
                double xCoor = Double.parseDouble(attrs[1]);
                double yCoor = Double.parseDouble(attrs[2]);
                int demand = (int)Double.parseDouble(attrs[3]);
                double readyTime = Double.parseDouble(attrs[4]);
                double deadline = Double.parseDouble(attrs[5]);
                // double truckServiceTime = Double.parseDouble(attrs[6]);

                Node node = new Node(id-1, xCoor, yCoor, demand, readyTime, deadline);

                VRPInstance.nodes.add(node);

            }
            
            if(counter == 0) {
                System.err.println("counter = 0");
                return;
            }

            Constant.TOTAL_CUSTOMER = counter - 1; // customers and 1 depot(id=0)

            br.close();
            System.out.println("Done read file");
            System.out.println();
            
        } catch(FileNotFoundException e) {
            System.err.println("FILE NOT FOUND: " + '"' + path + '"');
        } catch (Exception e) {
            System.err.println("LOAD DATA ERROR: " + e.getMessage());
        }

    }
}
