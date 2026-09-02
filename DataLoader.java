import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Map;

public class DataLoader {
    
    public static int truckNum;
    public static int droneNum;
    public static int nodeNum;
    public static String instanceName;
    public static Map<Integer, Route> routeMap;


    public static void loadCustomer(String path) {
        System.out.println("Start reading file...");

        int counter = 0;

        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
    
            String line;
    
            while((line=br.readLine()) != null && counter <= Config.SIZE_CUSTOMER_DATASET) {

                if (line.trim().isEmpty()) {
                    continue;
                }

                String[] attrs = line.trim().split("\\s+");

                int id = Integer.parseInt(attrs[0]) - 1;
                double xCoor = Double.parseDouble(attrs[1]);
                double yCoor = Double.parseDouble(attrs[2]);
                int demand = (int)Double.parseDouble(attrs[3]);
                double readyTime = Double.parseDouble(attrs[4]) / 60;
                double deadline = Double.parseDouble(attrs[5]) / 60;
                double truckServiceTime = Double.parseDouble(attrs[6]) / 60; // m -> h

                Node node = new Node(id, xCoor, yCoor, demand, readyTime, deadline, truckServiceTime);

                VRPInstance.nodes.add(node);

                counter++;

            }
            
            if(counter == 0) {
                System.err.println("counter = 0");
                return;
            }

            Constant.TOTAL_CUSTOMER = counter - 1; // customers and 1 depot(id=0)
            Constant.TRUCK_MAX_SHIFT_TIME = VRPInstance.nodes.get(0).tw_b;

            br.close();
            System.out.println("Done read file");
            System.out.println();
            
        } catch(FileNotFoundException e) {
            System.err.println("FILE NOT FOUND: " + '"' + path + '"');
        } catch (Exception e) {
            System.err.println("LOAD DATA ERROR: " + e.getMessage());
        }

    }

    public static void loadFleet() {
        try(BufferedReader br = new BufferedReader(new FileReader("./data/Solomon/capacities.txt"))) {
            
            String type = Config.INPUT_TYPE;
            String set = Config.INPUT_SET;
            String line;

            while((line = br.readLine()) != null) {
                String[] attr = line.split(":");
                if(attr[0].equals(type) && attr[1].compareTo(set) < 0) {
                    Constant.TRUCK_PAYLOAD = Double.parseDouble(attr[2]);
                    Constant.DRONE_AND_EQUIPMENT_WEIGHT = Constant.TRUCK_PAYLOAD / 5;
                } 
            }

            br.close();

        } catch(IOException e) {
            System.err.println("capacities.txt NOT FOUND");
        }
    }
}
