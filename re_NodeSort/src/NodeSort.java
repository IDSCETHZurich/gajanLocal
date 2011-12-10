import ros.*;
import ros.communication.*;

public class NodeSort {
    public static void main(String args[]) 
    throws InterruptedException, RosException {
        final Ros ros = Ros.getInstance();
        ros.init("Listner");
        
        NodeHandle n = ros.createNodeHandle(); 
        
        final Publisher<ros.pkg.re_kinect_object_detector.msg.OrderedList> pub =
        	       n.advertise("re_kinect/orderedList", new ros.pkg.re_kinect_object_detector.msg.OrderedList(), 1);
        
        Subscriber.Callback<ros.pkg.re_kinect_object_detector.msg.DetectionResult> callback = 
        new Subscriber.Callback<ros.pkg.re_kinect_object_detector.msg.DetectionResult>() {
            public void call(ros.pkg.re_kinect_object_detector.msg.DetectionResult msg) {
                ros.logInfo("Received Msg"); 
                java.util.ArrayList<java.lang.String> DetectedObjectList = msg.DetectedObjectList;
                java.util.ArrayList<java.lang.String> FullObjectList = msg.FullObjectList;
                
                for(int i=0; i<DetectedObjectList.size();i++){
                	ros.logInfo(DetectedObjectList.get(i));
                }
                
                for(int i=0; i<FullObjectList.size();i++){
                	ros.logInfo(FullObjectList.get(i));
                }
                
                // Okan here is where your code goes
                
                int[] FullOrderedObjectList = new int[FullObjectList.size()];
                
                ros.pkg.re_kinect_object_detector.msg.OrderedList ol = new ros.pkg.re_kinect_object_detector.msg.OrderedList();
                ol.FullOrderedObjectList = FullOrderedObjectList;
                
                pub.publish(ol);
                
            }
        };    
        
        Subscriber<ros.pkg.re_kinect_object_detector.msg.DetectionResult> sub; 
        sub = n.subscribe("re_kinect/detection_results", 
                          new ros.pkg.re_kinect_object_detector.msg.DetectionResult(), 
                          callback, 
                          1);
        
        n.spin();
    }
}
