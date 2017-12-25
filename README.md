BLG456E Project - Aider Drone

The drone we used is hector quadrotor. 
Drone takes a package which contains first aid equipments such as food, water, medicine or first aid kits and brings them to the emergency area. The aid package will be taken from the base by the hook on the drone. Then, the package will be dropped off when the marked area is recognized.


 Note: Add the lines below into launch file to publis joint states to fix the problem in Rviz.
 
 \<!-- For non-fixed joints publish joint states -->
 
 \<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" >\</node>
