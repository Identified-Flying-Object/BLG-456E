Includes new hector quadrotor design and sensors.

Sensors included directly:
1) Front facing Hokuyo UTM-30LX mounted above the quadrotor body (Use rfiz to see data)
2) Bottom facing OV7670 RGB camera mounted front of the quadrotor body (Use rfiz to see data)
3) Bottom facing LV-EZ4 sonar sensor mounted back of the quadrotor body (Type "rostopic echo /sonar_height" in terminal to see data)

Sensors included via plugins which can be useful:
1) Barometer (Type "rostopic echo /pressure_height" in terminal to see data)
2) Magnetometer (Type "rostopic echo /magnetic" in terminal to see data)
3) GPS (Type "rostopic echo /fix" in terminal to see data)




