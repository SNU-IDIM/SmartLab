import os




home_dir = os.getenv("HOME")
current_dir = os.getcwd() 
vis1 = '%s/catkin_ws/src/snu_interface/initialize.launch'%home_dir
vis2 = '%s/initialize.launch'%current_dir
print(vis1)
print(vis2)
if vis1 == vis2:
	print("GGG")
