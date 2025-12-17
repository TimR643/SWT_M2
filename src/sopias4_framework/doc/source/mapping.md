# Mapping and using maps
In general, the map is provided by the running Sopias Fleetbroker instance. This instance usually runs on the PC in the lab, but in theory it can be run from any Dev Container which is in the network. You only have to make sure that it is running once.

A static map is saved with two files which must be in the same directory with the same name. The first file is a YAML file which contains metadata about the map which tells the map server how and where to place the static map in the virtual environment. The second file is the pgm file and holds the actual map as a image. You can also edit this map with e.g. gimp as long as you only change the pixels i.e. don't make the image bigger/smaller.

Sometimes you need to create new maps or load another static map. In the further section these processes are described. In the advanced information of the documentation you can find additional information which should only be necessary for the supervisor.

## Create a map using SLAM
Some general information regarding the mapping process:
- Multiple TurtleBots can run the SLAM simultaneously because each SLAM publishes the created map in the topic `/<namespace of robot>/map`. In this topic you can see the progress of the mapping process
- The created map is saved onto the running Sopias4 Fleetbroker instance and its filesystem. This means:
  - The path for saving the map refers to the file system of Sopias4 Fleetbroker. However, if you use the Dev Container, then the structure of the file system is the same. This allows to use e.g. a file picker on Sopias4 Application to choose a directory because they exist on both file systems (except directories which where created manually in either system)
  - The map as a file the exists only on Sopias4 Fleetbroker and not on the Sopias4 Application which runs the slam
- Each mapping process creates a YAML file with the meta data of the map and a pgm file which holds the actual map as a image

The process of creating a map is named mapping. Sopias4 provides SLAM which can be either started with the corresponding launch file or with the methods inside the `gui_node.py` which you can use to build your own gui. For mapping, you have to follow these steps:
1. Make sure your Sopias4 Application and one Sopias4 Fleetbroker instance is running
2. Start SLAM by either using the launch file `bringup_slam.launch.py` or the method from the gui method
3. Wait until everything started. Then drive the Turtlebot around until you are satisfied with your map
4. Save the map by either:
      - Calling the corresponding service `nav2_msgs/srv/SaveMap` (you can also use the service `sopias4_msgs/srv/<StopMapping` which is a wrapper for this)
      - Use the `stop_mapping()` method inside the `GUINode` base class which calls the service under the hood for you
   
   Either methods needs the parameter `map name` (named `map path` in the `stop_mapping()` method) which is the complete path to where the map should be saved, so its the path to the directory and the name of the map (remember that this path refers to the file system of the running Sopias4 Fleetbroker instance). Usually the map is saved to the `src/sopias4_fleetbroker/maps` directory.

   The `map topic` parameter should be the topic where the created map which should be saved is published. This topic is usually `/<namespace of robot>/map`. All the other parameters should only be changed if you know what you are doing and should'nt be touched otherwise.

## Load a existing map
Some general information regarding loading a map:
- The map under `src/sopias4_fleetbroker/maps/default_map.yaml` is loaded as a default at startup. If you want a new standard map, then make a backup of the old one and save the new one under this path 
- The map saved in the `src/sopias4_fleetbroker/maps` are saved in a persistent way so they don't get lost if you rebuild the project. In theory you can also save the map into the `install/sopias4_fleetbroker/share/sopias4_fleetbroker/maps` directory, however these files get deleted and overwritten with the maps from the first mentioned path if the project is rebuild
- The map you load must be located in the file system of the running Sopias4 Fleetbroker instance

To load a map, you have to call the `nav2_msgs/srv/LoadMap` service. The only needed parameter is the full path to the map you want to load (remember that the path to the map refers to the filesystem of the running Sopias4 Fleetbroker instance). You have to refer the YAML file in the path. The GUI of Sopias4 Fleetbroker provides a GUI to do this. If the GUI runs in the container of the running instance, then you can use the file picker button, if you run it in another container/system, then you have to enter the path manually.