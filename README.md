# MATLABrosbagReader
A simple rosbag reader to access rosbags easily


The final script is the main file to visualize data/ create plots
>> there are 2 helper functions : rosbagReader and convert2struct

The rosbagReader requires input rosbag name in str format and then generate a struct containing all topics inside the bag:
![image](https://github.com/Autonomousanz/MATLABrosbagReader/assets/64002247/203ebb5b-7f5d-41ec-8b65-b63618187576)

the struct of each rostopic contains the fieldnames:

![image](https://github.com/Autonomousanz/MATLABrosbagReader/assets/64002247/33048164-9983-40dc-a5d8-13bf10c96ba5)

And all the messages in varied timestamps are accumulated inside one entire data:

example linear acceleration X will contain all the X data from start of ros bag to end

![image](https://github.com/Autonomousanz/MATLABrosbagReader/assets/64002247/1b67e317-99fc-49da-bc8a-0366a9843107)

All you need to do in final script to access data taking example of Linear Acceleration :

[name of experiment] = rosbagReader('bagName.bag')

[name_of_experiment].[name_of_topic].LinearAcceleration.X

Note - There is additional Reltime topic to every struct of topic container Header.Stamp.Sec & NSec entries. 
This is nothing but all timestamps - t0 time for ease of plotting the data

MultipleBags Processing script does the following:
1. Processes all ros bags in given rosbag directory and creates one cell array containing all bags
2. velocityPlotter is helper function to plot Husky cmd_vel topics of all bags cell array input
3. plotImuData is helper function to plot Linear Acc X,Y, Z of all bags cell array input
4. plotGps is helper function to plot all lat,lon data in ENU coordinate system 

