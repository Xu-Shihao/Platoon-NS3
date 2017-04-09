# Platoon-NS3 Matlab NS3 VISSIM7
Author:XU SHIHAO

The aim of this project is to simulate the communication system switching between CCH and SCH of a autonomous vehicle platoon merging and splitting 
in the broadcasting environment

The Dedicated Short-Range Communications (DSRC) standards suite is based on multiple cooperating standards mainly developed by the IEEE.
In particular, we focus on the autonomous vehicle platoon control law and the simulation of platoon networking in Wireless Access in Vehicular Networks (WAVE) model. 
The available spectrum is configured into 1 control channel (CCH) and 6 service channels (SCHs). The CCH is reserved for carrying high-priority short messages or management data,
while other data are transmitted on the SCHs.  In this paper we analyzed the command delay and collision when platoon devices switching
between CCH and SCH during merge and split. And all surrunded vehicle keep broadcasting daily massages simultaneously.

Note:
CPU folder address need to change in matlab code 
The NS3 code need to copy to Scratch Folder in NS3-3.24
Shoud be run in:
./waf --run scratch/Folder_name/Folder_name --command-template="%s 8000"
