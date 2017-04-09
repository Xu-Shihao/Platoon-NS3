%% Code responsible for platoon splitting using the PID control algorithm
% This code will be responsible for sending the vehicle position data of
% the platoon which is governed using the control PID control algorithm
% For the control algorithm, we assume perfect communcation. It is only at
% the around 162 seconds when the 
% Doesnt really have to send anything else apart from position information


close all;
clear all;
clc;
Vissim = actxserver('Vissim.Vissim');
Main_Folder = 'E:\NTU\Project\VISSIM platooning\';
Filename = fullfile(Main_Folder, 'vissim-new-1.inp0');
flag_read_additionally  = false; 
Vissim.LoadNet(Filename, flag_read_additionally);

Filename = fullfile(Main_Folder, 'Vissim-New-1.layx');
Vissim.LoadLayout(Filename); 
     
vissim_simulation = Vissim.Simulation;
Random_Seed = 42;
vissim_simulation.set('AttValue', 'RandSeed', Random_Seed);
Period = 1000; 
vissim_simulation.set('AttValue', 'SimPeriod', Period);
Resolution = 10;
vissim_simulation.set('AttValue', 'SimRes', Resolution);
simNet = Vissim.Net;
Platoon_Length = 10;
ms2kmh = 3.6;                                                              
MaxSpeed = 85;                                                             
MinSpeed = 0; 
EndPoint = 1000;                                                          
Platoons = 0;
Lane_Change_Timer = 0;
x_sin = 0:0.1:Period*Resolution;
y_sin = 90+5*sin(2*pi*0.1*x_sin)+5;
plot_sin = [];
p_c = 1;
p_c1 = 1;
platoon_created = 0;
platoon_distance = [];
First_comm = 0; 
Platoon_1 = zeros(Platoon_Length,3);
speed_set = 0;
speed_set3=0;
acc_cnt = 1;
deacc_cnt = 1;
q_m = 1;
comm_iter = 1;
veh_change = [];
dropped_packets = [];
platoons_splitted = 0;
vehicle_input = simNet.VehicleInputs.ItemByKey(1);
vehicle_input.set('AttValue','Volume(1)',0);
StringFromNS3 = [];
Platoon_2=[];


for j=1:simNet.DrivingBehaviors.Count
        simNet.DrivingBehaviors.ItemByKey(j).set('AttValue','CarFollowModType','WIEDEMANN99');
        simNet.DrivingBehaviors.ItemByKey(j).set('AttValue','W99cc0',0);
        simNet.DrivingBehaviors.ItemByKey(j).set('AttValue','W99cc1',0);
end

t1 = tcpip('192.168.56.101', 8000, 'NetworkRole', 'client'); 

% Set size of receiving buffer, if needed. 
set(t1, 'InputBufferSize', 60000);
set(t1, 'OutputBufferSize', 2048); 
% set(t1, 'Timeout', 10);
% Open connection to the server. 
fopen(t1); 

Vissim.Simulation.RunSingleStep;


for i=1:Period*Resolution
    if (uint32(i) ~= uint32(Vissim.Simulation.get('AttValue', 'SimSec') * Resolution))
          continue; 
    end
    Vissim.Simulation.RunSingleStep;
    
    if (i > 50 * Resolution)    
        if (platoon_created == 0)
            for j = 1:Platoon_Length
                for z=1:07
                  Vissim.Simulation.RunSingleStep;
                end
                  vehicle_type = 200;
                  desired_speed = 70;
                  link = 1;
                  lane = 1;
                  position = 00; %0;
                  interaction = true;
                  veh_object(j,1) = simNet.Vehicles.AddVehicleAtLinkPosition(vehicle_type, link, lane,...
                                                                position, desired_speed, interaction);
                  set(veh_object(j,1), 'AttValue', 'DesLane', 1);
            end
                  platoon_created = 1;
        end
        
        for k = 1:length(Platoon_1)
            Platoon_1(k,1) = get(veh_object(k,1), 'AttValue' , 'No');
            Platoon_1(k,2) = get(veh_object(k,1), 'AttValue' , 'Pos');
            Platoon_1(k,3) = get(veh_object(k,1), 'AttValue', 'Speed');
        end
            [~,tmp] = sort(Platoon_1(:,2),'descend');
            Platoon_1 = Platoon_1(tmp,:);
      
        if(speed_set == 0)
            for k = 1:Platoon_Length
                simNet.Vehicles.ItemByKey(Platoon_1(k,1)).set('AttValue','DesSpeed',70);
                simNet.Vehicles.ItemByKey(Platoon_1(k,1)).set('AttValue','Speed',70);
            end
                speed_set = 1;
        end
        if(i == 120*Resolution)
                simNet.Vehicles.ItemByKey(Platoon_1(1,1)).set('AttValue','DesSpeed',70);
                simNet.Vehicles.ItemByKey(Platoon_1(1,1)).set('AttValue','Speed',70);
        end

     String4NS3 = [];
      
       % Wait for all the vehicle to change the lane to the same lane. As soon
       % as that happens we send the forst vehicle platoon information to NS3
       % Have to find a way to check if the vehicles have formed a platoon or
       % not


 %% At T=162, the platoon followers start behaving autonomously and follow the leader and whera laos the communication code will go...Howver now the communciation data is sent to not get 
 % communication metrics for the platoon control algorithm
    if(i > 162*Resolution ) 
        
       %NS3 SECTION   
%       if (~ischar(StringFromNS3))
       if (First_comm == 0) % && all(-diff(Platoon_1(:,2)) > 10))
           %Create the first dataset and send it to NS3
           for k = 1:Platoon_Length
               if(k==1)
                    String4NS3 = strcat(num2str(i/Resolution),'-', num2str(Platoon_Length),'-',num2str(Platoon_1(k,1)), ',', num2str(Platoon_1(k,2)), ',-');
               else
                    String4NS3 = strcat(String4NS3, ',', num2str(Platoon_1(k,1)), ',', num2str(Platoon_1(k,2)), ',-');  
               end
           end
            fprintf(t1,'%s',String4NS3);
            First_comm = 1;
%              plot_sin(p_c,1) = simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Speed');
%             simNet.Vehicles.ItemByKey(Platoon_1(1,1)).set('AttValue','DesSpeed',y_sin(i));
%             simNet.Vehicles.ItemByKey(Platoon_1(1,1)).set('AttValue','Speed',y_sin(i));
       elseif (First_comm == 1)
           % Send the destination waypoints thereafter
           for k = 1:length(Platoon_1)
                String4NS3 = strcat(String4NS3, ',', num2str(Platoon_1(k,1)), ',', num2str(Platoon_1(k,2)), ',-');  
           end
           for k = 1:length(Platoon_2)
                String4NS3 = strcat(String4NS3, ',', num2str(Platoon_2(k,1)), ',', num2str(Platoon_2(k,2)), ',-');  
           end
           fprintf(t1,'%s',String4NS3);
%            plot_sin(p_c,1) = simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Speed');
%            plot_sin(p_c,1) = simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Speed');
%            if ((i > (600 * Resolution)) && (acc_cnt < 200))
%     % 		if(q_m == 1)
%     % 			Vissim.Graphics.CurrentNetworkWindow.set('AttValue','Quickmode',0);
%     % 			q_m = 0;
%     % 		end
%                 simNet.Vehicles.ItemByKey(Platoon_1(1,1)).set('AttValue','DesSpeed',ceil(70+acc_cnt*0.1));
%                 simNet.Vehicles.ItemByKey(Platoon_1(1,1)).set('AttValue','Speed',ceil(70+acc_cnt*0.1));
%                 acc_cnt = acc_cnt + 3;
%      
       else
               continue;
       end
        Simulation_Comms_data = [];
%        for m = 2:Platoon_Length
%             data = [simNet.Vehicles.ItemByKey(Platoon_1(m,1)).get('AttValue','No')...
%                     simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Pos')-simNet.Vehicles.ItemByKey(Platoon_1(m,1)).get('AttValue','Pos') 0];
%             Simulation_Comms_data = vercat(Simulation_Comms_data, data); 
%        end
 
% Receive simulation results from NS3
     pause(3);
            while (get(t1, 'BytesAvailable') > 0) 
        %       DataReceived = fscanf(t1)
                DataReceived = fread(t1,t1.BytesAvailable);
                act_data = DataReceived';
                StringFromNS3 = char(act_data);
                disp(StringFromNS3);
                
             % The following will split the string 
%                  SplitString = strsplit(StringFromNS3,',');
% % 
% %                 NS3_Vehicle_id = [];
% %                 NS3_Vehicle_time = [];
% %                 NS3_RSU_time = [];
% %                 next_RSU = 0;
%                 Transmitter_Matrix = zeros(Platoon_Length,2);
%                 Receiver_Matrix = zeros(Platoon_Length, (Platoon_Length-1)*2);
%                 row_T = 0;
% %                 row_R  =1;
%                
%                 for kk = 1:length(SplitString)-1
% 
%                     StringSplit2 = strsplit(SplitString{kk},':');
%                     if (~isempty(strfind(char(StringSplit2{1}),'RSU')))
%                        row_T = row_T + 1; 
%                        col_R = 0 ;
%                        transmitter = strsplit(StringSplit2{1},' ');
%                        Transmitter_Matrix(row_T,1) = str2double(transmitter{2});
%                        Transmitter_Matrix(row_T,2) = str2double(StringSplit2{2});    
%                     else
%                         col_R = col_R + 1;
%                         Receiver_Matrix(row_T,col_R) = str2double(StringSplit2{1});
%                         col_R = col_R + 1;
%                         Receiver_Matrix(row_T,col_R) = str2double(StringSplit2{2});           
%                     end
% 
%                 end
                
            end
    
%       end
            for h=2:length(Platoon_1)
            
                v_leader = simNet.Vehicles.ItemByKey(Platoon_1(h-1,1)).get('AttValue','Speed');
                v_follower = simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','Speed');
                v_header = simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Speed');
                      
                pos_leader = simNet.Vehicles.ItemByKey(Platoon_1(h-1,1)).get('AttValue','Pos');
                pos_follower = simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','Pos');
                pos_header= simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Pos');
 
                Acc_leader = simNet.Vehicles.ItemByKey(Platoon_1(h-1,1)).get('AttValue','Acceleration');
                Acc_follower = simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','Acceleration');
                Acc_header = simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Acceleration');
       
                Gap_des2 = 5 + 10.22;
                Gap_des1 = (h-1)*Gap_des2;
                          
            
                kp1=800;ki1=2250;kd1=200;
                kp2=800;ki2=2250;kd2=200;
 
                rho=1.2;fr=0.01;g=9.81;cdrag=0.4;af=1.2;cair=rho*af*cdrag;
                
                t=0.1;
                desired_acceleration = (kd2*(Acc_header+Acc_leader)+kp2*(v_header-v_follower)+kp1*(v_leader-v_follower)+ki2*(pos_header-pos_follower-Gap_des1)+ki1*(pos_leader-pos_follower-Gap_des2))/(cair*v_follower+2*kd1);
                        
                desired_speed = 0.1 * desired_acceleration + v_follower;         
                                                                                    
                simNet.Vehicles.ItemByKey(Platoon_1(h,1)).set('AttValue','DesSpeed',desired_speed);
                simNet.Vehicles.ItemByKey(Platoon_1(h,1)).set('AttValue','Speed',desired_speed);
                
                plot_sin(p_c,h) = simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','Speed');
                Position_error(p_c,h) =  simNet.Vehicles.ItemByKey(Platoon_1(h-1,1)).get('AttValue','Pos') - simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','Pos') ; 
                Position_error2(p_c,h) = simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Pos') - simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','Pos');

            end      

          
            
            
            
%% The following is supposed to happen only when the communication protocol finishes
% So the if condition, instead of time dependent will now becoime event
% dependent where the event in this case is whether the whole set
% of message exchanges actually took place or not 
    if (ischar(StringFromNS3))
%         if h>8
       
            speed_set2=0;   
            splitcar=7;
            
            for y = splitcar:Platoon_Length
                p = (y-splitcar+1);
                Platoon_2(p,1) = get(veh_object(y,1), 'AttValue' , 'No');
                Platoon_2(p,2) = get(veh_object(y,1), 'AttValue' , 'Pos');
                Platoon_2(p,3) = get(veh_object(y,1), 'AttValue', 'Speed');
            end
                [~,tmp] = sort(Platoon_2(:,2),'descend');
                Platoon_2 = Platoon_2(tmp,:);
                
                if (platoons_splitted == 0)
                   Platoon_1 = Platoon_1(1:splitcar-1,:);
                   platoons_splitted = 1;
                end
       
             
        if ((deacc_cnt < 200) && i< (299* Resolution))
                  simNet.Vehicles.ItemByKey(Platoon_2(1,1)).set('AttValue','DesSpeed',ceil(70-deacc_cnt*0.1));
                  simNet.Vehicles.ItemByKey(Platoon_2(1,1)).set('AttValue','Speed',ceil(70-deacc_cnt*0.1));
                  deacc_cnt = deacc_cnt + 1;
        end
                 
        if i > (300* Resolution) && (acc_cnt < 600)
                      simNet.Vehicles.ItemByKey(Platoon_2(1,1)).set('AttValue','DesSpeed',ceil(50+acc_cnt*0.1));
                      simNet.Vehicles.ItemByKey(Platoon_2(1,1)).set('AttValue','Speed',ceil(50+acc_cnt*0.1));
                      acc_cnt = acc_cnt + 1;
                    end
             
            for m=2:length(Platoon_2)
                                         

                        v_header1 = simNet.Vehicles.ItemByKey(Platoon_2(1,1)).get('AttValue','Speed');  % NO 9
                        v_leader1 = simNet.Vehicles.ItemByKey(Platoon_2(m-1,1)).get('AttValue','Speed');
                        v_follower1 = simNet.Vehicles.ItemByKey(Platoon_2(m,1)).get('AttValue','Speed');
    
                        pos_header1= simNet.Vehicles.ItemByKey(Platoon_2(1,1)).get('AttValue','Pos');
                        pos_leader1 = simNet.Vehicles.ItemByKey(Platoon_2(m-1,1)).get('AttValue','Pos');
                        pos_follower1 = simNet.Vehicles.ItemByKey(Platoon_2(m,1)).get('AttValue','Pos');
  
                        Acc_header1 = simNet.Vehicles.ItemByKey(Platoon_2(1,1)).get('AttValue','Acceleration');
                        Acc_leader1 = simNet.Vehicles.ItemByKey(Platoon_2(m-1,1)).get('AttValue','Acceleration');
                        Acc_follower1 = simNet.Vehicles.ItemByKey(Platoon_2(m,1)).get('AttValue','Acceleration');
    
                        Gap_des21 = 5 + 10.22;
                        Gap_des11 = (m-1)*Gap_des21;
                   
                        kp1=800;ki1=2250;kd1=200;
                        kp2=800;ki2=2250;kd2=200;
 
                        rho=1.2;fr=0.01;g=9.81;cdrag=0.4;af=1.2;cair=rho*af*cdrag;

                        t=0.1;
                                               
                            desired_acceleration1 = (kd2*(Acc_header1+Acc_leader1)+kp2*(v_header1-v_follower1)+kp1*(v_leader1-v_follower1)+ki2*(pos_header1-pos_follower1-Gap_des11)+ki1*(pos_leader1-pos_follower1-Gap_des21))/(cair*v_follower1+2*kd1);
                            desired_speed12 = 0.1 * desired_acceleration1 + v_follower1 ;%simulation step = 0.1s         
                                     
                            simNet.Vehicles.ItemByKey(Platoon_2(m,1)).set('AttValue','DesSpeed',desired_speed12);
                            simNet.Vehicles.ItemByKey(Platoon_2(m,1)).set('AttValue','Speed',desired_speed12);
%                         
                              plot_sin1(p_c,m) = simNet.Vehicles.ItemByKey(Platoon_2(m,1)).get('AttValue','Speed');
                              position_error1(p_c,m) =  simNet.Vehicles.ItemByKey(Platoon_1(m-1,1)).get('AttValue','Pos') - simNet.Vehicles.ItemByKey(Platoon_1(m,1)).get('AttValue','Pos') ; 
                                   
                                    for pl1 = 1: length(Platoon_1)
                                             First_Platoon1(p_c,pl1) = simNet.Vehicles.ItemByKey(Platoon_1(pl1,1)).get('AttValue','Speed');
                                    end     
                                    for pl2 = 1: length(Platoon_2)
                                   
                                        First_Platoon2(p_c,pl2) = simNet.Vehicles.ItemByKey(Platoon_2(pl2,1)).get('AttValue','Speed');
                                    end 
                                    %Platoon_finale = [First_Platoon1 ; First_Platoon2];
                                                           
                                 for y = splitcar:Platoon_Length
                                        if i> (250* Resolution)
                                                                set(veh_object(y,1), 'AttValue', 'DesLane', 2)
                                        if i> (500* Resolution)
                                                                set(veh_object(y,1), 'AttValue', 'DesLane', 1)
                                         end
                                        end
                                 end
                           
                end
                          
    end
           
                p_c = p_c + 1;
        end
    end
    end
     
% end
 

Vissim.Simulation.Stop;
fclose(t1);
delete(t1);
clc


% 
% Position_error2(p_c,m) = simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Pos') - simNet.Vehicles.ItemByKey(Platoon_1(m,1)).get('AttValue','Pos');
%                          
%        Position_error1(p_c,m) =  simNet.Vehicles.ItemByKey(Platoon_1(m-1,1)).get('AttValue','Pos') - simNet.Vehicles.ItemByKey(Platoon_1(m,1)).get('AttValue','Pos') ; 
%      Position_error21(p_c,m) = simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Pos') - simNet.Vehicles.ItemByKey(Platoon_1(m,1)).get('AttValue','Pos');
%       plot_sin1(p_c,m) = simNet.Vehicles.ItemByKey(Platoon_1(m,1)).get('AttValue','Speed');

% 
