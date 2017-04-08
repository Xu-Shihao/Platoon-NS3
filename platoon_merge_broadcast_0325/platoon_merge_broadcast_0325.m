close all;
clear all;
clc;

Vissim = actxserver('Vissim.Vissim');
Main_Folder = 'C:\Users\xushi\Desktop\CPU\';
Filename = fullfile(Main_Folder, 'vissim-new-1.inpx');
flag_read_additionally  = false; 
Vissim.LoadNet(Filename, flag_read_additionally);

Filename = fullfile(Main_Folder, 'Vissim-New-1.layx');
Vissim.LoadLayout(Filename);
% Vissim.Graphics.CurrentNetworkWindow.set('AttValue','Quickmode',1);
     
vissim_simulation = Vissim.Simulation;
Random_Seed = 42;
vissim_simulation.set('AttValue', 'RandSeed', Random_Seed);
Period = 300; %600;
vissim_simulation.set('AttValue', 'SimPeriod', Period);
Resolution = 10;
vissim_simulation.set('AttValue', 'SimRes', Resolution);
simNet = Vissim.Net;
Platoon_Length1 = 7;
Platoon_Length2 = 3;
Platoon_Length = Platoon_Length1 + Platoon_Length2;
ms2kmh = 3.6;                                                              %3.6 m/s = 1 km/h
MaxSpeed = 85;                                                             %Unit: km/h
MinSpeed = 0; 
StartPoint = 0.5;                                                          %Position of vehicle input in lane 1
EndPoint = 1000;                                                          %Position of Signal Header (1) in lane 1
Platoons = 0;
% Platoon_1 = [];
Lane_Change_Timer = 0;
x_sin = 0:0.1:Period*Resolution;
%x_sin = 0:1:Period*Resolution;
y_sin = 90+5*sin(2*pi*0.1*x_sin)+5;
merge_signal=0;
merge_moment=0; % The time NS3 send back merge signal
%y_sin = 90+5*sin(x_sin)+5;
plot_sin_1 = [];
plot_sin_2 = [];
p_c = 1;
platoon_created = 0;
platoon_distance = [];
First_comm = 0; 
Platoon_1 = zeros(Platoon_Length1,3);
Platoon_2 = zeros(Platoon_Length2,3);
speed_set = 0;
acc_cnt = 0;
deacc_cnt = 0;
q_m = 1;
comm_iter = 1;
veh_change = [];
dropped_packets = [];
Overall_Comms_data = [];
vehicle_input = simNet.VehicleInputs.ItemByKey(1);
vehicle_input.set('AttValue','Volume(1)',600);


for j=1:simNet.DrivingBehaviors.Count
        simNet.DrivingBehaviors.ItemByKey(j).set('AttValue','CarFollowModType','WIEDEMANN99');
        simNet.DrivingBehaviors.ItemByKey(j).set('AttValue','W99cc0',0);
         simNet.DrivingBehaviors.ItemByKey(j).set('AttValue','W99cc1',0);
end
%%  COMMENTED NS3 SECTION 1
t1 = tcpip('192.168.56.101', 8000, 'NetworkRole', 'client'); 

% Set size of receiving buffer, if needed. 
set(t1, 'InputBufferSize', 60000);
set(t1, 'OutputBufferSize', 2048); 
% set(t1, 'Timeout', 10);
% Open connection to the server. 
fopen(t1); 


%%
Vissim.Simulation.RunSingleStep;


for i=1:Period*Resolution

    if (uint32(i) ~= uint32(Vissim.Simulation.get('AttValue', 'SimSec') * Resolution))
          continue; 
    end
    
    Vissim.Simulation.RunSingleStep;    
    
  % Create platoon after 50 simulation seconds  
  if (i > 50 * Resolution)  
   
      
 %% Platoon Creation  
       if (platoon_created == 0)

%%%%%%%%%%%%%%%%% CHANGE LANE FOR ALL VEHICLES THAT ARE AT THE FRONT OF PLATOON    
            All_Vehicles = simNet.Vehicles.GetAll;
            for cnt_Veh = 1 : length(All_Vehicles)                                 
                veh_number     = get(All_Vehicles{cnt_Veh}, 'AttValue', 'No');
                veh_type       = get(All_Vehicles{cnt_Veh}, 'AttValue', 'VehType');
                veh_speed      = get(All_Vehicles{cnt_Veh}, 'AttValue', 'Speed');
                veh_position   = get(All_Vehicles{cnt_Veh}, 'AttValue', 'Pos');
                veh_linklane   = get(All_Vehicles{cnt_Veh}, 'AttValue', 'Lane');

                VehType(cnt_Veh) = str2num(veh_type);
                VehNumber(cnt_Veh) = veh_number;
                VehSpeed(cnt_Veh) = veh_speed;
                VehPosition(cnt_Veh) = veh_position;
                VehLinklane(cnt_Veh) = cellstr(veh_linklane);  
                VehLink(cnt_Veh) = str2double(strtok(VehLinklane(cnt_Veh), '-'));
                [tmp1, tmp2] = strtok(VehLinklane(cnt_Veh), '-');
                VehLane(cnt_Veh) = str2double(strtok(tmp2, '-'));
            end

            veh_change = find(VehLane == 1 & VehType == 100 & VehLink == 1);

            
            % Code for changing the lane for all the non-platoon vehicles
            if((length(veh_change) > 0) && (i < 70*Resolution))
                for j=1:length(veh_change)
                    simNet.Vehicles.ItemByKey(VehNumber(veh_change(j))).set('AttValue','DesLane','2');
                end
            end

        %%%%%%%%%%%%%%%%%	

                  for j = 1:Platoon_Length1

                      for z=1:7
                        Vissim.Simulation.RunSingleStep;
                      end
                      vehicle_type = 200;
                      desired_speed = 70;
                      link = 1;
                      lane = 1;
                      position = 50; %0;
                      interaction = true;
                      veh_object(j,1) = simNet.Vehicles.AddVehicleAtLinkPosition(vehicle_type, link, lane,...
                                                                position, desired_speed, interaction);

                    % Set the desired lane of all the vehicles to be lane 1
                      set(veh_object(j,1), 'AttValue', 'DesLane', 1);
                  end
                  platoon_created = 1;
       end
       if(platoon_created == 1)
            for j = 1:Platoon_Length2
                for b=1:7
                    Vissim.Simulation.RunSingleStep;
                end
                  vehicle_type = 200;
                  desired_speed1 = 60;
                  link = 1;
                  lane = 1;
                  position = 50; %0;
                  interaction = true;
                  veh_object1(j,1) = simNet.Vehicles.AddVehicleAtLinkPosition(vehicle_type, link, lane,...
                                                                position, desired_speed1, interaction);
                  set(veh_object1(j,1), 'AttValue', 'DesLane', 1);
            end
            platoon_created = 2;
        end
       
      for k = 1:Platoon_Length1
      
          Platoon_1(k,1) = get(veh_object(k,1), 'AttValue' , 'No');
          Platoon_1(k,2) = get(veh_object(k,1), 'AttValue' , 'Pos');
          Platoon_1(k,3) = get(veh_object(k,1), 'AttValue', 'Speed');
      
      end
       for a = 1:Platoon_Length2
          Platoon_2(a,1) = get(veh_object1(a,1), 'AttValue' , 'No');
          Platoon_2(a,2) = get(veh_object1(a,1), 'AttValue' , 'Pos');
          Platoon_2(a,3) = get(veh_object1(a,1), 'AttValue', 'Speed');
       end
      [~,tmp] = sort(Platoon_1(:,2),'descend');
      [~,tmp1] = sort(Platoon_2(:,1),'ascend');
      Platoon_1 = Platoon_1(tmp,:);
      Platoon_2 = Platoon_2(tmp1,:);
      
	  if(speed_set == 0)
		for k = 1:Platoon_Length1
			simNet.Vehicles.ItemByKey(Platoon_1(k,1)).set('AttValue','DesSpeed',70);
			simNet.Vehicles.ItemByKey(Platoon_1(k,1)).set('AttValue','Speed',70);
		end
		speed_set = 1;
      end
       if(speed_set == 1)
          for c = 1:Platoon_Length2
            simNet.Vehicles.ItemByKey(Platoon_2(c,1)).set('AttValue','DesSpeed',67);
            simNet.Vehicles.ItemByKey(Platoon_2(c,1)).set('AttValue','Speed',67);
          end
          speed_set = 2;
       end

      % At 150 seconds making all the vehicles to stop moving
      if(i == 150*Resolution)
			simNet.Vehicles.ItemByKey(Platoon_1(1,1)).set('AttValue','DesSpeed',0);
			%simNet.Vehicles.ItemByKey(Platoon_1(1,1)).set('AttValue','Speed',0);
            simNet.Vehicles.ItemByKey(Platoon_2(1,1)).set('AttValue','DesSpeed',0);
           % simNet.Vehicles.ItemByKey(Platoon_2(2,1)).set('AttValue','DesSpeed',0);
            
			%simNet.Vehicles.ItemByKey(Platoon_2(1,1)).set('AttValue','Speed',0);
      end
      % At 220 they start moving again
      if(i == 220*Resolution)
		 for k = 1:Platoon_Length1
            simNet.Vehicles.ItemByKey(Platoon_1(k,1)).set('AttValue','DesSpeed',70);
			simNet.Vehicles.ItemByKey(Platoon_1(k,1)).set('AttValue','Speed',70);
         end
         for c = 1:Platoon_Length2
            simNet.Vehicles.ItemByKey(Platoon_2(c,1)).set('AttValue','DesSpeed',70);
			simNet.Vehicles.ItemByKey(Platoon_2(c,1)).set('AttValue','Speed',70);
         end
%     
      end
    

    
 %% Begin network simulations after 225 seconds   
    if(i > 225*Resolution ) %&&  i < 700 * Resolution) 
        
    String4NS3 = [];
      
       % Wait for all the vehicle to change the lane to the same lane. As soon
       % as that happens we send the first vehicle platoon information to NS3
       % Have to find a way to check if the vehicles have formed a platoon or
       % not

%% NS3 SECTION 2    
       if (First_comm == 0) % && all(-diff(Platoon_1(:,2)) > 10))
           %Create the first dataset and send it to NS3
           for m = 1:Platoon_Length1
               if(m==1)
                    String4NS3 = strcat(num2str(i/Resolution),'-', num2str(Platoon_Length1),'-',num2str(Platoon_1(m,1)), ',', num2str(Platoon_1(m,2)), ',-');
               else
                    String4NS3 = strcat(String4NS3, ',', num2str(Platoon_1(m,1)), ',', num2str(Platoon_1(m,2)), ',-');  
               end
           end
            fprintf(t1,'%s',String4NS3);
             for l = 1:Platoon_Length2
               if(l==1)
                   String4NS3 = strcat(num2str(i/Resolution),'-', num2str(Platoon_Length2),'-',num2str(Platoon_2(l,1)), ',', num2str(Platoon_2(l,2)), ',-');
               else
                   String4NS3 = strcat(String4NS3, ',', num2str(Platoon_2(l,1)), ',', num2str(Platoon_2(l,2)), ',-');
               end
             end
            First_comm = 1;
            fprintf(t1,'%s',String4NS3);
%             
       elseif (First_comm == 1)
      
           for n = 1:Platoon_Length1
                String4NS3 = strcat(String4NS3, ',', num2str(Platoon_1(n,1)), ',', num2str(Platoon_1(n,2)), ',-');  
           end
           fprintf(t1,'%s',String4NS3);
           String4NS3=[];
           for b = 1:Platoon_Length2
                String4NS3 = strcat(String4NS3, ',', num2str(Platoon_2(b,1)), ',', num2str(Platoon_2(b,2)), ',-');  
           end
           fprintf(t1,'%s',String4NS3);
           
       else
               continue;
       end
        Simulation_Comms_data = [];
%      
%% NS3 SECTION
% Receive simulation results from NS3
     pause(1);
           while (get(t1, 'BytesAvailable') > 0) 
        %       DataReceived = fscanf(t1)
                DataReceived = fread(t1,t1.BytesAvailable);
                act_data = DataReceived';
                StringFromNS3 = char(act_data);
                
             % The following will split the string 
                SplitString = strsplit(StringFromNS3,',');     
                Transmitter_Matrix = zeros(Platoon_Length,2);
                Receiver_Matrix = zeros(Platoon_Length, (Platoon_Length-1)*2);
                row_T = 0;
%                 row_R  =1;
               
                for kk = 1:length(SplitString)-1

                    StringSplit2 = strsplit(SplitString{kk},':');
                    if (~isempty(strfind(char(StringSplit2{1}),'RSU')))
                       row_T = row_T + 1; 
                       col_R = 0 ;
                       transmitter = strsplit(StringSplit2{1},' ');
                       Transmitter_Matrix(row_T,1) = str2double(transmitter{2});
                       Transmitter_Matrix(row_T,2) = str2double(StringSplit2{2}); 
                    elseif(~isempty(strfind(char(StringSplit2{1}),'Merge'))) % When NS3 send back Merge signal
                        merge_signal=1;
                        merge_moment=StringSplit2{2};
                    else
                        col_R = col_R + 1;
                        Receiver_Matrix(row_T,col_R) = str2double(StringSplit2{1});
                        col_R = col_R + 1;
                        Receiver_Matrix(row_T,col_R) = str2double(StringSplit2{2});           
                    end

                end
        
                
% 	  Storing the platoon vehicle speeds only once the network simulations
% 	  begin

        plot_sin_1(p_c,1) = simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Speed');
        plot_sin_2(p_c,1) = simNet.Vehicles.ItemByKey(Platoon_2(1,1)).get('AttValue','Speed');


        % Speed of leader increases at 250             
            if ((i > (200 * Resolution)) && (acc_cnt < 200))
                simNet.Vehicles.ItemByKey(Platoon_1(1,1)).set('AttValue','DesSpeed',ceil(70+acc_cnt*0.1));
         		simNet.Vehicles.ItemByKey(Platoon_1(1,1)).set('AttValue','Speed',ceil(70+acc_cnt*0.1));
                simNet.Vehicles.ItemByKey(Platoon_2(1,1)).set('AttValue','DesSpeed',ceil(70+acc_cnt*0.1));
         		simNet.Vehicles.ItemByKey(Platoon_2(1,1)).set('AttValue','Speed',ceil(70+acc_cnt*0.1));
         		acc_cnt = acc_cnt + 3;
            end
           
            
%% iterate Platoon_1 and Platoon_2 when have not merged
        if(merge_signal==0)
            for h=2:Platoon_Length1

                        C_1 = 0.5;
                        Xi = 1.7;
                        Omega_n = 0.4;
                        Alpha_1 = 1 - C_1;
                        Alpha_2 = C_1;
                        Alpha_3 = -(2*Xi - C_1*(Xi+sqrt(Xi^2 - 1)))*Omega_n;
                        Alpha_4 = -C_1*(Xi+sqrt(Xi^2 - 1))*Omega_n;
                        Alpha_5 = -(Omega_n^2);
                        Gap_des = 5;
                        v_actual = simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','Speed');
                        pos_actual = simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','Pos');
                        Numb_actual = simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','No');
                        
                        
       %% The following calulations will be intiiated only if the communcation was successful, something that will be provided% by NS3
                         Leader_vehicle_row = find(simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','No')== Transmitter_Matrix(:,1)); 
                         Prev_vehicle_row = find(simNet.Vehicles.ItemByKey(Platoon_1(h-1,1)).get('AttValue','No')== Transmitter_Matrix(:,1));
                      % Following if condition checks if the Platoon_1(h,1) vehicle has
                      % received a packet from Platoon_1(h,1) and Platoon_1(h-1,1)
                           Prev = 0;
                           Leader = 0;
                            if (Receiver_Matrix(Prev_vehicle_row,find(Numb_actual == Receiver_Matrix(Prev_vehicle_row,:))+1)~=0)
                                v_front = simNet.Vehicles.ItemByKey(Platoon_1(h-1,1)).get('AttValue','Speed');
                                Acc_front = simNet.Vehicles.ItemByKey(Platoon_1(h-1,1)).get('AttValue','Acceleration');
                                pos_front = simNet.Vehicles.ItemByKey(Platoon_1(h-1,1)).get('AttValue','Pos');
                                 Prev = 1;
                            end

                           if (Receiver_Matrix(Leader_vehicle_row,find(Numb_actual == Receiver_Matrix(Leader_vehicle_row,:))+1)~=0 )
                                v_leader = simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Speed');
                                Acc_leader = simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Acceleration');
                                Leader = 1;
                           end

                           if (Prev == 1 && Leader == 1)
                                Epsilon_i = pos_actual - pos_front + Gap_des;
                                Epsilon_ix = v_actual - v_front;
                                desired_acceleration = Alpha_1*Acc_front+Alpha_2*Acc_leader+Alpha_3*Epsilon_ix+Alpha_4*(v_actual-v_leader)+Alpha_5*Epsilon_i;
                                desired_speed = 0.1 * desired_acceleration + v_actual; %simulation step = 0.1s         
                                simNet.Vehicles.ItemByKey(Platoon_1(h,1)).set('AttValue','DesSpeed',desired_speed);
                                simNet.Vehicles.ItemByKey(Platoon_1(h,1)).set('AttValue','Speed',desired_speed);
                                
                           end
                        
                        %% The position plotting will be done regardless of what speed is followed so the following has to be separated from the above
%                         plot_sin(p_c,h) = simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','Speed');
%                         Position_error(p_c,h) =  simNet.Vehicles.ItemByKey(Platoon_1(h-1,1)).get('AttValue','Pos') - pos_actual - 6; 
%                         data = [simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','No')...
%                                 simNet.Vehicles.ItemByKey(Platoon_1(1,1)).get('AttValue','Pos')-simNet.Vehicles.ItemByKey(Platoon_1(h,1)).get('AttValue','Pos') Leader];
%                
%                         Simulation_Comms_data = vertcat(Simulation_Comms_data, data); 
                        
                        %truck_length = 6m ???
            end
            for p=2:Platoon_Length2

                        C_1_1 = 0.5;
                        Xi_1 = 1.7;
                        Omega_n_1 = 0.4;
                        Alpha_1_1 = 1 - C_1_1;
                        Alpha_2_1 = C_1_1;
                        Alpha_3_1 = -(2*Xi_1 - C_1_1*(Xi_1+sqrt(Xi_1^2 - 1)))*Omega_n_1;
                        Alpha_4_1 = -C_1_1*(Xi_1+sqrt(Xi_1^2 - 1))*Omega_n_1;
                        Alpha_5_1 = -(Omega_n_1^2);
                        Gap_des_1 = 5;
                        v_actual_1 = simNet.Vehicles.ItemByKey(Platoon_2(p,1)).get('AttValue','Speed');
                        pos_actual_1 = simNet.Vehicles.ItemByKey(Platoon_2(p,1)).get('AttValue','Pos');
                        Numb_actual_1 = simNet.Vehicles.ItemByKey(Platoon_2(p,1)).get('AttValue','No');
                        
                        
       %% The following calulations will be intiiated only if the communcation was successful, something that will be provided% by NS3
                         Leader_vehicle_row_1 = find(simNet.Vehicles.ItemByKey(Platoon_2(1,1)).get('AttValue','No')== Transmitter_Matrix(:,1)); 
                         Prev_vehicle_row_1 = find(simNet.Vehicles.ItemByKey(Platoon_2(p-1,1)).get('AttValue','No')== Transmitter_Matrix(:,1));
                      % Following if condition checks if the Platoon_1(h,1) vehicle has
                      % received a packet from Platoon_1(h,1) and Platoon_1(h-1,1)
                           Prev_1 = 0;
                           Leader_1 = 0;
                            if (Receiver_Matrix(Prev_vehicle_row_1,find(Numb_actual_1 == Receiver_Matrix(Prev_vehicle_row_1,:))+1)~=0)
                                v_front_1 = simNet.Vehicles.ItemByKey(Platoon_2(p-1,1)).get('AttValue','Speed');
                                Acc_front_1 = simNet.Vehicles.ItemByKey(Platoon_2(p-1,1)).get('AttValue','Acceleration');
                                pos_front_1 = simNet.Vehicles.ItemByKey(Platoon_2(p-1,1)).get('AttValue','Pos');
                                 Prev_1 = 1;
                            end

                           if (Receiver_Matrix(Leader_vehicle_row_1,find(Numb_actual_1 == Receiver_Matrix(Leader_vehicle_row_1,:))+1)~=0 )
                                v_leader_1 = simNet.Vehicles.ItemByKey(Platoon_2(1,1)).get('AttValue','Speed');
                                Acc_leader_1 = simNet.Vehicles.ItemByKey(Platoon_2(1,1)).get('AttValue','Acceleration');
                                Leader_1 = 1;
                           end

                           if (Prev_1 == 1 && Leader_1 == 1)
                                Epsilon_i_1 = pos_actual_1 - pos_front_1 + Gap_des_1;
                                Epsilon_ix_1 = v_actual_1 - v_front_1;
                                desired_acceleration_1 = Alpha_1_1*Acc_front_1+Alpha_2_1*Acc_leader_1+Alpha_3_1*Epsilon_ix_1+Alpha_4_1*(v_actual_1-v_leader_1)+Alpha_5_1*Epsilon_i_1;
                                desired_speed_1 = 0.1 * desired_acceleration_1 + v_actual_1; %simulation step = 0.1s         
                                simNet.Vehicles.ItemByKey(Platoon_2(p,1)).set('AttValue','DesSpeed',desired_speed_1);
                                simNet.Vehicles.ItemByKey(Platoon_2(p,1)).set('AttValue','Speed',desired_speed_1);
                                
                           end
                        
                        %% The position plotting will be done regardless of what speed is followed so the following has to be separated from the above
%                         plot_sin(p_c1,p) = simNet.Vehicles.ItemByKey(Platoon_2(p,1)).get('AttValue','Speed');
%                         Position_error(p_c1,p) =  simNet.Vehicles.ItemByKey(Platoon_2(p-1,1)).get('AttValue','Pos') - pos_actual_1 - 6; 
%                         data = [simNet.Vehicles.ItemByKey(Platoon_2(p,1)).get('AttValue','No')...
%                                 simNet.Vehicles.ItemByKey(Platoon_2(1,1)).get('AttValue','Pos')-simNet.Vehicles.ItemByKey(Platoon_2(p,1)).get('AttValue','Pos') Leader];
%                
%                         Simulation_Comms_data = vertcat(Simulation_Comms_data, data); 
%                         
                        %truck_length = 6m ???
            end
        end   
 %% After recieved merge signal
             if (merge_signal==1)
                  Platoon_3 = [Platoon_1;Platoon_2];
                  for u=2:Platoon_Length

                        C_1_2 = 0.5;
                        Xi_2 = 1.7;
                        Omega_n_2 = 0.4;
                        Alpha_1_2 = 1 - C_1_2;
                        Alpha_2_2 = C_1_2;
                        Alpha_3_2 = -(2*Xi_2 - C_1_2*(Xi_2+sqrt(Xi_2^2 - 1)))*Omega_n_2;
                        Alpha_4_2 = -C_1_2*(Xi_2+sqrt(Xi_2^2 - 1))*Omega_n_2;
                        Alpha_5_2 = -(Omega_n_2^2);
                        Gap_des_2 = 5;
                        v_actual_2 = simNet.Vehicles.ItemByKey(Platoon_3(u,1)).get('AttValue','Speed');
                        pos_actual_2 = simNet.Vehicles.ItemByKey(Platoon_3(u,1)).get('AttValue','Pos');
                        Numb_actual_2 = simNet.Vehicles.ItemByKey(Platoon_3(u,1)).get('AttValue','No');
                        
                        
       %% The following calulations will be intiiated only if the communcation was successful, something that will be provided% by NS3
                         Leader_vehicle_row_2 = find(simNet.Vehicles.ItemByKey(Platoon_3(1,1)).get('AttValue','No')== Transmitter_Matrix(:,1)); 
                         Prev_vehicle_row_2 = find(simNet.Vehicles.ItemByKey(Platoon_3(u-1,1)).get('AttValue','No')== Transmitter_Matrix(:,1));
                      % Following if condition checks if the Platoon_1(h,1) vehicle has
                      % received a packet from Platoon_1(h,1) and Platoon_1(h-1,1)
                           Prev_2 = 0;
                           Leader_2 = 0;
                            if (Receiver_Matrix(Prev_vehicle_row_2,find(Numb_actual_2 == Receiver_Matrix(Prev_vehicle_row_2,:))+1)~=0)
                                v_front_2 = simNet.Vehicles.ItemByKey(Platoon_3(u-1,1)).get('AttValue','Speed');
                                Acc_front_2 = simNet.Vehicles.ItemByKey(Platoon_3(u-1,1)).get('AttValue','Acceleration');
                                pos_front_2 = simNet.Vehicles.ItemByKey(Platoon_3(u-1,1)).get('AttValue','Pos');
                                 Prev_2 = 1;
                            end

                           if (Receiver_Matrix(Leader_vehicle_row_2,find(Numb_actual_2 == Receiver_Matrix(Leader_vehicle_row_2,:))+1)~=0 )
                                v_leader_2 = simNet.Vehicles.ItemByKey(Platoon_3(1,1)).get('AttValue','Speed');
                                Acc_leader_2 = simNet.Vehicles.ItemByKey(Platoon_3(1,1)).get('AttValue','Acceleration');
                                Leader_2 = 1;
                           end

                           if (Prev_2 == 1 && Leader_2 == 1)
                                Epsilon_i_2 = pos_actual_2 - pos_front_2 + Gap_des_2;
                                Epsilon_ix_2 = v_actual_2 - v_front_2;
                                desired_acceleration_2 = Alpha_1_2*Acc_front_2+Alpha_2_2*Acc_leader_2+Alpha_3_2*Epsilon_ix_2+Alpha_4_2*(v_actual_2-v_leader_2)+Alpha_5_2*Epsilon_i_2;
                                desired_speed_2 = 0.1 * desired_acceleration_2 + v_actual_2; %simulation step = 0.1s         
                                simNet.Vehicles.ItemByKey(Platoon_3(u,1)).set('AttValue','DesSpeed',desired_speed_2);
                                simNet.Vehicles.ItemByKey(Platoon_3(u,1)).set('AttValue','Speed',desired_speed_2);
                                
                           end
                        Platoon_1=Platoon_3(1:Platoon_Length1,:);
                        Platoon_2=Platoon_3(end-Platoon_Length2+1:1:end,:);
                        %% The position plotting will be done regardless of what speed is followed so the following has to be separated from the above
%                         plot_sin(p_c1,p) = simNet.Vehicles.ItemByKey(Platoon_2(p,1)).get('AttValue','Speed');
%                         Position_error(p_c1,p) =  simNet.Vehicles.ItemByKey(Platoon_2(p-1,1)).get('AttValue','Pos') - pos_actual_1 - 6; 
%                         data = [simNet.Vehicles.ItemByKey(Platoon_2(p,1)).get('AttValue','No')...
%                                 simNet.Vehicles.ItemByKey(Platoon_2(1,1)).get('AttValue','Pos')-simNet.Vehicles.ItemByKey(Platoon_2(p,1)).get('AttValue','Pos') Leader];
%                
%                         Simulation_Comms_data = vertcat(Simulation_Comms_data, data); 
%                         
                        %truck_length = 6m ???
            end
                               
            
            %% Packet delivery results from NS3
%                 Packets_dropped(comm_iter) = (sum(Receiver_Matrix(Leader_vehicle_row,:) == 0))/2;
%                 comm_iter = comm_iter + 1;
%                 Overall_Comms_data = cat(3, Overall_Comms_data,  Simulation_Comms_data);
%                 p_c = p_c + 1;
%                 p_c1 = p_c1 + 1;
%                 p_c2 = p_c2 + 1;
            end
        
     end
    end
  end
end
   
% I have removed the plotting codes....you can plot the data as per your
% convenience

Vissim.Simulation.Stop;
fclose(t1);
delete(t1);