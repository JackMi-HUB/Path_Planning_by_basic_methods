% ==============================================================================
%  MATLAB Source Codes in association with the paper entitled "Maneuver
%  Planning for Automatic Parking with Safe Travel Corridors: A Numerical
%  Optimal Control Approach" published by European Control Conference (ECC)
%  2020.
%  Copyright (C) 2020 Bai Li
%  2020.02.15
% ==============================================================================
% The codes are licensed under the GNU General Public License v3.0.
% Users need to get a licensed version of AMPL(free for 30 days) from the
% official website https://ampl.com/try-ampl/request-a-full-trial/
% Users are suggested to cite the following articles in association with
% the codes:
% (1) Li, B., & Shao, Z. (2015). A unified motion planning method for
% parking an autonomous vehicle in the presence of irregularly placed
% obstacles. Knowledge-Based Systems, 86, pp. 11-20.
% (2) Li, B. et al. (2020). Maneuver planning for automatic parking
% with safe travel corridors: A numerical optimal control approach. In
% Proc. 2020 European Control Conference (ECC), pp. XX-XX.
% ==============================================================================
clear;close all; clc
% global vehicle_TPBV_ obstacle_vertexes_
% load('Case1.mat');
% figure(1)
% InitParams();
% imshow(costmap_);
% Location_M = [];
% bVertexs = get_bVertex(costmap_);
% for i=1:106
%     Location_M = [Location_M;ConvertIndexToX(bVertexs(i,1))-20,ConvertIndexToY(bVertexs(i,2))-20];
% end
% % % Plot basic setups
% axis equal; box on; grid on; axis([-20 20 -20 20]);
% set(gcf,'outerposition',get(0,'screensize'));
% hold on;
% for ii = 1 : Nobs
%     fill(obstacle_vertexes_{ii}.x, obstacle_vertexes_{ii}.y, [125, 125, 125] ./ 255);%填充灰色
% end
% % for k = 1:201
% %     for i = 1:201
% %         costamp_(k,i) =  
% %         fill();
% %     end
% % end
% Arrow([vehicle_TPBV_.x0, vehicle_TPBV_.y0], [vehicle_TPBV_.x0 + cos(vehicle_TPBV_.theta0), vehicle_TPBV_.y0 + sin(vehicle_TPBV_.theta0)], 'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
% Arrow([vehicle_TPBV_.xtf, vehicle_TPBV_.ytf], [vehicle_TPBV_.xtf + cos(vehicle_TPBV_.thetatf), vehicle_TPBV_.ytf+ sin(vehicle_TPBV_.thetatf)],  'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
% drawnow
% 
% % % Hybrid A* search for a coarse path
% [x, y, theta, path_length, completeness_flag] = SearchHybridAStarPath();
% plot(x,y,'r'); drawnow
% % % Convert path to trajectory by attaching an optimal velocity to it
% [x, y, theta, v, a, phy, w, tf] = ResamplePath(x, y, theta);
x0 = [0:10/99:10];
y0 = [0:10/99:10];
th0 = [0:pi/99:pi];
[x, y, theta, v, a, phy, w, tf] = ResamplePath(x0, y0, th0);
plot(x,y,'g'); drawnow
[~, ~, xr, yr, xf, yf] = SpecifyLocalBoxes(x, y, theta);
WriteInitialGuess(x, y, theta, xr, yr, xf, yf, v, a, phy, w, tf);
WriteBoundaryValues();
!ampl rr.run

if (exist('opti_flag.txt','file'))
    load opti_flag.txt
    if (opti_flag)
        Statics();
        Dynamics();
    end
end

function [x1] = ConvertIndexToX(ind1)
global hybrid_astar_
x1 = ind1 * hybrid_astar_.resolution_x;
end

function [y1] = ConvertIndexToY(ind2)
global hybrid_astar_
y1 = ind2 * hybrid_astar_.resolution_y;
end