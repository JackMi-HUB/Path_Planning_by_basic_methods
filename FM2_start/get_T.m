function T = get_T(pose,mat_T,mat_V)
%计算pose处的T，对其进行全向检索，判断是否位于边界点或者障碍点
%左
pose_left=[pose(1)-1,pose(2)];
     if isnan(mat_T(pose_left(1),pose_left(2)))||pose_left(1)<1
         T1=mat_T(pose(1)+1,pose(2));
     else
         T1=min(mat_T(pose(1)-1,pose(2)),mat_T(pose(1)+1,pose(2)));
     end
%右
pose_right=[pose(1)+1,pose(2)];
     if isnan(mat_T(pose_right(1),pose_right(2)))||pose_right(1)>200
         T1=mat_T(pose(1)-1,pose(2));
     else
         T1=min(mat_T(pose(1)-1,pose(2)),mat_T(pose(1)+1,pose(2)));
     end

%上
pose_up=[pose(1),pose(2)-1];
     if isnan(mat_T(pose_up(1),pose_up(2)))||pose_up(2)<1
         T2=mat_T(pose(1),pose(2)+1);
     else
         T2=min(mat_T(pose(1),pose(2)-1),mat_T(pose(1),pose(2)+1));
     end
%下  
pose_down=[pose(1),pose(2)+1];
     if isnan(mat_T(pose_down(1),pose_down(2)))||pose_down(2)>200
         T2=mat_T(pose(1),pose(2)-1);
     else
         T2=min(mat_T(pose(1),pose(2)-1),mat_T(pose(1),pose(2)+1));
     end
% 计算T(pose)
    if mat_T(pose(1),pose(2))>=T1 && mat_T(pose(1),pose(2))<=T2
        T=T1+1/mat_V(pose(1),pose(2));
    elseif mat_T(pose(1),pose(2))>=T2 && mat_T(pose(1),pose(2))<=T1
        T=T2+1/mat_V(pose(1),pose(2));
    else
%         disp('error')
        T=min(T1,T2)+1/mat_V(pose(1),pose(2));

    end
end

