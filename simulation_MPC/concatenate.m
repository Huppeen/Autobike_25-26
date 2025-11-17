function [Xref,Yref]=concatenate(Xref,Yref,Xref2,Yref2)

    % put the second part after the first one in the correct direction
    
    % position in the origine
    Xref2=Xref2-Xref2(1);
    Yref2=Yref2-Yref2(1);
    
    % rotate
    v1=[Xref(end)-Xref(end-1) Yref(end)-Yref(end-1)];
    v2=[Xref2(2)-Xref2(1) Yref2(2)-Yref2(1)];
    v1=v1/norm(v1);
    v2=v2/norm(v2);
    th=acos(v1*v2')*sign(v1*[0 -1;1 0]*v2');
    rot=[cos(th) -sin(th);sin(th) cos(th)];
    ref2=rot*[Xref2 Yref2]';
    Xref2=ref2(1,:)';
    Yref2=ref2(2,:)';
    
    % position after last part
    Xref2=Xref2+2*Xref(end)-Xref(end-1);
    Yref2=Yref2+2*Yref(end)-Yref(end-1);
    % 
    % Xref=[Xref;Xref2(2:end)];
    % Yref=[Yref;Yref2(2:end)];    
    Xref=[Xref;Xref2];
    Yref=[Yref;Yref2];

end