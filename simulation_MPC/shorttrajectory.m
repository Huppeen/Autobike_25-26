clear all;
clc;

[Xref,Yref,Psiref] = ReferenceGenerator('line',0.3,21,1);
tref = Xref/2.5;

[Xref2,Yref2,Psiref2] = ReferenceGenerator('line',0.3,20,1);
tref2 = Xref2/2.5;

[Xref3,Yref3,Psiref3] = ReferenceGenerator('line',0.3,20,1);
tref3 = Xref3/2.5;

[Xref4,Yref4,Psiref4] = ReferenceGenerator('line',0.3,20,1);
tref4 = Xref4/2.5;

[Xref,Yref]=concatenate(Xref,Yref,Xref2,Yref2);

[Xref,Yref]=concatenate(Xref,Yref,Xref3,Yref3);

[Xref,Yref]=concatenate(Xref,Yref,Xref4,Yref4);

% Vref = [ones(1,20)*1, ones(1,20)*1.5, ones(1,20)*2, ones(1,20)*2.5, ones(1,20)*3]';
  tref = [ones(1,21)*0.12, ones(1,20)*0.12, ones(1,20)*0.12, ones(1,20)*0.12]';

% 段1：从0开始，步长0.3，共40个点（即39次步进）
t1 = 0 + cumsum([0, ones(1,20) * 0.12]);

% 段2：接着t1，步长0.2，共40个点
t2 = t1(end) + cumsum(ones(1,20) * 0.12);

% 段3：步长0.15
t3 = t2(end) + cumsum(ones(1,20) * 0.12);

% 段4：步长0.12
t4 = t3(end) + cumsum(ones(1,20) * 0.12);

% 拼接所有时间
tref = [t1, t2, t3, t4]';

data = [Xref, Yref, tref];

filename = 'AAshortStraight_highspeed2.5.csv';

writematrix(data, filename);


[Xref,Yref,Psiref,Vref,t]=Refgeneration({'x','y','t'},filename);


T = table(Xref, Yref, Psiref, Vref, t, 'VariableNames', {'Xref', 'Yref', 'Psiref', 'Vref', 'Time'});


writetable(T, 'reference_output_highspeed2.5.csv');