function [v,w]= getKeyBoardAndControl
% 28 leftarrow, 29 rightarrow, 30 uparrow, 31 downarrow
k = waitforbuttonpress;
value = double(get(gcf,'CurrentCharacter'));
if(value==28)% Quay trai
    v=0.05;w=0.2;
elseif (value==29) % Quay phai
    v=0.05;w=-0.2;
elseif (value==30) % Tien
    v=0.3;w=0;
elseif (value==31) % Lui
    v=-0.3;w=0;
else
    v=0; w=0;
end
        