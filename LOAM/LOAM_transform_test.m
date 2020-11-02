% origin
syms sx cx sy cy sz cz a b c d x y
Rx = [cx sx 0;-sx cx 0;0 0 1];
Ry = [cy 0 -sy;0 1 0;sy 0 cy];
Rz = [1 0 0;0 cz sz;0 -sz cz];
% in LOAM
% x=y,y=z,z=x
Rz = [cz sz 0;-sz cz 0;0 0 1];
Rx = [cx 0 -sx;0 1 0;sx 0 cx];
Ry = [1 0 0;0 cy sy;0 -sy cy];
% actually is
Rx = [1 0 0;0 cx sx;0 -sx cx];
Ry = [cy 0 -sy;0 1 0;sy 0 cy];
Rz = [cz sz 0;-sz cz 0;0 0 1];
Rx*Ry*Rz

syms s3 c3 s2 c2 s1 c1
C3 = [c3 s3 0;-s3 c3 0;0 0 1];
C2 = [c2 0 -s2;0 1 0;s2 0 c2];
C1 = [1 0 0;0 c1 s1;0 -s1 c1];
C3*C2*C1

p = [a b x;c d y;0 0 1];
Rx_1 = [cx -sx 0;sx cx 0;0 0 1];

p0 = [1,2,45].'; % x,y,yaw(degree)
p0_mat = eye(3,3);
p0_mat(1:2,3) = p0(1:2);
rot_temp = rotz(p0(3)); % degree
p0_mat(1:2,1:2) = rot_temp(1:2,1:2);

delta_yaw = 45;
delta_t = [-1,0].';
TR = eye(3,3);
rot_temp = rotz(delta_yaw); % degree
TR(1:2,1:2) = rot_temp(1:2,1:2); 
Tt = eye(3,3);
Tt(1:2,3) = delta_t;

p1_mat = p0_mat * TR * Tt
