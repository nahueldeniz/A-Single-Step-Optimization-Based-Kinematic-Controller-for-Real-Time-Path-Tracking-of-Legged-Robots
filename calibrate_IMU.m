rosshutdown;
rosinit;
vec2IMU     = rossubscriber('/vectornav2/IMU');
quat        = zeros(4,1);
nroSamples  = 1000;

for i=1:nroSamples
    vec2imu = receive(vec2IMU);
    quat_aux = [vec2imu.Orientation.X;vec2imu.Orientation.Y;vec2imu.Orientation.Z;vec2imu.Orientation.W];
    quat = quat + quat_aux;
    angle_aux  = quat2eul(quat');
    angle_aux(3)
end
quat = quat/nroSamples


angles      = quat2eul(quat');

IMU_ZERO    = angles(3)




%%
RTK         = rossubscriber('/fix_holder');
for i=1:100
    tic;
    rtk = receive(RTK);
    a= rtk.Latitude;
    b=rtk.Longitude;
    toc
end

%%
figure; hold on; grid on;

plot(S.path.coordinates(1,:),S.path.coordinates(2,:),'k','LineWidth',3);
plot(S.data.performance.xest{1}(1,:),S.data.performance.xest{1}(2,:),'b','LineWidth',3);
daspect([1 1 1])




