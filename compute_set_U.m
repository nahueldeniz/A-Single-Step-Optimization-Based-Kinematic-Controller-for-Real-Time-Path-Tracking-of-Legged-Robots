

dpxdt       = diff(S.path.coordinates(1,:))./S.config.Ts;
dpydt       = diff(S.path.coordinates(2,:))./S.config.Ts;
theta_ref   = atan2(dpydt, dpxdt);
dpthetadt   = diff(theta_ref)./S.config.Ts;

vfM = -inf;
vfm = inf;
vlM = -inf;
vlm = inf;
wfM = -inf;
wfm = inf;

theta = 0:0.01:2*pi;

for i=1:length(theta)
     ma
end