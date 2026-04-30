function vel = ComputeVelocity_DiscretizedPath(path, posNow, NowIndex, TargetIndex)
% Compute the velocity from the discrcretized path stored under Control
% SpeedFactor is a multiplying factor for the speed

global ControlParams
FeedbackLinearizationDistance = 0.2;
SearchAheadPointCount = 10;
LookAheadPointsCount = 2;
SpeedFactor = 1;
speed = 0.1;
MaxTurnRate = 0.08;

currentPos = posNow.p + FeedbackLinearizationDistance*[cos(posNow.theta); sin(posNow.theta)];
searchAheadPointIndices = NowIndex:min([NowIndex+SearchAheadPointCount, size(path,2)]);

distSqFromAheadPoints = sum((path(:,searchAheadPointIndices) - repmat(currentPos,[1 length(searchAheadPointIndices)])) .^ 2);
[nearestPtDistSq, nearestPtIndex] = min(distSqFromAheadPoints);

NowIndex = searchAheadPointIndices(nearestPtIndex);

%targetPointIndex = min([ Control.PathPoints.NowIndex+round(ControlParams.TargetPointDistanceAhead/ControlParams.PathDiscretizationResolution), ...
%    size(Control.PathPoints.points,2) ]);

nowPoint = path(:,NowIndex);
targetPoint = path(:,TargetIndex);

%speed = ControlParams.RobotSpeed * SpeedFactor;
if NowIndex>TargetIndex
    SpeedFactor = 0;
end

globalVel = targetPoint - posNow.p;
globalVel = speed * globalVel / norm(globalVel);

localVel = [cos(posNow.theta), sin(posNow.theta); -sin(posNow.theta), cos(posNow.theta)] * globalVel;
vel = [localVel(1), localVel(2)/FeedbackLinearizationDistance];

if abs(vel(2)) > MaxTurnRate
    vel = vel * abs(MaxTurnRate / vel(2));
end
