function data = Robot_Communicate(option, robotID, params)
persistent init_struct robot0_id truth0_id laser0_id x
persistent scarab pid laser_id tracker_id
persistent start_time


switch option

    case 'initiate'

        scarab{robotID} = playerAPI('client','create',params{1}, 6665);
        playerAPI('client','connect',scarab{robotID});
        playerAPI('client','set_replace_rule',scarab{robotID},-1,-1,'PLAYER_MSGTYPE_DATA',-1, 1);
        playerAPI('client','datamode',scarab{robotID},'PLAYERC_DATAMODE_PULL');

        pid{robotID} = playerAPI('position2d','create', scarab{robotID}, 0);
        playerAPI('position2d', 'subscribe', pid{robotID}, 'PLAYERC_OPEN_MODE');

        if any(strcmp('laser',params))
            laser_id{robotID} = playerAPI('laser','create', scarab{robotID}, 0);
            playerAPI('laser', 'subscribe', laser_id{robotID}, 'PLAYERC_OPEN_MODE');
        end
        if any(strcmp('truth',params))
            tracker_id{robotID} = playerAPI('fiducial','create', scarab{robotID}, 0);
            playerAPI('fiducial', 'subscribe', tracker_id{robotID}, 'PLAYERC_OPEN_MODE');
        end

    case 'getData_Pose'

        tracker_data = playerAPI('fiducial', tracker_id{robotID});
        data.p = [tracker_data.fiducials.pose.px; tracker_data.fiducials.pose.py];
        data.theta = tracker_data.fiducials.pose.pyaw;

    case 'getData_LaserLocal'

        LaserLocalPose = [0.0622340; 0.0];
        laser_data = playerAPI('laser', laser_id{robotID});
        if nargin > 1 && isstr(params)
            data = laser_data;
        else
            angles = laser_data.scan_start + (0:(laser_data.scan_count-1))*laser_data(1).scan_res;
            ValidIndices = laser_data.ranges < GlobalParams.WorkingLaserRange;
            angles = angles(ValidIndices);
            ranges = laser_data.ranges(ValidIndices);
            data.LaserPoints_local = [];
            data.LaserPoints_local(1,:) = ranges.*cos(angles) + LaserLocalPose(1);
            data.LaserPoints_local(2,:) = ranges.*sin(angles) + LaserLocalPose(2);
            data.ranges = laser_data.ranges;
        end

    case 'setVelocity'

        % params(1) -- forward velocity ; params(2) -- angular velocity
        playerAPI('position2d','set_cmd_vel', pid{robotID}, params(1), 0, params(2), 1);

    case 'getTimestamp'

        data = clock;

end

