#ifndef SFO_OXTS_H
#define SFO_OXTS_H

struct oxts { // oxts is the brand of the INS used in the KITTI dataset
    /*
    lat:           latitude of the oxts-unit (deg)
    lon:           longitude of the oxts-unit (deg)
    alt:           altitude of the oxts-unit (m)
    roll:          roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
    pitch:         pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
    yaw:           heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
    vn:            velocity towards north (m/s)
    ve:            velocity towards east (m/s)
    vf:            forward velocity, i.e. parallel to earth-surface (m/s)
    vl:            leftward velocity, i.e. parallel to earth-surface (m/s)
    vu:            upward velocity, i.e. perpendicular to earth-surface (m/s)
    ax:            acceleration in x, i.e. in direction of vehicle front (m/s^2)
    ay:            acceleration in y, i.e. in direction of vehicle left (m/s^2)
    ay:            acceleration in z, i.e. in direction of vehicle top (m/s^2)
    af:            forward acceleration (m/s^2)
    al:            leftward acceleration (m/s^2)
    au:            upward acceleration (m/s^2)
    wx:            angular rate around x (rad/s)
    wy:            angular rate around y (rad/s)
    wz:            angular rate around z (rad/s)
    wf:            angular rate around forward axis (rad/s)
    wl:            angular rate around leftward axis (rad/s)
    wu:            angular rate around upward axis (rad/s)
    pos_accuracy:  position accuracy (north/east in m)
    vel_accuracy:  velocity accuracy (north/east in m/s)
    navstat:       navigation status (see navstat_to_string)
    numsats:       number of satellites tracked by primary GPS receiver
    posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
    velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
    orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
    */
    double lat, lon, alt,
            roll, pitch, yaw,
            vn, ve, vf, vl, vu,
            ax, ay, az, af, al, au,
            wx, wy, wz, wf, wl, wu,
            pos_accuracy, vel_accuracy;
    int navsat, numsats,
            posmode, velmode, orimode;
    oxts() : lat(0), lon(0), alt(0),
             roll(0), pitch(0), yaw(0),
             vn(0), ve(0), vf(0), vl(0), vu(0),
             ax(0), ay(0), az(0), af(0), al(0), au(0),
             wx(0), wy(0), wz(0), wf(0), wl(0), wu(0),
             pos_accuracy(0), vel_accuracy(0),
             navsat(0), numsats(0),
             posmode(0), velmode(0), orimode(0)
    {
    }
};

#endif //SFO_OXTS_H
