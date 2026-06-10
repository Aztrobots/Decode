package org.firstinspires.ftc.teamcode.tools;

import com.acmerobotics.dashboard.config.Config;

/**
 * Shoot-On-The-Move (2-DOF: azimuth turret + flywheel velocity, pitch FIJO).
 *
 * Calculadora pura: no toca hardware. El OpMode le pasa pose + velocidad
 * field-frame (pulgadas, pulgadas/s) desde el Pinpoint o el Follower de Pedro,
 * y consume el resultado vía AutoRobot.setAzimuthThetaVelocity + turret.setFeedforward.
 *
 * Método: virtual-goal (shoot-on-the-fly). Desplaza el goal por v_robot * TOF;
 * el lead tangencial define el azimuth, la componente radial corrige la velocidad.
 */
@Config
public class SOTM {

    // ─── GOAL (RED) — field-frame, pulgadas ───────────────────────────────────
    // Para BLUE: espeja X (FIELD_X - GOAL_X) cuando integres .mirror.
    public static double GOAL_X = 144.0;
    public static double GOAL_Y = 144.0;

    // Debe coincidir con TurretSubsystem.TURRET_OFFSET_RAD
    public static double TURRET_OFFSET_RAD = 0.0;

    // ─── LÍMITES / SEGURIDAD ──────────────────────────────────────────────────
    public static double MAX_TPS       = 2200.0; // cap físico del flywheel
    public static double MIN_SPEED_INS = 3.0;    // deadband: bajo esto NO se aplica lead (anti-jitter)

    // ─── LATENCIA / LEAD (TUNE) ───────────────────────────────────────────────
    public static double LATENCY_TANGENTIAL = 0.5; // escala TOF para el lead de azimuth
    public static double LATENCY_RADIAL     = 2.8; // escala TOF para la corrección de velocidad

    // ─── TURRET FEEDFORWARD (TUNE; EMPIEZA EN 0) ──────────────────────────────
    // Actívalo solo cuando lead + velocity ya peguen en estático. Signo = empírico.
    public static double KF_TURRET = 0.0;

    // LUTs empíricas, key = distancia (pulgadas)
    private final LUT velocityLUT = new LUT(); // dist -> TPS
    private final LUT tofLUT      = new LUT(); // dist -> tiempo de vuelo (s)

    public SOTM() {
        // ── TUNEAR EN CAMPO ──────────────────────────────────────────────────
         velocityLUT.addData(48,  1420);
         velocityLUT.addData(63,  1540);
         velocityLUT.addData(98,  1740);
         velocityLUT.addData(145, 2280);

         tofLUT.addData(48,  0.42);
         tofLUT.addData(98,  0.60);
         tofLUT.addData(145, 0.78);
        // ─────────────────────────────────────────────────────────────────────
    }

    public void addVelocitySample(double distIn, double tps) { velocityLUT.addData(distIn, tps); }
    public void addTofSample(double distIn, double tofS)      { tofLUT.addData(distIn, tofS); }

    /**
     * @param robotX, robotY      pose field-frame (pulgadas)
     * @param robotHeadingRad     heading del chasis (rad)
     * @param vx, vy              velocidad field-frame (pulgadas/s)
     * @param angularVelRadPerSec velocidad angular del chasis (rad/s)
     * @return {azimuthRad (relativo al robot → setTargetRadians), velocityTPS, turretFeedforward}
     */
    public double[] calculate(double robotX, double robotY, double robotHeadingRad,
                              double vx, double vy, double angularVelRadPerSec) {

        double dx   = GOAL_X - robotX;
        double dy   = GOAL_Y - robotY;
        double dist = Math.hypot(dx, dy);
        if (dist < 1e-3) dist = 1e-3; // NaN guard (división)

        double speed = Math.hypot(vx, vy);

        // ── Solución estática: robot quieto o bajo deadband ──
        if (speed < MIN_SPEED_INS) {
            double az  = azimuthFor(dx, dy, robotHeadingRad);
            double tps = clampTps(velocityLUT.getValue(dist));
            return new double[]{az, tps, 0.0};
        }

        // ── TOF base + virtual goals (lead tangencial / corrección radial) ──
        double tof   = tofLUT.getValue(dist);          // 0 si LUT vacía → degrada a estático, seguro
        double tTang = LATENCY_TANGENTIAL * tof;
        double tRad  = LATENCY_RADIAL     * tof;

        // azimuth desde el virtual goal tangencial
        double dxT = (GOAL_X - vx * tTang) - robotX;
        double dyT = (GOAL_Y - vy * tTang) - robotY;
        double azimuth = azimuthFor(dxT, dyT, robotHeadingRad);

        // velocidad desde la distancia al virtual goal radial
        double distRad = Math.hypot((GOAL_X - vx * tRad) - robotX,
                (GOAL_Y - vy * tRad) - robotY);
        double tps = clampTps(velocityLUT.getValue(distRad));

        // ── Feedforward de turret = rate del line-of-sight + rotación del chasis ──
        // d(bearing)/dt por traslación = -(dx·vy - dy·vx)/dist² ; el turret target
        // también incluye -heading, de ahí el -angularVel. RED: signo directo.
        double losRate          = -(dx * vy - dy * vx) / (dist * dist);
        double targetTurretRate = losRate - angularVelRadPerSec;
        double turretFF         = KF_TURRET * targetTurretRate;

        return new double[]{azimuth, tps, turretFF};
    }

    private double azimuthFor(double dx, double dy, double headingRad) {
        return normalize(Math.atan2(dy, dx) - headingRad - TURRET_OFFSET_RAD);
    }

    private double clampTps(double tps) {
        if (Double.isNaN(tps)) return 0.0;
        return Math.max(0.0, Math.min(MAX_TPS, tps));
    }

    private double normalize(double rad) {
        return ((rad + Math.PI) % (2.0 * Math.PI) + 2.0 * Math.PI) % (2.0 * Math.PI) - Math.PI;
    }
}