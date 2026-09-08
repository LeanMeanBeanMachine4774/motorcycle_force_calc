"""
Created on Fri Sep  4 20:45:40 2026

@author: jaden
"""

import math

import numpy as np

NUMTESTS = 30  # number of tests to run

# Dimensions relative to rear contact patch

BIKE_MASS = 120  # m
RIDER_MASS = 100
GRAVITY = 9.8  # g
WHEEL_BASE = 1.3  # p
# Bike COG assuming centred
BIKECOG = np.array([0.65, 0, 0.895 / 2])  # x, y, z
# Rider COG is just placed randomly, should be changed based on erg study
RIDERCOG = np.array([0.50, 0, 1.0])  # x, y, z
COG = (BIKE_MASS * BIKECOG + RIDER_MASS * RIDERCOG) / (BIKE_MASS + RIDER_MASS)
FWHEELRAD = 0.578 / 2  # front wheel radius
RWHEELRAD = 0.601 / 2  # rear wheel radius
COF = 1.5  # coefficient of friction
FTYRERAD = FWHEELRAD - (0.090 * 0.8) / 2  # radius of front tyre's centreline
RTYRERAD = RWHEELRAD - (0.115 * 0.75) / 2  # radius of rear tyre's centreline
ROLL_ANG = np.linspace(-math.pi / 3, math.pi / 3, NUMTESTS)  # roll angle (rad)
AIR_DENS = 1.204  # sea level, 20C

CASTER_ANG = math.radians(22.5)  # caster angle
STEER_ANG = np.linspace(-math.pi / 6, math.pi / 6, NUMTESTS)  # steering angle
KINSTEER_ANG = (
    np.cos(CASTER_ANG) / np.cos(ROLL_ANG) * STEER_ANG
)  # kinematic steering angle

COLA = 0.09  # coefficient of lift*area (middle of range from cossalter)
CODA = 0.7  # coefficient of drag*area (big over estimate)
VEL_FORWARD = 20  # forward velocity
PMAX = 123  # max motor power
RCURVEREAR = WHEEL_BASE / np.tan(KINSTEER_ANG)  # Radius of curvature of rear wheel

FDRAG = 0.5 * AIR_DENS * CODA * VEL_FORWARD**2  # drag force
FAERO = 0.5 * AIR_DENS * COLA * VEL_FORWARD**2  # aerodynamic force

THRUST_LEVEL_SS = FDRAG


def level_free_stand():
    fnorm = BIKE_MASS * GRAVITY * (WHEEL_BASE - COG[0]) / WHEEL_BASE
    rnorm = BIKE_MASS * GRAVITY * COG[0] / WHEEL_BASE
    return fnorm, rnorm


def ss_rectilinear():
    fnorm = BIKE_MASS * GRAVITY * COG[0] / WHEEL_BASE - THRUST_LEVEL_SS * (
        COG[2] / WHEEL_BASE
    )

    rnorm = BIKE_MASS * GRAVITY * (
        WHEEL_BASE - COG[0]
    ) / WHEEL_BASE + THRUST_LEVEL_SS * (COG[2] / WHEEL_BASE)
    vmax = np.sqrt(
        (BIKE_MASS * GRAVITY)
        / (
            0.5 * AIR_DENS * CODA * (COG[2] / WHEEL_BASE)
            + 0.5 * AIR_DENS * COLA * (COG[0] / WHEEL_BASE)
        )
        * (COG[0] / WHEEL_BASE)
    )
    return fnorm, rnorm, vmax


def trans_rectilinear():
    amax_englim = (PMAX / VEL_FORWARD - FDRAG) / BIKE_MASS
    amax_traclim = (COF * GRAVITY * (WHEEL_BASE - COG[0]) / WHEEL_BASE) / (
        1 - COF * COG[2] / WHEEL_BASE
    ) - FDRAG / BIKE_MASS
    amax_wheelielim = GRAVITY * (COG[0] / COG[2]) - FDRAG / BIKE_MASS
    return amax_englim, amax_traclim, amax_wheelielim
    """trans_rectilinear_amax = min(amax_englim, amax_traclim, amax_wheelielim)
    if trans_rectilinear_amax == amax_englim:
        return amax_englim, "Engine limited"
    elif trans_rectilinear_amax == amax_traclim:
        return amax_traclim, "Traction limited"
    else:
        return amax_wheelielim, "Wheelie limited"""


def ss_cornering():
    fnorm = BIKE_MASS * GRAVITY * WHEEL_BASE / COG[0] - FAERO * (
        COG[2] / WHEEL_BASE
    ) * np.cos(ROLL_ANG)
    rnorm = BIKE_MASS * GRAVITY * (WHEEL_BASE - COG[0]) / COG[0] + FAERO * (
        COG[2] / WHEEL_BASE
    ) * np.cos(ROLL_ANG)
    flateral = fnorm / (GRAVITY * np.cos(KINSTEER_ANG)) * (VEL_FORWARD**2 / RCURVEREAR)
    rlateral = rnorm / GRAVITY * (VEL_FORWARD**2 / RCURVEREAR)
    return fnorm, rnorm, flateral, rlateral


lfsfnorm, lfsrnorm = level_free_stand()
ssrfnorm, ssrrnorm, ssvmax = ss_rectilinear()
tra_englim, tra_traclim, tra_wheelielim = trans_rectilinear()
ssafnorm, ssarnorm, ssaflateral, ssarlateral = ss_cornering()
print(
    f"Level Free Stand: Front Normal Force = {lfsfnorm}, Rear Normal Force = {lfsrnorm}"
)
print(
    f"Steady-State Rectilinear: Front Normal Force = {ssrfnorm}, Rear Normal Force = {ssrrnorm}, Maximum Velocity = {ssvmax}"
)
print(
    f"Transient Rectilinear: Maximum Engine Limited Acceleration = {tra_englim}, Traction Limited Acceleration = {tra_traclim}, Wheelie Limited Acceleration = {tra_wheelielim}"
)
print(
    f"Steady-State Acceleration: Front Normal Force = {ssafnorm}, Rear Normal Force = {ssarnorm}, Front Lateral Force = {ssaflateral}, Rear Lateral Force = {ssarlateral}"
)
