# Definitions of the supported input shapers
#
# Copyright (C) 2020-2026  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections, copy, math, re
import mathutil

SHAPER_VIBRATION_REDUCTION=20.
DEFAULT_DAMPING_RATIO = 0.1

InputShaperCfg = collections.namedtuple(
        'InputShaperCfg',
        ('name', 'init_func', 'min_freq', 'max_damping_ratio'))

class ShaperError(Exception):
    pass

def _validate_damping_ratio(damping_ratio, name='damping_ratio'):
    if not math.isfinite(damping_ratio) or damping_ratio < 0. \
            or damping_ratio >= 1.:
        raise ShaperError("%s must be in the range [0, 1)" % name)
    return damping_ratio

def _validate_vibration_tolerance(v_tol, name='v_tol'):
    if not math.isfinite(v_tol) or v_tol <= 0. or v_tol >= 1.:
        raise ShaperError("%s must be in the range (0, 1)" % name)
    return v_tol

def get_none_shaper():
    return ([], [])

def get_zv_shaper(shaper_freq, damping_ratio):
    df = math.sqrt(1. - damping_ratio**2)
    K = math.exp(-damping_ratio * math.pi / df)
    t_d = 1. / (shaper_freq * df)
    A = [1., K]
    T = [0., .5*t_d]
    return (A, T)

def get_zvd_shaper(shaper_freq, damping_ratio):
    df = math.sqrt(1. - damping_ratio**2)
    K = math.exp(-damping_ratio * math.pi / df)
    t_d = 1. / (shaper_freq * df)
    A = [1., 2.*K, K**2]
    T = [0., .5*t_d, t_d]
    return (A, T)

def get_mzv_coeffs(n, t):
    if n < 3:
        raise ShaperError("Too small n=%d, must be at least 3" % n)
    if n <= 2 * t + 1 + 1e-7:
        raise ShaperError("Too large t=%.6f for n=%d, must be less than %.6f" %
                          (t, n, 0.5 * (n - 1)))
    # Projected shaper duration with n -> \infinity for computing shaper zeros
    tau = t * (n - 2.) / (n - 2. * t - 1.)
    T = [i * t / (n - 1.) for i in range(n)]
    # Build a system of equations for A. The first equation is sum(A) = 1
    M = [[1.] * n]
    F = [1.]
    # Ensure correct placement of shaper zeros. Note that the system is not
    # over-contrained, as the extra equations are linearly-dependent.
    for i in range(n-1):
        W = [2. * math.pi * (1. + i / tau) * tj for tj in T]
        M.append([math.cos(w) for w in W])
        M.append([math.sin(w) for w in W])
        F.append(0.)
        F.append(0.)
    M_inv = mathutil.pseudo_inverse(M)
    if M_inv is None:
        raise ShaperError("Ill-formed shaper with n=%d, t=%.6f" % (n, t))
    A = mathutil.mat_mat_mul([F], mathutil.mat_transp(M_inv))[0]
    if any(a < -0.00001 for a in A):
        raise ShaperError("Negative-valued shaper with n=%d, t=%.6f" % (n, t))
    return (A, T)

def get_mzv_shaper(shaper_freq, damping_ratio, n=3, t=0.0, tau=0.0):
    if not tau and not t:
        t = 0.75
    elif tau:
        # Infer total shaper duration from a projected shaper duration with
        # n -> \infinity
        t = tau * (n - 1.) / (n + 2. * tau - 2.)
    A, T = get_mzv_coeffs(n, t)
    # Apply damping
    df = math.sqrt(1. - damping_ratio**2)
    K = math.exp(-2. * t * damping_ratio * math.pi / ((n - 1.) * df))
    t_d = 1. / (shaper_freq * df)
    Kp = K
    for i in range(1, n):
        T[i] *= t_d
        A[i] *= Kp
        Kp *= K
    return (A, T)

def get_ei_shaper(shaper_freq, damping_ratio,
                  v_tol=1./SHAPER_VIBRATION_REDUCTION):
    if not math.isfinite(shaper_freq) or shaper_freq <= 0.:
        raise ShaperError("shaper_freq must be positive")
    _validate_damping_ratio(damping_ratio)
    _validate_vibration_tolerance(v_tol)
    df = math.sqrt(1. - damping_ratio**2)
    t_d = 1. / (shaper_freq * df)
    dr = damping_ratio

    a1 = (0.24968 + 0.24961 * v_tol) + (( 0.80008 + 1.23328 * v_tol) +
                                        ( 0.49599 + 3.17316 * v_tol) * dr) * dr
    a3 = (0.25149 + 0.21474 * v_tol) + ((-0.83249 + 1.41498 * v_tol) +
                                        ( 0.85181 - 4.90094 * v_tol) * dr) * dr
    a2 = 1. - a1 - a3

    t2 = 0.4999 + ((( 0.46159 + 8.57843 * v_tol) * v_tol) +
                   (((4.26169 - 108.644 * v_tol) * v_tol) +
                    ((1.75601 + 336.989 * v_tol) * v_tol) * dr) * dr) * dr

    A = [a1, a2, a3]
    T = [0., t2 * t_d, t_d]
    return (A, T)

def _convolve_shapers(a1, t1, a2, t2, name):
    pulses = sorted((ti + tj, ai * aj)
                    for ti, ai in zip(t1, a1)
                    for tj, aj in zip(t2, a2))
    # Merge coincident times so the C helper receives strictly increasing T.
    A, T = [], []
    for t, a in pulses:
        if T and abs(t - T[-1]) < 1e-12:
            A[-1] += a
        else:
            T.append(t)
            A.append(a)
    if len(A) > 10:
        raise ShaperError("%s produces more than 10 pulses" % name)
    total = sum(A)
    return ([a / total for a in A], T)

def _get_composite_ei_params(damping_ratio, v_tol, damping2, v_tol2):
    """Resolve the optional parameters for the second EI stage."""
    damping2 = damping_ratio if damping2 is None else damping2
    v_tol2 = v_tol if v_tol2 is None else v_tol2
    _validate_damping_ratio(damping_ratio)
    _validate_damping_ratio(damping2, 'damping2')
    _validate_vibration_tolerance(v_tol)
    _validate_vibration_tolerance(v_tol2, 'v_tol2')
    return damping2, v_tol2

def get_composite_ei_shaper(shaper_freq, damping_ratio, f2=0.,
                            v_tol=1./SHAPER_VIBRATION_REDUCTION,
                            damping2=None, v_tol2=None):
    """Convolve two EI shapers to suppress two measured bands.

    ``shaper_freq`` is the first EI center and ``f2`` is the second one.
    ``v_tol`` and ``damping_ratio`` configure the first (usually low-frequency)
    stage.  ``v_tol2`` and ``damping2`` optionally configure the second stage;
    when omitted they inherit the first stage values.  EI x EI produces nine
    pulses, within the native ten-pulse kinematic shaper limit.

    The parser accepts named parameters in ``shaper_type_x``/``y``.  A formal
    configuration therefore uses one ``[input_shaper]`` section, for example::

        shaper_type_x: composite_ei(f2=161.3,v_tol=0.10)
        shaper_freq_x: 25.0

    This applies both notches to X; the same form can be used independently
    for Y.  A second ``[input_shaper]`` section is not valid Klipper config.
    """
    if not math.isfinite(f2) or f2 <= 0.:
        raise ShaperError("composite_ei requires a positive f2 parameter")
    damping2, v_tol2 = _get_composite_ei_params(
        damping_ratio, v_tol, damping2, v_tol2)
    return _convolve_shapers(
        *get_ei_shaper(shaper_freq, damping_ratio, v_tol),
        *get_ei_shaper(f2, damping2, v_tol2), "composite_ei")

def get_composite_ei_zvd_shaper(shaper_freq, damping_ratio,
                                f2=0., damping2=0.1):
    """Convolve a low-frequency EI with a separately damped high ZVD."""
    if not math.isfinite(f2) or f2 <= 0.:
        raise ShaperError("composite_ei_zvd requires a positive f2 parameter")
    return _convolve_shapers(
        *get_ei_shaper(shaper_freq, damping_ratio),
        *get_zvd_shaper(f2, damping2), "composite_ei_zvd")

def get_composite_ei_zv_shaper(shaper_freq, damping_ratio,
                               f2=0., damping2=0.1):
    """Convolve a low-frequency EI with a separately damped high ZV."""
    if not math.isfinite(f2) or f2 <= 0.:
        raise ShaperError("composite_ei_zv requires a positive f2 parameter")
    return _convolve_shapers(
        *get_ei_shaper(shaper_freq, damping_ratio),
        *get_zv_shaper(f2, damping2), "composite_ei_zv")

def get_composite_ei_ei_shaper(shaper_freq, damping_ratio,
                               f2=0., damping2=0.1,
                               v_tol=1./SHAPER_VIBRATION_REDUCTION,
                               v_tol2=None):
    """Convolve a low-frequency EI with a separately damped high EI."""
    if not math.isfinite(f2) or f2 <= 0.:
        raise ShaperError("composite_ei_ei requires a positive f2 parameter")
    damping2, v_tol2 = _get_composite_ei_params(
        damping_ratio, v_tol, damping2, v_tol2)
    return _convolve_shapers(
        *get_ei_shaper(shaper_freq, damping_ratio, v_tol),
        *get_ei_shaper(f2, damping2, v_tol2), "composite_ei_ei")

def get_composite_zvd_zvd_shaper(shaper_freq, damping_ratio,
                                 f2=0., damping2=0.1):
    """Convolve low- and high-frequency ZVD shapers."""
    if not math.isfinite(f2) or f2 <= 0.:
        raise ShaperError("composite_zvd_zvd requires a positive f2 parameter")
    return _convolve_shapers(
        *get_zvd_shaper(shaper_freq, damping_ratio),
        *get_zvd_shaper(f2, damping2), "composite_zvd_zvd")

def get_composite_zvd_zv_shaper(shaper_freq, damping_ratio,
                                f2=0., damping2=0.1):
    """Convolve a low-frequency ZVD with a separately damped high ZV."""
    if not math.isfinite(f2) or f2 <= 0.:
        raise ShaperError("composite_zvd_zv requires a positive f2 parameter")
    return _convolve_shapers(
        *get_zvd_shaper(shaper_freq, damping_ratio),
        *get_zv_shaper(f2, damping2), "composite_zvd_zv")

def get_composite_zvd_ei_shaper(shaper_freq, damping_ratio,
                                f2=0., damping2=0.1):
    """Convolve a low-frequency ZVD with a separately damped high EI."""
    if not math.isfinite(f2) or f2 <= 0.:
        raise ShaperError("composite_zvd_ei requires a positive f2 parameter")
    return _convolve_shapers(
        *get_zvd_shaper(shaper_freq, damping_ratio),
        *get_ei_shaper(f2, damping2), "composite_zvd_ei")

# Hardware-specific weighted profiles produced by the constrained two-band
# optimizer.  They keep nine non-negative pulses and sum to one.  The
# shaper_freq/damping arguments are retained for the native interface, while
# the measured Vulcan600 pulse timing is intentionally fixed for validation.
_COMPOSITE_FIT_X_AMPLITUDES = [
    0.045510457572373576, 0.12520731748866187,
    0.08407763817595743, 0.12227822382934686,
    0.24585272614752024, 0.12227822356234884,
    0.08407763830927023, 0.12520731748525,
    0.045510457429270956,
]
_COMPOSITE_FIT_X_TIMES = [
    0.0, 0.0031036960590313627, 0.006207392118062725,
    0.019866118028641223, 0.022969814087672586,
    0.02607351014670395, 0.03973223605728245,
    0.04283593211631381, 0.04593962817534517,
]
_COMPOSITE_FIT_Y_AMPLITUDES = [
    0.055067223570137165, 0.12396603590780393,
    0.07365476692260446, 0.12385055678293193,
    0.24692284485976598, 0.12385054631659678,
    0.07365477214767573, 0.1239660354274752,
    0.055067218065008784,
]
_COMPOSITE_FIT_Y_TIMES = [
    0.0, 0.002994175683742577, 0.005988351367485154,
    0.010561733635480144, 0.01355590931922272,
    0.016550085002965297, 0.02112346727096029,
    0.024117642954702867, 0.02711181863844544,
]

def _get_weighted_profile(amplitudes, times):
    return (list(amplitudes), list(times))

def get_composite_fit_x_shaper(shaper_freq, damping_ratio, profile=1):
    if profile != 1:
        raise ShaperError("composite_fit_x supports profile=1 only")
    return _get_weighted_profile(_COMPOSITE_FIT_X_AMPLITUDES,
                                 _COMPOSITE_FIT_X_TIMES)

def get_composite_fit_y_shaper(shaper_freq, damping_ratio, profile=1):
    if profile != 1:
        raise ShaperError("composite_fit_y supports profile=1 only")
    return _get_weighted_profile(_COMPOSITE_FIT_Y_AMPLITUDES,
                                 _COMPOSITE_FIT_Y_TIMES)

def _get_shaper_from_expansion_coeffs(shaper_freq, damping_ratio, t, a):
    tau = 1. / shaper_freq
    T = []
    A = []
    n = len(a)
    k = len(a[0])
    for i in range(n):
        u = t[i][k-1]
        v = a[i][k-1]
        for j in range(k-1):
            u = u * damping_ratio + t[i][k-j-2]
            v = v * damping_ratio + a[i][k-j-2]
        T.append(u * tau)
        A.append(v)
    return (A, T)

def get_2hump_ei_shaper(shaper_freq, damping_ratio):
    t = [[0., 0., 0., 0.],
         [0.49890,  0.16270, -0.54262, 6.16180],
         [0.99748,  0.18382, -1.58270, 8.17120],
         [1.49920, -0.09297, -0.28338, 1.85710]]
    a = [[0.16054,  0.76699,  2.26560, -1.22750],
         [0.33911,  0.45081, -2.58080,  1.73650],
         [0.34089, -0.61533, -0.68765,  0.42261],
         [0.15997, -0.60246,  1.00280, -0.93145]]
    return _get_shaper_from_expansion_coeffs(shaper_freq, damping_ratio, t, a)

def get_3hump_ei_shaper(shaper_freq, damping_ratio):
    t = [[0., 0., 0., 0.],
         [0.49974,  0.23834,  0.44559, 12.4720],
         [0.99849,  0.29808, -2.36460, 23.3990],
         [1.49870,  0.10306, -2.01390, 17.0320],
         [1.99960, -0.28231,  0.61536, 5.40450]]
    a = [[0.11275,  0.76632,  3.29160, -1.44380],
         [0.23698,  0.61164, -2.57850,  4.85220],
         [0.30008, -0.19062, -2.14560,  0.13744],
         [0.23775, -0.73297,  0.46885, -2.08650],
         [0.11244, -0.45439,  0.96382, -1.46000]]
    return _get_shaper_from_expansion_coeffs(shaper_freq, damping_ratio, t, a)

# min_freq for each shaper is chosen to have projected max_accel ~= 1500
INPUT_SHAPERS = [
    InputShaperCfg(name='zv', init_func=get_zv_shaper,
                   min_freq=21., max_damping_ratio=0.99),
    InputShaperCfg(name='mzv', init_func=get_mzv_shaper,
                   min_freq=23., max_damping_ratio=0.99),
    InputShaperCfg(name='zvd', init_func=get_zvd_shaper,
                   min_freq=29., max_damping_ratio=0.99),
    InputShaperCfg(name='ei', init_func=get_ei_shaper,
                   min_freq=29., max_damping_ratio=0.4),
    InputShaperCfg(name='2hump_ei', init_func=get_2hump_ei_shaper,
                   min_freq=39., max_damping_ratio=0.3),
    InputShaperCfg(name='3hump_ei', init_func=get_3hump_ei_shaper,
                   min_freq=48., max_damping_ratio=0.2),
]

COMPOSITE_EI_CFG = InputShaperCfg(
    name='composite_ei', init_func=get_composite_ei_shaper,
    min_freq=29., max_damping_ratio=0.4)
COMPOSITE_EI_ZVD_CFG = InputShaperCfg(
    name='composite_ei_zvd', init_func=get_composite_ei_zvd_shaper,
    min_freq=29., max_damping_ratio=0.4)
COMPOSITE_EI_ZV_CFG = InputShaperCfg(
    name='composite_ei_zv', init_func=get_composite_ei_zv_shaper,
    min_freq=29., max_damping_ratio=0.4)
COMPOSITE_EI_EI_CFG = InputShaperCfg(
    name='composite_ei_ei', init_func=get_composite_ei_ei_shaper,
    min_freq=29., max_damping_ratio=0.4)
COMPOSITE_ZVD_ZVD_CFG = InputShaperCfg(
    name='composite_zvd_zvd', init_func=get_composite_zvd_zvd_shaper,
    min_freq=29., max_damping_ratio=0.99)
COMPOSITE_ZVD_ZV_CFG = InputShaperCfg(
    name='composite_zvd_zv', init_func=get_composite_zvd_zv_shaper,
    min_freq=29., max_damping_ratio=0.99)
COMPOSITE_ZVD_EI_CFG = InputShaperCfg(
    name='composite_zvd_ei', init_func=get_composite_zvd_ei_shaper,
    min_freq=29., max_damping_ratio=0.99)
COMPOSITE_FIT_X_CFG = InputShaperCfg(
    name='composite_fit_x', init_func=get_composite_fit_x_shaper,
    min_freq=1., max_damping_ratio=0.99)
COMPOSITE_FIT_Y_CFG = InputShaperCfg(
    name='composite_fit_y', init_func=get_composite_fit_y_shaper,
    min_freq=1., max_damping_ratio=0.99)
COMPOSITE_SHAPERS = [COMPOSITE_EI_CFG, COMPOSITE_EI_ZVD_CFG,
                     COMPOSITE_EI_ZV_CFG, COMPOSITE_EI_EI_CFG,
                     COMPOSITE_ZVD_ZVD_CFG, COMPOSITE_ZVD_ZV_CFG,
                     COMPOSITE_ZVD_EI_CFG, COMPOSITE_FIT_X_CFG,
                     COMPOSITE_FIT_Y_CFG]

def get_shaper_cfg(shaper_name):
    m = re.match(r"(\w+)\s*\((.*)\)$", shaper_name)
    if m:
        shaper_name = m.group(1)
    for s in COMPOSITE_SHAPERS:
        if shaper_name == s.name:
            return s
    for s in INPUT_SHAPERS:
        if shaper_name == s.name:
            return s
    return None

def init_shaper(shaper_name, shaper_freq, damping_ratio, error=None):
    try:
        m = re.match(r"(\w+)\s*\((.*)\)$", shaper_name)
        args_l = []
        args_kv = {}
        if m:
            shaper_name = m.group(1)
            args = m.group(2)
            if args:
                parsed_args = re.findall(r"(?:(\w+)\s*=\s*)?\s*([\d.]+)", args)
                def parse_val(s):
                    if '.' in s:
                        return float(s)
                    return int(s)
                args_l = [parse_val(v) for k, v in parsed_args if not k]
                args_kv = {k: parse_val(v) for k, v in parsed_args if k}
                if args_l and args_kv:
                    raise ShaperError("Mixing named and non-named shaper"
                                      " parameters is not supported")
        for s in INPUT_SHAPERS:
            if shaper_name == s.name:
                return s.init_func(shaper_freq, damping_ratio,
                                   *args_l, **args_kv)
        for s in COMPOSITE_SHAPERS:
            if shaper_name == s.name:
                return s.init_func(shaper_freq, damping_ratio,
                                   *args_l, **args_kv)
    except ShaperError as e:
        if error is None:
            raise
        raise error("Failed to initialize shaper: %s" % str(e))
    return None
