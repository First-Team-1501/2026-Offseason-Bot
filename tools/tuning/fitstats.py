"""Statistics and fitting helpers (stdlib only): weighted least squares, jackknife CI, percentiles,
smoothing, derivatives, Nelder-Mead."""
import math

NAN = float('nan')

# two-sided 95% Student t critical values by degrees of freedom
_T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
        11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131, 16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093,
        20: 2.086, 25: 2.060, 30: 2.042, 40: 2.021, 60: 2.000, 120: 1.980}


def t95(df):
    if df <= 0:
        return math.inf
    if df in _T95:
        return _T95[df]
    keys = sorted(_T95)
    for a, b in zip(keys, keys[1:]):
        if a < df < b:
            return _T95[a] + (_T95[b] - _T95[a]) * (df - a) / (b - a)
    return 1.96


def finite(x):
    return x is not None and isinstance(x, (int, float)) and math.isfinite(x)


def clean(xs):
    return [x for x in xs if finite(x)]


def mean(xs):
    xs = clean(xs)
    return sum(xs) / len(xs) if xs else NAN


def wmean(xs, ws):
    s = sw = 0.0
    for x, w in zip(xs, ws):
        if finite(x) and finite(w):
            s += x * w
            sw += w
    return s / sw if sw > 0 else NAN


def stdev(xs):
    xs = clean(xs)
    if len(xs) < 2:
        return NAN
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def rms(xs):
    xs = clean(xs)
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else NAN


def percentile(xs, q):
    """Linear-interpolated percentile, q in [0, 100]."""
    xs = sorted(clean(xs))
    if not xs:
        return NAN
    if len(xs) == 1:
        return xs[0]
    pos = (len(xs) - 1) * q / 100.0
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def median(xs):
    return percentile(xs, 50)


def solve(A, b):
    """Gaussian elimination with partial pivoting. Returns None if singular."""
    n = len(A)
    M = [list(A[i]) + [b[i]] for i in range(n)]
    for i in range(n):
        piv = max(range(i, n), key=lambda k: abs(M[k][i]))
        if abs(M[piv][i]) < 1e-12:
            return None
        M[i], M[piv] = M[piv], M[i]
        for k in range(n):
            if k != i:
                f = M[k][i] / M[i][i]
                if f:
                    M[k] = [a - f * c for a, c in zip(M[k], M[i])]
    return [M[i][n] / M[i][i] for i in range(n)]


def wls(X, y, w=None):
    """Weighted least squares. X: rows of regressors, y: targets, w: weights.
    Returns dict(beta, r2, rmse, n, dof) or None when singular / too few rows."""
    n = len(y)
    if n == 0:
        return None
    p = len(X[0])
    if n < p:
        return None
    w = w or [1.0] * n
    A = [[0.0] * p for _ in range(p)]
    b = [0.0] * p
    for row, yi, wi in zip(X, y, w):
        for i in range(p):
            ri = row[i] * wi
            b[i] += ri * yi
            Ai = A[i]
            for j in range(p):
                Ai[j] += ri * row[j]
    beta = solve(A, b)
    if beta is None:
        return None
    sw = sum(w)
    ym = sum(wi * yi for wi, yi in zip(w, y)) / sw
    ssr = sst = 0.0
    for row, yi, wi in zip(X, y, w):
        f = sum(bi * xi for bi, xi in zip(beta, row))
        ssr += wi * (yi - f) ** 2
        sst += wi * (yi - ym) ** 2
    r2 = 1.0 - ssr / sst if sst > 0 else NAN
    return {'beta': beta, 'r2': r2, 'rmse': math.sqrt(ssr / sw), 'n': n, 'dof': n - p}


def jackknife(groups, fit):
    """Delete-one-group jackknife. groups: list of row lists; fit(rows) -> list of parameters or None.
    Returns (estimate, standard errors, ci95 half-widths, number of groups used)."""
    allrows = [r for g in groups for r in g]
    full = fit(allrows)
    if full is None:
        return None, None, None, 0
    g = len(groups)
    if g < 3:
        return full, [NAN] * len(full), [NAN] * len(full), g
    reps = []
    for k in range(g):
        rows = [r for i, grp in enumerate(groups) if i != k for r in grp]
        est = fit(rows)
        if est is not None:
            reps.append(est)
    m = len(reps)
    if m < 3:
        return full, [NAN] * len(full), [NAN] * len(full), m
    se = []
    for j in range(len(full)):
        mj = sum(r[j] for r in reps) / m
        se.append(math.sqrt((m - 1) / m * sum((r[j] - mj) ** 2 for r in reps)))
    t = t95(m - 1)
    return full, se, [t * s for s in se], m


def linfit(xs, ys, ws=None):
    """y = a + b x. Returns (a, b) or (nan, nan)."""
    ws = ws or [1.0] * len(xs)
    sw = sum(ws)
    if sw <= 0 or len(xs) < 2:
        return NAN, NAN
    mx = sum(w * x for w, x in zip(ws, xs)) / sw
    my = sum(w * y for w, y in zip(ws, ys)) / sw
    sxx = sum(w * (x - mx) ** 2 for w, x in zip(ws, xs))
    sxy = sum(w * (x - mx) * (y - my) for w, x, y in zip(ws, xs, ys))
    if sxx <= 0:
        return my, NAN
    b = sxy / sxx
    return my - b * mx, b


def pearson(xs, ys):
    pts = [(x, y) for x, y in zip(xs, ys) if finite(x) and finite(y)]
    if len(pts) < 3:
        return NAN
    mx = sum(p[0] for p in pts) / len(pts)
    my = sum(p[1] for p in pts) / len(pts)
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    if sxx <= 0 or syy <= 0:
        return NAN
    return sxy / math.sqrt(sxx * syy)


def moving_mean(xs, half):
    """Centered moving mean over +-half samples (NaN-aware)."""
    n = len(xs)
    out = [NAN] * n
    pre = [0.0] * (n + 1)
    cnt = [0] * (n + 1)
    for i, x in enumerate(xs):
        ok = finite(x)
        pre[i + 1] = pre[i] + (x if ok else 0.0)
        cnt[i + 1] = cnt[i] + (1 if ok else 0)
    for i in range(n):
        a = max(0, i - half)
        b = min(n, i + half + 1)
        c = cnt[b] - cnt[a]
        if c:
            out[i] = (pre[b] - pre[a]) / c
    return out


def central_diff(xs, dt, half):
    """Derivative by central difference over +-half samples."""
    n = len(xs)
    out = [NAN] * n
    for i in range(half, n - half):
        a, b = xs[i - half], xs[i + half]
        if finite(a) and finite(b):
            out[i] = (b - a) / (2 * half * dt)
    return out


def rolling_slope(xs, dt, k):
    """Least-squares slope over windows of k samples starting at each index (prefix sums). Returns list of
    length n - k + 1 (NaN where a window contains NaN)."""
    n = len(xs)
    if n < k or k < 2:
        return []
    ts = [i * dt for i in range(n)]
    s1 = [0.0] * (n + 1)
    st = [0.0] * (n + 1)
    bad = [0] * (n + 1)
    for i, x in enumerate(xs):
        ok = finite(x)
        s1[i + 1] = s1[i] + (x if ok else 0.0)
        st[i + 1] = st[i] + (x * ts[i] if ok else 0.0)
        bad[i + 1] = bad[i] + (0 if ok else 1)
    out = []
    tsum_k = dt * k * (k - 1) / 2.0
    tt_k = dt * dt * (k - 1) * k * (2 * k - 1) / 6.0
    den = k * tt_k - tsum_k * tsum_k
    for i in range(n - k + 1):
        if bad[i + k] - bad[i]:
            out.append(NAN)
            continue
        sy = s1[i + k] - s1[i]
        sty = st[i + k] - st[i] - ts[i] * sy  # shift time origin to window start
        out.append((k * sty - tsum_k * sy) / den)
    return out


def nelder_mead(f, x0, step, iters=200, tol=1e-9):
    n = len(x0)
    pts = [list(x0)] + [[x0[j] + (step[j] if j == i else 0.0) for j in range(n)] for i in range(n)]
    vals = [f(p) for p in pts]
    for _ in range(iters):
        order = sorted(range(n + 1), key=lambda i: vals[i])
        pts = [pts[i] for i in order]
        vals = [vals[i] for i in order]
        if abs(vals[-1] - vals[0]) < tol:
            break
        c = [sum(p[j] for p in pts[:-1]) / n for j in range(n)]
        xr = [c[j] + (c[j] - pts[-1][j]) for j in range(n)]
        fr = f(xr)
        if fr < vals[0]:
            xe = [c[j] + 2 * (c[j] - pts[-1][j]) for j in range(n)]
            fe = f(xe)
            pts[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            pts[-1], vals[-1] = xr, fr
        else:
            xc = [c[j] + 0.5 * (pts[-1][j] - c[j]) for j in range(n)]
            fc = f(xc)
            if fc < vals[-1]:
                pts[-1], vals[-1] = xc, fc
            else:
                for i in range(1, n + 1):
                    pts[i] = [pts[0][j] + 0.5 * (pts[i][j] - pts[0][j]) for j in range(n)]
                    vals[i] = f(pts[i])
    i = min(range(n + 1), key=lambda k: vals[k])
    return pts[i], vals[i]


def runs(mask):
    """[(start, end_exclusive)] of consecutive True values."""
    out = []
    start = None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        elif not m and start is not None:
            out.append((start, i))
            start = None
    if start is not None:
        out.append((start, len(mask)))
    return out


def fmt(x, nd=2, unit='', none='-'):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return none
    return ('%.' + str(nd) + 'f%s') % (x, unit)
