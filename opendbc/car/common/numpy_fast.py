import numpy as np
from numbers import Number

def clip(x, lo, hi):
  np_ = np.clip(x, lo, hi)
  a = max(lo, min(hi, x))
  if np_ == None or not isinstance(np_, Number) or a != np_:
    with open('/tmp/debug_out_o','w') as fp:
      fp.write("clip:%f, %f,%f,%f" % (a , x,lo,hi))
    return a
  else:
    return np_


def interp(x, xp, fp):
  np_ = np.interp(x, xp, fp)
  N = len(xp)

  def get_interp(xv):
    hi = 0
    while hi < N and xv > xp[hi]:
      hi += 1
    low = hi - 1
    return fp[-1] if hi == N and xv > xp[low] else (
      fp[0] if hi == 0 else
      (xv - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low])

  a = [get_interp(v) for v in x] if hasattr(x, '__iter__') else get_interp(x)
  if np_ == None or not isinstance(np_, Number) or a != np_:
    with open('/tmp/debug_out_v','w') as fp:
      fp.write("interp:%f" % (a))
    return a
  else:
    return np_


def mean(x):
  return np.mean(x)
  return sum(x) / len(x)

