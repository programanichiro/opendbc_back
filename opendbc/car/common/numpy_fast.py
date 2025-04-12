import numpy as np
from numbers import Number

def clip(x, lo, hi):
    np_ = np.clip(x, lo, hi)
    return float(np_)


def interp(x, xp, fp):
  np_ = np.interp(x, xp, fp)
  if isinstance(np_, Number):
    return float(np_)
  else:
    return np_


def mean(x):
  return np.mean(x)

