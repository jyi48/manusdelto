"""Shared exponential moving-average filter used by ergo and ik retargeters.

dex is excluded — dex-retargeting's own SeqRetargeting already applies an
internal low-pass (config's low_pass_alpha), so a second EMA on top would be
redundant.
"""


class EMAFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self._prev = None

    def filter(self, vals):
        if self._prev is None:
            self._prev = list(vals)
            return list(vals)
        out = [self.alpha * v + (1.0 - self.alpha) * p for v, p in zip(vals, self._prev)]
        self._prev = out
        return out

    def reset(self):
        self._prev = None
