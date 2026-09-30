"""SplitMix64 generator with the numpy.random.Generator methods the simulator
uses. It defines the exact streams in docs/core-schema.md section 4 so that
Python fixtures and lcm-core consume identical random numbers."""
import math

MASK = (1 << 64) - 1
TWO_POW_M53 = 1.0 / 9007199254740992.0


class PortableRng:
    def __init__(self, seed=0):
        self.state = int(seed) & MASK

    def next_u64(self):
        self.state = (self.state + 0x9E3779B97F4A7C15) & MASK
        z = self.state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK
        return z ^ (z >> 31)

    def random(self):
        return (self.next_u64() >> 11) * TWO_POW_M53

    def _index(self, n):
        threshold = ((1 << 64) - n) % n
        while True:
            v = self.next_u64()
            if v >= threshold:
                return v % n

    def integers(self, low, high=None):
        if high is None:
            low, high = 0, low
        return low + self._index(high - low)

    def uniform(self, low=0.0, high=1.0, size=None):
        if size is None:
            return low + (high - low) * self.random()
        count = size[0] if isinstance(size, tuple) else size
        return [low + (high - low) * self.random() for _ in range(count)]

    def exponential(self, scale=1.0, size=None):
        if size is None:
            return -scale * math.log1p(-self.random())
        return [-scale * math.log1p(-self.random()) for _ in range(size)]

    def shuffle(self, xs):
        for i in range(len(xs) - 1, 0, -1):
            j = self._index(i + 1)
            xs[i], xs[j] = xs[j], xs[i]

    def choice(self, a, size, replace=False):
        assert not replace and isinstance(a, int)
        pool = list(range(a))
        for i in range(size):
            j = i + self._index(a - i)
            pool[i], pool[j] = pool[j], pool[i]
        return pool[:size]
