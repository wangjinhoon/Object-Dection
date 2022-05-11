def plus(cls, *args):
    return sum(args) + cls.a


class Basic:
    def __init__(self, f):
        self.a = 10
        print(f(self, 0, 0, 0))


basic = Basic(plus)