class a:
    def __init__(self, val):
        self.val = val

    def do_stuff(self):
        funcs = [self.p1, self.p2, self.p3]

        for func in funcs:
            self.val = func(self.val)

        return self.val

    def p1(self, val):
        return val+1

    def p2(self, val):
        return val+2

    def p3(self, val):
        return val+3


thing = a(420)
print(thing.do_stuff())