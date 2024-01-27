# Hack to get color picker inside vscode
class rgb:
    def __init__(self, r, g, b):
        self.r = r
        self.g = g
        self.b = b

    def __str__(self):
        return f"rgb({self.r}, {self.g}, {self.b})"

    def __repr__(self):
        return f"rgb({self.r}, {self.g}, {self.b})"

    def as_bgr(self) -> tuple:
        return (self.b, self.g, self.r)