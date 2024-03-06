from players.player import Player


class PlayerSimple(Player):
    def __init__(self):
        Player.__init__(self)

    def behave(self):
        for i in range(5):
            self.move("Forwards")
        for i in range(5):
            self.move("Backwards")
