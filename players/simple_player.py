from players.player import Player


class SimplePlayer(Player):
    def __init__(self):
        Player.__init__(self)

    def behave(self):
        for i in range(10):
            self.move("Forwards")
        for i in range(10):
            self.move("Backwards")
        return
