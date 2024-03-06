from players.player import Player
import cv2


class PlayerSimple(Player):
    def __init__(self):
        Player.__init__(self)

    def behave(self):
        for i in range(10):
            self.move("Forwards")
        for i in range(10):
            self.move("Backwards")
