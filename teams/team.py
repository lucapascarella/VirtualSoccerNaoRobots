from players.player import Player
from multiprocessing import Process

class Team:
    def __init__(self, num_players, team_id):
        self.players = [None] * num_players
        self.team_id = team_id
        self.score = 0

    def play(self):
        self.procs = [] 
        for player in self.players:
            # Create the process for the player
            proc = Process(target=player.behave)
            proc.start()

            self.procs.append(proc)

    def stop(self):
        for proc in self.procs:
            proc.kill()
 
    def getFormation(self, player_id):
        raise NotImplementedError

    def setPlayer(self, player_id):
        raise NotImplementedError

