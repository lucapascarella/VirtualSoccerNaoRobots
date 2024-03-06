from multiprocessing import Process


class Team:
    def __init__(self, num_players, team_id):
        self.players = [None] * num_players
        self.team_id = team_id
        self.score = 0
        self.procs = []

    def play(self):
        for player in self.players:
            # Create the process for the player
            proc = Process(target=player.behave)
            proc.start()

            self.procs.append(proc)

    def stop(self):
        for proc in self.procs:
            proc.kill()

    def get_formation(self, player_id):
        raise NotImplementedError

    def set_player(self, player_id):
        raise NotImplementedError
