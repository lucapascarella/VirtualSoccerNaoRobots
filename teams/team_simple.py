from teams.team import Team
from players.player_simple import PlayerSimple
from math import ceil


class TeamSimple(Team):
    def __init__(self, num_players, team_id):
        Team.__init__(self, num_players, team_id)

    def get_formation(self, player_id):
        return 0, float(player_id)

    def set_player(self, player_id):
        self.players[player_id] = PlayerSimple()
