import os
import csv


class MotionLibrary:
    def __init__(self, base_dir):
        self.base_dir = base_dir
        self.library = {}
        return

    def loadMotion(self, motion):
        self.library[motion] = []

        filepath = self.base_dir + motion + ".csv"
        assert (os.path.exists(filepath))

        cur = {}

        reader = csv.DictReader(open(filepath, 'r'), delimiter=",")
        for row in reader:
            # Convert time
            t = row["#Timing"].split(":")
            t = float(t[1]) + float(t[2]) / 1000

            # Remove unnecessary fields
            del row["#Timing"]
            del row["Name"]

            delete = []
            for key in row:
                row[key] = float(row[key])
                if key not in cur or cur[key] != row[key]:
                    cur[key] = row[key]
                else:
                    delete.append(key)

            for key in delete:
                del row[key]

            self.library[motion].append((t, row))

    def getMotion(self, motion):
        assert (motion in self.library)
        return self.library[motion]


Motions = MotionLibrary("resources/motions/")
Motions.loadMotion("Backwards")
Motions.loadMotion("Forwards")
Motions.loadMotion("Shoot")
Motions.loadMotion("SideStepLeft")
Motions.loadMotion("SideStepRight")
Motions.loadMotion("StandUpFromBack")
Motions.loadMotion("TurnLeft")
Motions.loadMotion("TurnRight")
