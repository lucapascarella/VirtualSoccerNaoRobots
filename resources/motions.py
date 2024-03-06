import os
import csv


class MotionLibrary:
    def __init__(self, base_dir):
        self.base_dir = base_dir
        self.library = {}
        return

    def load_motion(self, motion):
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

    def get_motion(self, motion):
        assert (motion in self.library)
        return self.library[motion]


motions = MotionLibrary("resources/motions/")
motions.load_motion("Backwards")
motions.load_motion("Forwards")
motions.load_motion("Shoot")
motions.load_motion("SideStepLeft")
motions.load_motion("SideStepRight")
motions.load_motion("StandUpFromBack")
motions.load_motion("TurnLeft")
motions.load_motion("TurnRight")
