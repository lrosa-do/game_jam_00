# Story regression run: copy to Assets/Scripts/zz_play_test.py, add a Node "ZTest" with it to a copy
# of Scenes/game.scene.json (Scenes/zz_test.scene.json) and run headless for 14000 frames (the waits of the story are in real seconds; a fast machine spends more frames on them); it
# prints each stage, whether its objective can be walked to, and MARK FINISHED at the end.
import math

# The kind of each stage of story.STAGES.
KINDS = ["training", "enter", "card", "hack", "reach", "transports", "upload", "free", "escort"]

# Runs the story stage by stage: checks each objective can be walked to from the last one and
# drives it to the end (teleports, kills the sergeant, holds E, blows the targets).

class Test(Node):
    frame = 0
    stage = -1
    since = 0
    last = None

    def reach(self, nav, a, b, label):
        path = nav.find_path(a, b)
        end = path[len(path) - 1] if len(path) > 0 else None
        gap = math.sqrt((end.x - b.x) * (end.x - b.x) + (end.z - b.z) * (end.z - b.z)) if end != None else 99.0
        print("MARK reach", label, "corners", len(path), "end gap", int(gap * 100.0) / 100.0, "OK" if gap < 2.5 else "UNREACHABLE")

    def go(self, player, target):
        nav = self.get_parent().nav
        spot = nav.nearest_point(target)
        if spot == None:
            spot = target
        player.global_position = Vec3(spot.x, spot.y + 0.9, spot.z)

    def _process(self, delta):
        self.frame += 1
        game = self.get_parent()
        story = game.story
        player = game.get_node("Player")
        player._health = 100000.0
        game._brief_state = 3
        if story == None or story._stage < 0:
            return
        story.autoplay = True
        if story._done:
            if self.since >= 0:
                print("MARK FINISHED at frame", self.frame, "score", game._score, "kills", game._kills)
                self.since = -1000000
            return
        if story._stage != self.stage:
            self.stage = story._stage
            self.since = 0
            print("MARK stage", self.stage, "frame", self.frame, game.mission, "target", story._target, story._target_label, "needed", story._needed, "count", story._count)
            if story._target != None and self.last != None:
                self.reach(game.nav, self.last, story._target, game.mission)
            if story._target != None:
                self.last = story._target
        self.since += 1
        kind = KINDS[self.stage]
        if self.since == 20:
            k = game.mission
            if kind == "training":
                story._step = 6
                car = story._items[0]
                print("MARK test car", car.get_parent().name, car.center())
                car.hit(car.center(), Vec3(0.0, 1.0, 0.0), Vec3(0.0, -1.0, 0.0), 999.0)
            elif kind == "card":
                sergeant = story._items[0]
                sergeant.die(Vec3(0.0, 0.0, 1.0), 1.0)
            elif kind == "transports":
                for car in story._items:
                    car.hit(car.center(), Vec3(0.0, 1.0, 0.0), Vec3(0.0, -1.0, 0.0), 999.0)
            elif kind == "free":
                cell = story._cell
                print("MARK cell", cell.get_parent().name, "locked", cell.locked, "unit", story._unit.global_position)
                self.go(player, story._target)
            elif story._target != None:
                self.go(player, story._target)
                if kind == "escort" and story._unit != None:
                    p = player.global_position
                    story._unit.global_position = Vec3(p.x + 1.0, p.y, p.z)
        if self.since == 40 and kind == "card" and story._target != None:
            self.reach(game.nav, player.global_position, story._target, "card")
            self.go(player, story._target)
        if self.since > 60 and kind == "transports" and (self.since % 60) == 0:
            print("MARK transports", story._count, "/", story._needed)
        if kind == "free" and self.since > 20 and (self.since % 50) == 0 and self.since < 400:
            print("MARK free", self.since, "unit_free", story.unit_free, "breakout", story._breakout, "cell broken", story._cell._broken)
        if self.since == 6000:
            print("MARK STUCK in stage", self.stage, game.mission)
